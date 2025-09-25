# This file is part of ts_mthexapod.
#
# Developed for the Rubin Observatory Telescope and Site System.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

__all__ = ["HexapodCsc", "run_mthexapod"]

import asyncio
import contextlib
import copy
import dataclasses
import math
import types
import typing
from pathlib import Path

import numpy as np
from lsst.ts import hexrotcomm, salobj
from lsst.ts.utils import make_done_future
from lsst.ts.xml.enums.MTHexapod import (
    ApplicationStatus,
    ControllerState,
    EnabledSubstate,
    ErrorCode,
)

from . import (
    __version__,
    base,
    compensation,
    constants,
    enums,
    mock_controller,
    structs,
    utils,
)
from .config_schema import CONFIG_SCHEMA
from .simple_hexapod import SimpleHexapod

# Maximum time to stop axes (seconds).
# The minimum needed is time to acquire the write lock,
# issue the command, and wait for motion to stop.
# However, it is fine to be generous.
MAXIMUM_STOP_TIME = 10

# Maximum value of the n_telemetry counter.
# Sets the maximum number of telemetry messages one can wait for.
MAX_N_TELEMETRY = 10


class CompensationInfo:
    """Information about a possibly compensated move

    Parameters
    ----------
    uncompensated_pos : `Position`
        Target position (without compensation applied).
    compensation_inputs : `CompensationInputs` or `None`
        The compensation inputs, if compensating and all inputs
        are available, else `None`.
    compensation_offset : `Position` or None.
        Amount of compensation, if relevant, else None.

    Attributes
    ----------
    compensated_pos : `Position`
        Compensated position: uncompensated_pos + compensation_offset
        if compensation_offset is not None, else uncompensated_pos.

    Notes
    -----
    All the parameters are also attributes.
    """

    def __init__(
        self,
        uncompensated_pos: base.Position,
        compensation_inputs: base.CompensationInputs | None,
        compensation_offset: base.Position | None,
    ) -> None:
        self.uncompensated_pos = uncompensated_pos
        self.compensation_inputs = compensation_inputs
        self.compensation_offset = compensation_offset
        if compensation_offset is None:
            self.compensated_pos = uncompensated_pos
        else:
            self.compensated_pos = uncompensated_pos + compensation_offset


class HexapodCsc(hexrotcomm.BaseCsc):
    """MTHexapod CSC.

    Parameters
    ----------
    index : `SalIndex` or `int`
        SAL index; see `SalIndex` for the allowed values.
    config_dir : `str` or `pathlib.Path` or None, optional
        Directory of configuration files, or None for the standard
        configuration directory (obtained from `_get_default_config_dir`).
        This is provided for unit testing.
    initial_state : `lsst.ts.salobj.State` or `int` (optional)
        The initial state of the CSC.
        Must be `lsst.ts.salobj.State.STANDBY` unless simulating
        (``simulation_mode != 0``).
    override : `str`, optional
        Configuration override file to apply if ``initial_state`` is
        `State.DISABLED` or `State.ENABLED`.
    simulation_mode : `int` (optional)
        Simulation mode. Allowed values:

        * 0: regular operation.
        * 1: simulation: use a mock low level controller.

    Raises
    ------
    ValueError
        If ``initial_state != lsst.ts.salobj.State.STANDBY``
        and not simulating (``simulation_mode = 0``).

    Notes
    -----
    **Error Codes**

    See `lsst.ts.xml.enums.MTHexapod.ErrorCode`
    """

    valid_simulation_modes = [0, 1]
    version = __version__

    def __init__(
        self,
        index: int,
        config_dir: str | Path | None = None,
        initial_state: salobj.State = salobj.State.STANDBY,
        override: str = "",
        simulation_mode: int = 0,
    ) -> None:
        index = enums.SalIndex(index)
        controller_constants = constants.IndexControllerConstants[index]
        self.subconfig_name = controller_constants.subconfig_name

        # The maximum position limits configured in the low-level controller.
        # The limits specified by the configureLimits command must be
        # within these limits.
        self.max_pos_limits = constants.MAX_POSITION_LIMITS[index]

        # Current position limits; initialize to max limits,
        # but update from configuration reported by the low-level controller.
        self.current_pos_limits = copy.copy(self.max_pos_limits)

        # The move command that is currently running, or None.
        self.move_command = None

        # Task for the current move. To cancel safely::
        #
        #     async with self.write_lock:
        #        self.compensation_loop_task.cancel()
        self.move_task = make_done_future()

        # Task to do the movement in steps.
        self.move_steps_task = make_done_future()

        # Events used by the self.move_steps_task
        self.move_steps_in_position_event = asyncio.Event()
        self.move_steps_start_event = asyncio.Event()

        # Event that is set when a move or offset command is received.
        # This is intended for unit tests, which may clear the event
        # and wait for it to be set to know the command has been received.
        self.move_command_received_event = asyncio.Event()

        # Compensation loop task. To cancel safely::
        #
        #     async with self.write_lock:
        #        self.compensation_loop_task.cancel()
        self.compensation_loop_task = make_done_future()
        self.initial_compensation_offset_applied = False

        # Task for the camera filter monitor.
        self.camera_filter_monitor_task = make_done_future()

        # Task to monitor the hexapod is in idle or not under the enabled
        # state.
        self.idle_time_monitor_task = make_done_future()

        # Record missing compensation inputs we have warned about,
        # to avoid duplicate warnings.
        self.bad_inputs_str = ""

        # Interval between compensation updates.
        # Set in `configure`, but we need an initial value.
        self.compensation_interval = 0.2

        # Maximum time for the longest possible move (sec) with some buffer.
        # Set in `config_callback` but we need an initial value.
        # 61.74 is the value for the mock controller as of 2021-04-15.
        self.max_move_duration = 65

        # Asynchronous lock for the idle time in the enabled state.
        self._lock_idle_time = asyncio.Lock()
        self._idle_time_in_enabled_state = 0.0

        # Event set when a telemetry message is received from
        # the low-level controller, after it has been parsed.
        self.telemetry_event = asyncio.Event()

        # Count of number of telemetry samples read
        # since issuing a low-level command.
        # Maxes out at MAX_N_TELEMETRY.
        self.n_telemetry = 0

        # Initialize filter offset
        zero_offset = types.SimpleNamespace(
            x=0,
            y=0,
            z=0,
            u=0,
            v=0,
            w=0,
        )
        self.filter_offset = base.Position.from_struct(zero_offset)

        # Set the pivot
        pivot = (
            constants.PIVOT_CAMERA
            if (index == enums.SalIndex.CAMERA_HEXAPOD)
            else constants.PIVOT_M2
        )
        self._pivot = np.array(pivot) * utils.UM_TO_M

        # Set the base and mirror positions for the hexapod
        base_positions = (
            constants.ACTUATOR_BASE_POSITIONS_CAMERA
            if index == enums.SalIndex.CAMERA_HEXAPOD
            else constants.ACTUATOR_BASE_POSITIONS_M2
        )
        mirror_positions = (
            constants.ACTUATOR_MIRROR_POSITIONS_CAMERA
            if index == enums.SalIndex.CAMERA_HEXAPOD
            else constants.ACTUATOR_MIRROR_POSITIONS_M2
        )

        self._base_positions = np.array(base_positions).T * utils.UM_TO_M
        self._mirror_positions = np.array(mirror_positions).T * utils.UM_TO_M

        super().__init__(
            name="MTHexapod",
            index=index,
            CommandCode=enums.CommandCode,
            ConfigClass=structs.Config,
            TelemetryClass=structs.Telemetry,
            config_schema=CONFIG_SCHEMA,
            config_dir=config_dir,
            initial_state=initial_state,
            override=override,
            simulation_mode=simulation_mode,
        )

        self.mtmount = salobj.Remote(
            domain=self.domain,
            name="MTMount",
            include=["target", "elevation", "azimuth"],
        )
        self.mtrotator = salobj.Remote(
            domain=self.domain, name="MTRotator", include=["target", "rotation"]
        )

        # See the ts_config_ocs/ESS
        # 121 and 122 (= 120 + index.value) are defined in
        # ts_config_ocs/ESS/v8/_init.yaml
        self.ess_temperature = salobj.Remote(
            domain=self.domain,
            name="ESS",
            index=(120 + index.value),
            include=["temperature"],
        )

    @property
    def host(self) -> str:
        return getattr(self.config, self.subconfig_name)["host"]

    @property
    def port(self) -> int:
        return getattr(self.config, self.subconfig_name)["port"]

    @property
    def compensation_mode(self) -> bool:
        """Return True if moves are compensated, False otherwise."""
        return self.evt_compensationMode.data.enabled

    @property
    def enable_lut_temperature(self) -> bool:
        """Enable the LUT temperature compensation or not.

        Returns
        -------
        `bool`
            True if the LUT temperature compensation is enabled, False
            otherwise.
        """
        return getattr(self.config, self.subconfig_name)["enable_lut_temperature"]

    @property
    def no_movement_idle_time(self) -> float:
        """Time limit for no movement in seconds under the Enabled state.

        Returns
        -------
        `float`
            Time limit in seconds.
        """
        return getattr(self.config, self.subconfig_name)["no_movement_idle_time"]

    @property
    def step_size_xy(self) -> float:
        """Absolute maximum step size in the x, y direction in um. 0 means to
        do the movement in a single step.

        Returns
        -------
        `float`
            Step size xy in um.
        """
        return getattr(self.config, self.subconfig_name)["step_size_xy"]

    @property
    def step_size_z(self) -> float:
        """Absolute maximum step size in the z direction in um. 0 means to do
        the movement in a single step.

        Returns
        -------
        `float`
            Step size z in um.
        """
        return getattr(self.config, self.subconfig_name)["step_size_z"]

    @property
    def step_size_uv(self) -> float:
        """Absolute maximum step size in the rx, ry rotation in deg. 0 means to
        do the movement in a single step.

        Returns
        -------
        `float`
            Step size uv in deg.
        """
        return getattr(self.config, self.subconfig_name)["step_size_uv"]

    @property
    def step_size_w(self) -> float:
        """Absolute maximum step size in the rz rotation in deg. 0 means to do
        the movement in a single step.

        Returns
        -------
        `float`
            Step size w in deg.
        """
        return getattr(self.config, self.subconfig_name)["step_size_w"]

    async def config_callback(self, client: hexrotcomm.CommandTelemetryClient) -> None:
        """Called when the low-level controller outputs configuration.

        Parameters
        ----------
        client : `lsst.ts.hexrotcomm.CommandTelemetryClient`
            TCP/IP client.
        """

        configuration = dict(
            maxXY=client.config.pos_limits[0],
            minZ=client.config.pos_limits[1],
            maxZ=client.config.pos_limits[2],
            maxUV=client.config.pos_limits[3],
            minW=client.config.pos_limits[4],
            maxW=client.config.pos_limits[5],
            maxVelocityXY=client.config.vel_limits[0],
            maxVelocityZ=client.config.vel_limits[1],
            maxVelocityUV=client.config.vel_limits[2],
            maxVelocityW=client.config.vel_limits[3],
            pivotX=client.config.pivot[0],
            pivotY=client.config.pivot[1],
            pivotZ=client.config.pivot[2],
            maxDisplacementStrut=client.config.max_displacement_strut,
            maxVelocityStrut=client.config.max_velocity_strut,
            accelerationStrut=client.config.acceleration_strut,
            drivesEnabled=client.config.drives_enabled,
        )

        await self.evt_configuration.set_write(**configuration)

        # Update the internal pivot
        for idx in range(3):
            self._pivot[idx] = client.config.pivot[idx] * utils.UM_TO_M

        self.current_pos_limits = base.PositionLimits.from_struct(
            self.evt_configuration.data
        )

        # Compute a simplistic model for the maximum duration of a move (sec).
        # Ignore the distance over which the strut accelerates
        # and decelerates, when estimating the time spent at maximum velocity,
        # as that should be negligible.
        # Note that max_displacement_strut is a half distance (+/- from 0).
        if client.config.max_displacement_strut <= 0:
            self.log.warning(
                f"Reported max_displacement_strut {client.config.max_displacement_strut} <= 0; "
                f"using existing max_move_duration {self.max_move_duration} sec"
            )
            return
        if client.config.max_velocity_strut <= 0:
            self.log.warning(
                f"Reported max_velocity_strut {client.config.acceleration_strut} <= 0; "
                f"using existing max_move_duration {self.max_move_duration} sec"
            )
            return
        if client.config.acceleration_strut <= 0:
            self.log.warning(
                f"Reported strut acceleration {client.config.acceleration_strut} <= 0; "
                f"using existing max_move_duration {self.max_move_duration} sec"
            )
            return

        telemetry_interval = 0.2
        accel_duration = (
            client.config.max_velocity_strut / client.config.acceleration_strut
        )
        vel_duration = (
            client.config.max_displacement_strut / client.config.max_velocity_strut
        )
        self.max_move_duration = 2.1 * (
            telemetry_interval + accel_duration + vel_duration
        )
        self.log.info(f"max_move_duration={self.max_move_duration}")

    async def configure(self, config: types.SimpleNamespace) -> None:
        self.log.info("Configure started...")
        await super().configure(config)
        self.compensation_interval = config.compensation_interval
        subconfig = types.SimpleNamespace(**getattr(config, self.subconfig_name))
        self.min_compensation_adjustment = np.array(
            subconfig.min_compensation_adjustment, dtype=float
        )
        self.compensation = compensation.Compensation(
            elevation_coeffs=subconfig.elevation_coeffs,
            azimuth_coeffs=subconfig.azimuth_coeffs,
            rotation_coeffs=subconfig.rotation_coeffs,
            temperature_coeffs=subconfig.temperature_coeffs,
            min_temperature=subconfig.min_temperature,
            max_temperature=subconfig.max_temperature,
        )
        if not self.camera_filter_monitor_task.done():
            self.log.info("Camera filter monitor task still running. Restarting.")
            self.camera_filter_monitor_task.cancel()
            try:
                await self.camera_filter_monitor_task
            except asyncio.CancelledError:
                pass

        if hasattr(subconfig, "camera"):
            self.log.info("Starting camera filter monitor task.")
            self.filter_offsets_dict = subconfig.filter_offsets
            self.camera_filter_monitor_task = asyncio.create_task(
                self.monitor_camera_filter(subconfig.camera)
            )

        self.log.info(
            f"Enable LUT temperature: {self.enable_lut_temperature}. Configure done."
        )

    async def monitor_camera_filter(self, camera: str) -> None:
        """Monitor the camera filter and apply compensation as needed.

        Parameters
        ----------
        camera : `str`
            Camera name.
        """
        self.log.info(f"Starting camera filter monitor for {camera}.")

        try:
            async with salobj.Remote(
                domain=self.domain,
                name=camera,
                readonly=True,
                include=["endSetFilter"],
            ) as camera_remote:
                camera_remote.evt_endSetFilter.flush()
                try:
                    end_set_filter = await camera_remote.evt_endSetFilter.aget(
                        timeout=MAXIMUM_STOP_TIME
                    )
                    current_filter = end_set_filter.filterName
                    self.log.info(f"Initial camera filter {current_filter}.")
                    await self.handle_camera_filter(current_filter)
                except asyncio.TimeoutError:
                    self.log.warning("No initial camera filter information. Ignoring.")

                self.log.info("Camera filter monitor loop starting.")
                while self.disabled_or_enabled:
                    end_set_filter = await camera_remote.evt_endSetFilter.next(
                        flush=False
                    )
                    if current_filter != end_set_filter.filterName:
                        self.log.info(
                            f"Updating camera filter {current_filter} -> {end_set_filter.filterName}."
                        )
                        current_filter = end_set_filter.filterName
                        await self.handle_camera_filter(current_filter)
                    else:
                        self.log.info(f"Already in {current_filter}.")
                self.log.info("Camera filter monitor ending.")
        except Exception as e:
            self.log.exception("Error in camera filter monitor.")
            async with self.csc_level_fault():
                await self.fault(
                    code=ErrorCode.CAMERA_FILTER_MONITOR_FAULT.value,
                    report=f"Error in camera filter monitor: {e!r}",
                )

    async def compensation_loop(self) -> None:
        """Apply compensation at regular intervals.

        The algorithm is to repeat the following sequence:

        * Wait until it is time to apply compensation
          (see `compensation_wait` for details).
        * Compute the compensation offset and apply it if large enough.

        If a compensation update fails then compensation is disabled.
        """
        if not self._has_uncompensated_position():
            self.log.error("Compensation failed; no position has been commanded")
            return

        while self.summary_state == salobj.State.ENABLED:
            try:
                await self.compensation_wait()
                self.log.debug("Apply compensation")
                uncompensated_pos = self._get_uncompensated_position()
                await self._move(
                    uncompensated_pos=uncompensated_pos,
                    sync=True,
                    is_compensation_loop=True,
                    overwrite_step_size_from_config=True,
                )
            except Exception:
                self.log.exception("Compensation failed; CSC going to Fault.")
                async with self.csc_level_fault():
                    await self.fault(
                        code=ErrorCode.COMPENSATION_FAULT.value,
                        report="Compensation failed.",
                    )
                return

    async def compensation_wait(self) -> None:
        """Wait until it is time to apply the next compensation update.

        Wait for motion to stop, then wait for self.compensation_interval
        seconds (a configuration parameter).
        Warn and repeat if the axes are moving again (very unlikely).

        Raises
        ------
        asyncio.CancelledError
            If the CSC is no longer enabled.
        asyncio.TimeoutError
            If waiting for motion to stop takes longer than
            ``self.max_move_duration``.
        """
        while True:
            await asyncio.wait_for(self.wait_stopped(), timeout=self.max_move_duration)

            await asyncio.sleep(self.compensation_interval)
            if self.summary_state != salobj.State.ENABLED:
                raise asyncio.CancelledError()

            if self.client.telemetry.enabled_substate == EnabledSubstate.STATIONARY:
                # Axes are still halted; we're done!
                return
            else:
                # This should never happen, because any new move cancels
                # the compensation loop. But just in case...
                self.log.warning(
                    "Found the hexapod moving after waiting the "
                    "compensation interval; waiting again."
                )

    def compute_compensation(
        self, uncompensated_pos: base.Position
    ) -> CompensationInfo:
        """Check uncompensated and, if relevant, compensated position
        and return compensation information.

        Parameters
        ----------
        uncompensated_pos : `Position`
            Target position (without compensation applied).

        Returns
        -------
        compensation_info : `CompensationInfo`
            Compensation information.

        Raises
        ------
        salobj.ExpectedError
            If uncompensated or compensated position is out of bounds.
        """
        self._check_position(uncompensated_pos)

        compensation_inputs = None
        compensation_offset = None
        compensated_pos = None

        if self.compensation_mode:
            compensation_inputs = self.get_compensation_inputs()
            if compensation_inputs is not None:
                compensation_offset = (
                    self.compensation.get_offset(compensation_inputs)
                    + self.filter_offset
                )

                compensated_pos = uncompensated_pos + compensation_offset
                self._check_position(compensated_pos)

        return CompensationInfo(
            uncompensated_pos=uncompensated_pos,
            compensation_inputs=compensation_inputs,
            compensation_offset=compensation_offset,
        )

    def _check_position(self, position: base.Position) -> None:
        """Check the position is within limits or not.

        Parameters
        ----------
        position : `Position`
            Position to check.

        Raises
        ------
        `salobj.ExpectedError`
            If the position is out of bounds.
        """

        # Check the position for each degree of freedom
        utils.check_position(
            position,
            self.current_pos_limits,
            ExceptionClass=salobj.ExpectedError,
        )

        # Check the delta actuator displacement
        positions = np.array(
            [
                position.x * utils.UM_TO_M,
                position.y * utils.UM_TO_M,
                position.z * utils.UM_TO_M,
                np.deg2rad(position.u),
                np.deg2rad(position.v),
                np.deg2rad(position.w),
            ]
        )
        delta_strut_length = SimpleHexapod.inverse_kinematics(
            positions,
            self._pivot,
            self._mirror_positions,
            self._base_positions,
        )

        idx_max = np.abs(delta_strut_length).argmax()
        max_abs_delta_strut_length = delta_strut_length[idx_max] / utils.UM_TO_M

        if not (
            constants.ACTUATOR_MIN_LENGTH
            < max_abs_delta_strut_length
            < constants.ACTUATOR_MAX_LENGTH
        ):
            raise salobj.ExpectedError(
                f"{max_abs_delta_strut_length=} not in range "
                f"({constants.ACTUATOR_MIN_LENGTH}, {constants.ACTUATOR_MAX_LENGTH})"
            )

    def get_compensation_inputs(self) -> base.CompensationInputs | None:
        """Return the current compensation inputs, or None if not available.

        Log a warning if inputs are missing and the missing list
        does not match the previous warning.

        Returns
        -------
        compensation_inputs : `CompensationInputs` or `None`
            The compensation inputs, if all inputs are available, else `None`.
        """

        mount_target = self.mtmount.evt_target.get()
        mount_elevation = self.mtmount.tel_elevation.get()
        mount_azimuth = self.mtmount.tel_azimuth.get()
        mount_elevation_azimuth = self._get_mount_elevation_azimuth(
            mount_target, mount_elevation, mount_azimuth
        )

        missing_inputs = []
        nan_inputs = []
        if mount_elevation_azimuth is None:
            missing_inputs.append("MTMount.elevation, azimuth")
        else:
            if math.isnan(mount_elevation_azimuth[0]):
                nan_inputs.append("MTMount.elevation")
            if math.isnan(mount_elevation_azimuth[1]):
                nan_inputs.append("MTMount.azimuth")

        rotator_target = self.mtrotator.evt_target.get()
        rotator_rotation = self.mtrotator.tel_rotation.get()
        rotator_position = self._get_rotator_position(rotator_target, rotator_rotation)
        if rotator_position is None:
            missing_inputs.append("MTRotator.position")
        elif math.isnan(rotator_position):
            nan_inputs.append("MTRotator.position")

        temperature = self._get_temperature()

        bad_inputs_items = []
        if missing_inputs:
            bad_inputs_items.append("missing inputs: " + ", ".join(missing_inputs))
        if nan_inputs:
            bad_inputs_items.append("nan inputs: " + ", ".join(nan_inputs))

        if bad_inputs_items:
            bad_inputs_str = "; ".join(bad_inputs_items)
            if self.bad_inputs_str != bad_inputs_str:
                self.log.warning(f"Cannot apply compensation; {bad_inputs_str}")
                self.bad_inputs_str = bad_inputs_str
            return None

        self.bad_inputs_str = ""

        # Workaround the mypy check
        assert mount_elevation_azimuth is not None
        assert rotator_position is not None

        return base.CompensationInputs(
            elevation=mount_elevation_azimuth[0],
            azimuth=mount_elevation_azimuth[1],
            rotation=rotator_position,
            temperature=temperature,
        )

    def _get_mount_elevation_azimuth(
        self,
        mount_target: salobj.BaseMsgType,
        mount_elevation: salobj.BaseMsgType,
        mount_azimuth: salobj.BaseMsgType,
    ) -> None | tuple[float, float]:
        """Get the mount elevation and azimuth.

        Parameters
        ----------
        mount_target : `salobj.BaseMsgType`
            Mount target event.
        mount_elevation : `salobj.BaseMsgType`
            Mount elevation telemetry.
        mount_azimuth : `salobj.BaseMsgType`
            Mount azimuth telemetry.

        Returns
        -------
        None or `tuple` [`float`, `float`]
            Mount elevation and azimuth.
        """

        if mount_target is None:
            if (mount_elevation is not None) and (mount_azimuth is not None):
                return (mount_elevation.actualPosition, mount_azimuth.actualPosition)

        else:
            return (mount_target.elevation, mount_target.azimuth)

        return None

    def _get_rotator_position(
        self,
        rotator_target: salobj.BaseMsgType,
        rotator_rotation: salobj.BaseMsgType,
    ) -> None | float:
        """Get the rotator position.

        Parameters
        ----------
        rotator_target : `salobj.BaseMsgType`
            Rotator target event.
        rotator_rotation : `salobj.BaseMsgType`
            Rotator rotation telemetry.

        Returns
        -------
        None or `float`
            Rotator position.
        """

        if rotator_target is None:
            if rotator_rotation is not None:
                return rotator_rotation.actualPosition

        else:
            return rotator_target.position

        return None

    def _get_temperature(self) -> float:
        """Get the temperature.

        Returns
        -------
        `float`
            Temperature. Return 0.0 if the temperature is not available.
        """

        if (
            self.enable_lut_temperature
            and self.ess_temperature.tel_temperature.has_data
        ):
            temperatures = np.array(
                self.ess_temperature.tel_temperature.get().temperatureItem
            )

            # Only the first 6 elements are used
            return np.mean(temperatures[0:6])

        return 0.0

    async def do_configureAcceleration(self, data: salobj.BaseMsgType) -> None:
        """Specify the acceleration limit."""
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        utils.check_positive_value(
            data.acceleration,
            "acceleration",
            constants.MAX_ACCEL_LIMIT,
            ExceptionClass=salobj.ExpectedError,
        )
        await self.run_command(
            code=enums.CommandCode.CONFIG_ACCEL, param1=data.acceleration
        )

    async def do_configureLimits(self, data: salobj.BaseMsgType) -> None:
        """Specify position and rotation limits."""
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        try:
            new_limits = base.PositionLimits.from_struct(data)
        except ValueError as e:
            raise salobj.ExpectedError(str(e))
        utils.check_new_position_limits(
            limits=new_limits,
            max_limits=self.max_pos_limits,
            ExceptionClass=salobj.ExpectedError,
        )
        command_kwargs = {
            f"param{i+1}": value
            for i, value in enumerate(dataclasses.astuple(new_limits))
        }
        await self.run_command(code=enums.CommandCode.CONFIG_LIMITS, **command_kwargs)
        # The new limits are set by config_callback

    async def do_configureVelocity(self, data: salobj.BaseMsgType) -> None:
        """Specify velocity limits."""
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        utils.check_positive_value(
            data.xy,
            "xy",
            constants.MAX_LINEAR_VEL_LIMIT,
            ExceptionClass=salobj.ExpectedError,
        )
        utils.check_positive_value(
            data.uv,
            "uv",
            constants.MAX_ANGULAR_VEL_LIMIT,
            ExceptionClass=salobj.ExpectedError,
        )
        utils.check_positive_value(
            data.z,
            "z",
            constants.MAX_LINEAR_VEL_LIMIT,
            ExceptionClass=salobj.ExpectedError,
        )
        utils.check_positive_value(
            data.w,
            "w",
            constants.MAX_ANGULAR_VEL_LIMIT,
            ExceptionClass=salobj.ExpectedError,
        )
        await self.run_command(
            code=enums.CommandCode.CONFIG_VEL,
            param1=data.xy,
            param2=data.uv,
            param3=data.z,
            param4=data.w,
        )

    async def do_move(self, data: salobj.BaseMsgType) -> None:
        """Move to a specified position and orientation.

        Parameters
        ----------
        data : `salobj.BaseMsgType`
            Data of the SAL message.
        """
        await self._move_hexapod(
            base.Position.from_struct(data),
            data.sync,
        )

    async def _move_hexapod(
        self,
        uncompensated_pos: base.Position,
        sync: bool,
        overwrite_step_size_from_config: bool = False,
        step_size_xy: float = 0.0,
        step_size_z: float = 0.0,
        step_size_uv: float = 0.0,
        step_size_w: float = 0.0,
    ) -> None:
        """Move the hexapod.

        Check the target before and after compensation (if applied).
        Both the target and the compensated position (if compensating)
        should be in range, so we can turn off compensation at will.
        If compensation mode is off we do not test compensated position,
        as it allows running with invalid compensation coefficients or inputs.

        Parameters
        ----------
        uncompensated_pos : `base.Position`
            Uncompensated position.
        sync : `bool`
            Should this be a synchronized move? Usually True.
        overwrite_step_size_from_config : `bool`, optional
            If True then the step size passed to this function are overwritten
            from the configuration. (the default is False)
        step_size_xy : `float`, optional
            Absolute maximum step size in the x, y direction in um. Put 0 if
            you want to do the movement in a single step. (default is 0.0)
        step_size_z : `float`, optional
            Absolute maximum step size in the z direction in um. Put 0 if you
            want to do the movement in a single step. (default is 0.0)
        step_size_uv : `float`, optional
            Absolute maximum step size in the rx, ry direction in deg. Put 0 if
            you want to do the movement in a single step. (default is 0.0)
        step_size_w : `float`, optional
            Absolute maximum step size in the rz direction in deg. Put 0 if
            you want to do the movement in a single step. (default is 0.0)
        """

        self.assert_enabled()
        self.move_command_received_event.set()

        # Check the new position _before_ cancelling the current move (if any)
        # and starting a new move.
        self.compute_compensation(uncompensated_pos)
        async with self.write_lock:
            self.move_steps_task.cancel()
            self.move_task.cancel()
            self.compensation_loop_task.cancel()
        self.move_task = asyncio.create_task(
            self._move(
                uncompensated_pos=uncompensated_pos,
                sync=sync,
                overwrite_step_size_from_config=overwrite_step_size_from_config,
                step_size_xy=step_size_xy,
                step_size_z=step_size_z,
                step_size_uv=step_size_uv,
                step_size_w=step_size_w,
            )
        )
        await self.move_task

    async def do_moveInSteps(self, data: salobj.BaseMsgType) -> None:
        """Move to a specified position and orientation in steps.

        Parameters
        ----------
        data : `salobj.BaseMsgType`
            Data of the SAL message.
        """

        if data.overwriteStepSizeFromConfig:
            await self._move_hexapod(
                base.Position.from_struct(data),
                data.sync,
                overwrite_step_size_from_config=True,
            )
        else:
            await self._move_hexapod(
                base.Position.from_struct(data),
                data.sync,
                step_size_xy=data.stepSizeXY,
                step_size_z=data.stepSizeZ,
                step_size_uv=data.stepSizeUV,
                step_size_w=data.stepSizeW,
            )

    def get_filter_offset(self, filter_name: str) -> base.Position:
        """Get the z offset for a given filter name.

        Parameters
        ----------
        filter_name : `str`
            Camera filter name.

        Returns
        -------
        filter_offset : `Position`
            Filter offset.
        """

        z_offset = 0

        filter_object = self.filter_offsets_dict.get(filter_name, None)
        if filter_object is not None:
            z_offset = filter_object.get("z_offset", 0.0)
        else:
            available_filters = ", ".join(self.filter_offsets_dict)
            raise RuntimeError(
                f"Filter {filter_name} not in the list of available filters: {available_filters}. "
                "Make sure all mounted filters are properly configured with a filter offset in "
                "the hexapod configuration."
            )

        data = types.SimpleNamespace(
            x=0,
            y=0,
            z=z_offset,
            u=0,
            v=0,
            w=0,
        )
        filter_offset = base.Position.from_struct(data)

        return filter_offset

    async def handle_camera_filter(self, filter_name: str) -> None:
        """Move the hexapod in focusZ to account for the filter thickness.

        Parameters
        ----------
        filter_name : `str`
            Camera filter name.
        """
        self.filter_offset = self.get_filter_offset(filter_name)

    async def do_offset(self, data: salobj.BaseMsgType) -> None:
        """Move by a specified offset in position and orientation.

        See note for self._move_hexapod() regarding checking the target
        position.
        """

        uncompensated_pos = self._get_uncompensated_position_with_offset(
            base.Position.from_struct(data)
        )

        await self._move_hexapod(uncompensated_pos, data.sync)

    def _get_uncompensated_position_with_offset(
        self, offset: base.Position
    ) -> base.Position:
        """Get the uncompensated position with the offset applied.

        Parameters
        ----------
        offset : `base.Position`
            Position offset.

        Returns
        -------
        `base.Position`
            Uncompensated position with the offset applied
        """

        curr_uncompensated_pos = self._get_uncompensated_position()
        return curr_uncompensated_pos + offset

    async def do_offsetInSteps(self, data: salobj.BaseMsgType) -> None:
        """Move by a specified offset in position and orientation in steps.

        See note for self._move_hexapod() regarding checking the target
        position.
        """

        uncompensated_pos = self._get_uncompensated_position_with_offset(
            base.Position.from_struct(data)
        )
        if data.overwriteStepSizeFromConfig:
            await self._move_hexapod(
                uncompensated_pos, data.sync, overwrite_step_size_from_config=True
            )
        else:
            await self._move_hexapod(
                uncompensated_pos,
                data.sync,
                step_size_xy=data.stepSizeXY,
                step_size_z=data.stepSizeZ,
                step_size_uv=data.stepSizeUV,
                step_size_w=data.stepSizeW,
            )

    async def do_setCompensationMode(self, data: salobj.BaseMsgType) -> None:
        self.assert_enabled()
        self.initial_compensation_offset_applied = False
        await self.evt_compensationMode.set_write(enabled=data.enable)
        async with self.write_lock:
            self.move_steps_task.cancel()
            self.move_task.cancel()
            self.compensation_loop_task.cancel()
        if not self._has_uncompensated_position():
            if data.enable:
                self.log.info(
                    "No position as been commanded since this CSC started, "
                    "so compensation will begin once you command a move."
                )
            else:
                self.log.info(
                    "No position as been commanded since this CSC started, "
                    "so there is no compensation offset to remove."
                )
            return

        uncompensated_pos = self._get_uncompensated_position()
        if data.enable:
            self.log.info("Applying compensation to the current target position")
        else:
            self.log.info("Removing compensation from the current target position")
        self.move_task = asyncio.create_task(
            self._move(
                uncompensated_pos=uncompensated_pos,
                sync=True,
                overwrite_step_size_from_config=True,
            )
        )

    async def do_setPivot(self, data: salobj.BaseMsgType) -> None:
        """Set the coordinates of the pivot point."""
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        await self.run_command(
            code=enums.CommandCode.SET_PIVOTPOINT,
            param1=data.x,
            param2=data.y,
            param3=data.z,
        )

    async def do_stop(self, data: salobj.BaseMsgType) -> None:
        """Halt tracking or any other motion."""
        self.assert_enabled()
        async with self.write_lock:
            self.move_steps_task.cancel()
            self.move_task.cancel()
            self.compensation_loop_task.cancel()
        # This seems to be necessary for the low-level controller
        # to reliably respond to the stop command when issued
        # shortly after issuing a move command.
        # I would much rather just issue the stop command!
        await self.stop_motion()

    @contextlib.asynccontextmanager
    async def csc_level_fault(self) -> typing.AsyncIterator[None]:
        """Context manager to handle CSC level fault."""
        try:
            await self.stop_motion()
        except Exception:
            self.log.exception(
                "Stop motion failed while handling csc level fault. Continuing."
            )

        try:
            await self.standby_controller()
        except Exception:
            self.log.exception(
                "Sending controller to standby failed while handling csc level fault."
            )

        yield

    async def basic_run_command(self, command: hexrotcomm.Command) -> None:
        # Overload of lsst.ts.hexrotcomm.BaseCsc's version
        # that resets the n_telemetry attribute.
        self.n_telemetry = 0
        self.log.debug(
            f"send low-level command {enums.CommandCode(command.code)!r}; "
            f"params={command.param1}, {command.param2}, {command.param3}, "
            f"{command.param4}, {command.param5}, {command.param6}"
        )
        await super().basic_run_command(command)
        self.n_telemetry = 0

    async def handle_summary_state(self) -> None:
        await super().handle_summary_state()
        if self.summary_state != salobj.State.ENABLED:
            async with self.write_lock:
                self.move_steps_task.cancel()
                self.move_task.cancel()
                self.compensation_loop_task.cancel()
            self.idle_time_monitor_task.cancel()
            await self.evt_compensationMode.set_write(enabled=False)
            names = base.Position.field_names()
            await self.evt_compensationOffset.set_write(
                elevation=math.nan,
                azimuth=math.nan,
                rotation=math.nan,
                temperature=math.nan,
                **dict((name, math.nan) for name in names),
            )
            await self.evt_uncompensatedPosition.set_write(
                **dict((name, math.nan) for name in names),
            )
            await self.evt_compensatedPosition.set_write(
                **dict((name, math.nan) for name in names),
            )

        if self.summary_state == salobj.State.ENABLED:
            self.idle_time_monitor_task = asyncio.create_task(self.idle_time_monitor())
            self.initial_compensation_offset_applied = False

    async def idle_time_monitor(self, period: float = 1.0, max_count: int = 10) -> None:
        """Idle time monitor under the enabled state.

        Parameters
        ----------
        period : `float`, optional
            Polling period. (the default is 1.0)
        max_count : `int`, optional
            Maximum count of the polling. When the count reachs the maximum,
            the idle time will be updated. This is to save the system resource
            to acquire/release the asynchronous lock. (the default is 10)
        """

        self.log.info(
            "Start the task to monitor the idle time under the enabled state."
        )

        count = 0
        while self.summary_state == salobj.State.ENABLED:
            await asyncio.sleep(period)

            if self.client.telemetry.state == ControllerState.ENABLED:
                count += 1
            else:
                count = 0

            if count >= max_count:
                await self._update_idle_time(count * period)

                if self._idle_time_in_enabled_state >= self.no_movement_idle_time:
                    await self.standby_controller()

                    self.log.info(
                        "Standby the controller after the timeout of no movement when enabled."
                    )

                count = 0

        # Reset the idle time when not monitoring
        await self._update_idle_time(0.0)

        self.log.info("End the task to monitor the idle time under the enabled state.")

    async def _update_idle_time(self, offset: float) -> None:
        """Update the idle time.

        Parameters
        ----------
        offset : `float`
            Offset to add to the current idle time. If 0, the idle time will be
            reset.
        """

        async with self._lock_idle_time:
            if offset == 0.0:
                self._idle_time_in_enabled_state = 0.0
            else:
                self._idle_time_in_enabled_state += offset

    async def start(self) -> None:
        await super().start()
        await asyncio.gather(self.mtmount.start_task, self.mtrotator.start_task)
        await self.evt_compensationMode.set_write(enabled=False)

    async def telemetry_callback(
        self, client: hexrotcomm.CommandTelemetryClient
    ) -> None:
        """Called when the low-level controller outputs telemetry.

        Parameters
        ----------
        client : `lsst.ts.hexrotcomm.CommandTelemetryClient`
            TCP/IP client.
        """
        tai_unix = client.header.tai_sec + client.header.tai_nsec / 1e9

        # Strangely telemetry.state and enabled_substate
        # are all floats from the controller. But they should only have
        # integer value, so I output them as integers.
        enabled_substate = int(client.telemetry.enabled_substate)
        await self.evt_controllerState.set_write(
            controllerState=int(client.telemetry.state),
            enabledSubstate=enabled_substate,
            applicationStatus=client.telemetry.application_status,
        )

        if (not self.move_steps_start_event.is_set()) and (
            enabled_substate == EnabledSubstate.MOVING_POINT_TO_POINT.value
        ):
            self.move_steps_start_event.set()

        pos_error = [
            client.telemetry.measured_xyz[i] - client.telemetry.commanded_pos[i]
            for i in range(3)
        ] + [
            client.telemetry.measured_uvw[i] - client.telemetry.commanded_pos[i + 3]
            for i in range(3)
        ]

        # Change the unit from m to um
        calibrated = [
            single_posfiltvel.pos_filt * 1e6
            for single_posfiltvel in client.telemetry.estimated_posfiltvel
        ]
        await self.tel_actuators.set_write(
            calibrated=calibrated,
            raw=client.telemetry.strut_measured_pos_raw,
            positionError=client.telemetry.strut_pos_error,
            timestamp=tai_unix,
        )
        await self.tel_application.set_write(
            demand=client.telemetry.commanded_pos,
            position=list(client.telemetry.measured_xyz)
            + list(client.telemetry.measured_uvw),
            error=pos_error,
        )

        electrical = dict(
            copleyStatusWordDrive=client.telemetry.status_word,
            copleyLatchingFaultStatus=client.telemetry.latching_fault_status_register,
            copleyFaultStatus=client.telemetry.copley_fault_status_register,
            motorCurrent=client.telemetry.motor_current,
            busVoltage=client.telemetry.bus_voltage,
        )
        await self.tel_electrical.set_write(**electrical)

        in_position = (
            client.telemetry.application_status & ApplicationStatus.MOVE_COMPLETE
        )
        if (not self.move_steps_in_position_event.is_set()) and in_position:
            self.move_steps_in_position_event.set()

        # Only publish the inPosition event when the self.move_steps_task is
        # done. This is to avoid the interference of the results of each small
        # step movement.
        if self.move_steps_task.done() and in_position:
            await self.evt_inPosition.set_write(inPosition=True)

        await self.evt_commandableByDDS.set_write(
            state=bool(
                client.telemetry.application_status
                & ApplicationStatus.DDS_COMMAND_SOURCE
            )
        )

        safety_interlock = (
            client.telemetry.application_status & ApplicationStatus.SAFETY_INTERLOCK
        )
        await self.evt_interlock.set_write(engaged=safety_interlock)
        if (self.summary_state == salobj.State.FAULT) and safety_interlock:
            await self.evt_errorCode.set_write(
                errorCode=ErrorCode.INTERLOCK_OPEN.value,
                errorReport="Safety interlock open",
            )

        if self.n_telemetry < MAX_N_TELEMETRY:
            self.n_telemetry += 1
        self.telemetry_event.set()

    def make_mock_controller(self) -> mock_controller.MockMTHexapodController:
        return mock_controller.MockMTHexapodController(
            log=self.log,
            index=self.salinfo.index,
            port=0,
            initial_state=ControllerState.STANDBY,
        )

    def _make_position_set_command(self, position: base.Position) -> hexrotcomm.Command:
        """Make a POSITION_SET command for the low-level controller.

        Parameters
        ----------
        position : `Position`
            Desired position.
        """
        self._check_position(position)
        command_kwargs = {
            f"param{i+1}": value
            for i, value in enumerate(dataclasses.astuple(position))
        }
        return self.make_command(code=enums.CommandCode.POSITION_SET, **command_kwargs)

    def _has_uncompensated_position(self) -> bool:
        """Return True if the uncompensated position has been set,
        e.g. by a move command.
        """
        return self.evt_uncompensatedPosition.has_data

    def _get_uncompensated_position(self) -> base.Position:
        """Return the current uncompensated position.

        Returns
        -------
        position : `Position`
            The uncompensated position.

        Raises
        ------
        salobj.ExpectedError
            If the current uncompensated position has never been reported.
        """
        uncompensated_data = self.evt_uncompensatedPosition.data
        current_position = self._get_current_position()

        if uncompensated_data is None:
            raise salobj.ExpectedError("No uncompensated position to offset from")

        names = base.Position.field_names()

        for name in names:
            if math.isnan(getattr(uncompensated_data, name)):
                setattr(
                    uncompensated_data,
                    name,
                    getattr(current_position, name),
                )
        return base.Position.from_struct(uncompensated_data)

    async def stop_motion(self) -> None:
        """Stop motion and wait for it to stop.

        Raises:
            asyncio.CancelledError if not in enabled state.
        """
        if self.client.telemetry.state != ControllerState.ENABLED:
            raise asyncio.CancelledError("Not enabled")

        # TODO: once DM-29975 is fixed remove this code block
        # and just run the STOP command. For now it appears to be
        # necessary to give the low-level controller some time
        # to process the previous command before it can be stopped.
        await self.wait_n_telemetry()
        if self.client.telemetry.enabled_substate == EnabledSubstate.STATIONARY:
            return

        # Cancel the self.move_steps_task if it is running
        if not self.move_steps_task.done():
            self.move_steps_task.cancel()

        await self.run_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.STOP,
        )

        await self.wait_stopped()

    async def wait_n_telemetry(self, n_telemetry: int = 4) -> None:
        """Wait for n_telemetry telemetry messages since the most recent
        low-level command.

        Parameters
        ----------
        n_telemetry : `int`, optional
            Minimum number of telemetry messages since the most recent
            low-level command. Must be positive. A value of 3 is necessary
            to reliably allow stop or move to interrupt another move.

        Raises
        ------
        asyncio.CancelledError
            If the system goes out of enabled state.
        """
        if n_telemetry < 0 or n_telemetry > MAX_N_TELEMETRY:
            raise ValueError(
                f"n_telemetry={n_telemetry} must be in range [0, {MAX_N_TELEMETRY}]"
            )

        if self.n_telemetry >= n_telemetry:
            return

        while self.n_telemetry < n_telemetry:
            self.telemetry_event.clear()
            await self.telemetry_event.wait()
            if self.client.telemetry.state != ControllerState.ENABLED:
                raise asyncio.CancelledError()

    async def wait_stopped(self, n_telemetry: int = 4) -> bool:
        """Wait for the current motion, if any, to stop.

        Parameters
        ----------
        n_telemetry : `int`, optional
            Minimum number of telemetry messages since the most recent
            low-level command. Must be positive. A value of 3 is necessary
            to reliably allow stop or move to interrupt another move.

        Returns
        -------
        is_stopped : `bool`
            Is motion stopped? Always True unless max_nstatus is not None.

        Raises:
            asyncio.CancelledError if not in enabled state.
        """
        if n_telemetry < 0 or n_telemetry > MAX_N_TELEMETRY:
            raise ValueError(
                f"n_telemetry={n_telemetry} must be in range [0, {MAX_N_TELEMETRY}]"
            )

        if self.client.telemetry.state != ControllerState.ENABLED:
            raise asyncio.CancelledError()

        self.log.debug(
            f"Waiting for at least {n_telemetry} consecutive telemetry samples with stationary state."
        )

        n_telemetry_stopped = 0
        while n_telemetry_stopped < n_telemetry:
            self.telemetry_event.clear()
            await self.telemetry_event.wait()
            if self.client.telemetry.state != ControllerState.ENABLED:
                raise asyncio.CancelledError()
            if self.client.telemetry.enabled_substate == EnabledSubstate.STATIONARY:
                n_telemetry_stopped += 1
                self.log.debug(
                    f"[{n_telemetry_stopped}/{n_telemetry}]: Hexapod stationary."
                )
            else:
                if n_telemetry_stopped > 0:
                    self.log.warning(
                        f"[{n_telemetry_stopped}/{n_telemetry}]: "
                        "Hexapod became non-stationary after stationary state and waiting for it to stop. "
                        "Reset counter."
                    )
                n_telemetry_stopped = 0

        return True

    async def _move(
        self,
        uncompensated_pos: base.Position,
        sync: bool,
        is_compensation_loop: bool = False,
        overwrite_step_size_from_config: bool = False,
        step_size_xy: float = 0.0,
        step_size_z: float = 0.0,
        step_size_uv: float = 0.0,
        step_size_w: float = 0.0,
    ) -> None:
        """Command a move and output appropriate events.

        Parameters
        ----------
        uncompensated_pos : `Position`
            Target position (without compensation applied).
        sync : `bool`
            Should this be a synchronized move? Usually True.
        is_compensation_loop : `bool`, optional
            If True then this is being called by the background
            compensation loop, in which case:

            * Check the amount of compensation and only move if at least one
              axis has changed by at least self.min_compensation_adjustment.
            * Do not restart the compensation loop.
        overwrite_step_size_from_config : `bool`, optional
            If True then the step size passed to this function are overwritten
            from the configuration (the default is False).
        step_size_xy : `float`, optional
            Absolute maximum step size in the x, y direction in um. Put 0 if
            you want to do the movement in a single step. (the default is 0.0)
        step_size_z : `float`, optional
            Absolute maximum step size in the z direction in um. Put 0 if you
            want to do the movement in a single step. (the default is 0.0)
        step_size_uv : `float`, optional
            Absolute maximum step size in the rx, ry direction in deg. Put 0 if
            you want to do the movement in a single step. (the default is 0.0)
        step_size_w : `float`, optional
            Absolute maximum step size in the rz direction in deg. Put 0 if
            you want to do the movement in a single step. (the default is 0.0)
        """
        try:
            # Enable the controller state first if it is not. Note that it
            # might be put into the Standby state when the CSC is under the
            # Enabled state without the movement after the timeout by the
            # self.idle_time_monitor().
            if self.client.telemetry.state != ControllerState.ENABLED:
                await self.enable_controller()

                self.log.info("Re-enable the controller from the idle.")

            compensation_info = self.compute_compensation(uncompensated_pos)

            if is_compensation_loop:
                if compensation_info.compensation_offset is None:
                    return
                names = base.Position.field_names()
                compensated_pos_list = [
                    getattr(compensation_info.compensated_pos, name) for name in names
                ]
                current_position = self._get_current_position()
                current_position_list = [
                    getattr(current_position, name) for name in names
                ]

                delta = np.subtract(
                    compensated_pos_list,
                    current_position_list,
                )
                if np.all(np.abs(delta) < self.min_compensation_adjustment):
                    if not self.initial_compensation_offset_applied:
                        self.log.info(
                            f"Compensation offset too small: {delta}. "
                            "However this is the initial compensation offset so applying it anyway."
                        )
                    else:
                        self.log.debug(
                            "Compensation offset too small to apply: %s", delta
                        )
                        return

                self.initial_compensation_offset_applied = True
            # Stop the current motion, if any, and wait for it to stop.
            await asyncio.wait_for(self.stop_motion(), timeout=MAXIMUM_STOP_TIME)

            # Get the step size from the configuration if needed.
            step_size_xy_final = (
                self.step_size_xy if overwrite_step_size_from_config else step_size_xy
            )
            step_size_z_final = (
                self.step_size_z if overwrite_step_size_from_config else step_size_z
            )
            step_size_uv_final = (
                self.step_size_uv if overwrite_step_size_from_config else step_size_uv
            )
            step_size_w_final = (
                self.step_size_w if overwrite_step_size_from_config else step_size_w
            )

            # Command the new motion.
            self.move_steps_task = asyncio.create_task(
                self._command_controller_to_move_hexapod(
                    compensation_info.compensated_pos,
                    sync,
                    step_size_xy=step_size_xy_final,
                    step_size_z=step_size_z_final,
                    step_size_uv=step_size_uv_final,
                    step_size_w=step_size_w_final,
                )
            )

            await self.evt_uncompensatedPosition.set_write(**vars(uncompensated_pos))
            await self.evt_compensatedPosition.set_write(
                **vars(compensation_info.compensated_pos)
            )
            if compensation_info.compensation_offset is not None:
                # Workaround the mypy check
                assert compensation_info.compensation_inputs is not None

                await self.evt_compensationOffset.set_write(
                    elevation=compensation_info.compensation_inputs.elevation,
                    azimuth=compensation_info.compensation_inputs.azimuth,
                    rotation=compensation_info.compensation_inputs.rotation,
                    temperature=compensation_info.compensation_inputs.temperature,
                    **vars(compensation_info.compensation_offset),
                )
        except Exception:
            # This move failed; restart the compensation loop anyway,
            # if it is wanted.
            if self.compensation_mode and not is_compensation_loop:
                self.compensation_loop_task = asyncio.create_task(
                    self.compensation_loop()
                )
            raise

        if self.compensation_mode and not is_compensation_loop:
            self.compensation_loop_task = asyncio.create_task(self.compensation_loop())

        await self._update_idle_time(0.0)

    async def _command_controller_to_move_hexapod(
        self,
        position_target: base.Position,
        sync: bool,
        step_size_xy: float = 0.0,
        step_size_z: float = 0.0,
        step_size_uv: float = 0.0,
        step_size_w: float = 0.0,
    ) -> None:
        """Command the controller to move the hexapod.

        Parameters
        ----------
        position_target : `base.Position`
            Target position.
        sync : `bool`
            Should this be a synchronized move? Usually True.
        step_size_xy : `float`, optional
            Absolute maximum step size in the x, y direction in um. Put 0 if
            you want to do the movement in a single step. (the default is 0.0)
        step_size_z : `float`, optional
            Absolute maximum step size in the z direction in um. Put 0 if you
            want to do the movement in a single step. (the default is 0.0)
        step_size_uv : `float`, optional
            Absolute maximum step size in the rx, ry direction in deg. Put 0 if
            you want to do the movement in a single step. (the default is 0.0)
        step_size_w : `float`, optional
            Absolute maximum step size in the rz direction in deg. Put 0 if
            you want to do the movement in a single step. (the default is 0.0)
        """

        await self.evt_inPosition.set_write(inPosition=False)

        position_next = self._get_next_position(
            position_target,
            step_size_xy,
            step_size_z,
            step_size_uv,
            step_size_w,
        )
        while position_next != position_target:
            # Use this event to know the hexapod begins the movement or not.
            self.move_steps_start_event.clear()

            await self._internal_commands_to_controller(position_next, sync)

            await self.move_steps_start_event.wait()

            # Wait for the in_position event for the step-movement.
            self.move_steps_in_position_event.clear()
            await self.move_steps_in_position_event.wait()

            position_next = self._get_next_position(
                position_target,
                step_size_xy,
                step_size_z,
                step_size_uv,
                step_size_w,
            )

        # Issue the final movement outside the above while loop to let the CSC
        # can publish the inPosition event.
        await self._internal_commands_to_controller(position_next, sync)

    def _get_next_position(
        self,
        position_target: base.Position,
        step_size_xy: float,
        step_size_z: float,
        step_size_uv: float,
        step_size_w: float,
    ) -> base.Position:
        """Get the next position.

        Parameters
        ----------
        position_target : `base.Position`
            Target position.
        step_size_xy : `float`
            Absolute maximum step size in the x, y direction in um. Put 0 if
            you want to do the movement in a single step.
        step_size_z : `float`
            Absolute maximum step size in the z direction in um. Put 0 if you
            want to do the movement in a single step.
        step_size_uv : `float`
            Absolute maximum step size in the rx, ry direction in deg. Put 0 if
            you want to do the movement in a single step.
        step_size_w : `float`
            Absolute maximum step size in the rz direction in deg. Put 0 if
            you want to do the movement in a single step.

        Returns
        -------
        `base.Position`
            The next position.
        """
        return utils.get_next_position(
            self._get_current_position(),
            position_target,
            step_size_xy,
            step_size_z,
            step_size_uv,
            step_size_w,
        )

    def _get_current_position(self) -> base.Position:
        """Get the current position.

        Returns
        -------
        `base.Position`
            The current position.
        """
        measured_xyz = self.client.telemetry.measured_xyz
        measured_uvw = self.client.telemetry.measured_uvw
        return base.Position(
            x=measured_xyz[0],
            y=measured_xyz[1],
            z=measured_xyz[2],
            u=measured_uvw[0],
            v=measured_uvw[1],
            w=measured_uvw[2],
        )

    async def _internal_commands_to_controller(
        self, position: base.Position, sync: bool, delay: float = 0.1
    ) -> None:
        """Issue the internal commands to the controller.

        Parameters
        ----------
        position : `base.Position`
            Position.
        sync : `bool`
            Should this be a synchronized move? Usually True.
        delay : `float`, optional
            Delay time between the commands. (the default is 0.1)
        """

        cmd1 = self._make_position_set_command(position)
        cmd2 = self.make_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
            param2=int(sync),
        )
        # Need to wait some time between two commands for the Simulink
        # model to transition the state correctly with the appropriate
        # parameter.
        await self.run_multiple_commands(cmd1, cmd2, delay=delay)


def run_mthexapod() -> None:
    """Run the MTHexapod CSC."""
    asyncio.run(HexapodCsc.amain(index=enums.SalIndex))
