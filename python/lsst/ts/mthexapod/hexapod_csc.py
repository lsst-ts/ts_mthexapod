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

__all__ = ["HexapodCsc"]

import asyncio
import copy
import dataclasses
import types

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from lsst.ts.idl.enums.MTHexapod import (
    ControllerState,
    EnabledSubstate,
    ApplicationStatus,
)
from . import __version__
from .config_schema import CONFIG_SCHEMA
from . import base
from . import compensation
from . import constants
from . import enums
from . import mock_controller
from . import structs
from . import utils

# Maximum time to stop axes (seconds).
# The minimum needed is time to acquire the write lock,
# issue the command, and wait for motion to stop.
# However, it is fine to be generous.
MAXIMUM_STOP_TIME = 10

# Maximum number of consecutive enabled/stationary states
# one can wait for. Sets the maximum value of nstopped
MAXIMUM_WAIT_STOPPED = 10


class CompensationInfo:
    """Information about a possibly compensated move

    Parameters
    ----------
    uncompensated_pos : `dict` [`str`, `float`]
        Target position (without compensation applied)
        as a dict of axis name: position
        keys are x, y, z (um), u, v, w (deg).
    compensation_offset : `dict` [`str`, `float`] or `None`
        Position with compensation applied, if relevant,
        as a dict of axis name: position
        keys are x, y, z (um), u, v, w (deg).
    compensation_inputs : `CompensationInputs` or `None`
        The compensation inputs, if compensating and all inputs
        are available, else `None`.

    Attributes
    ----------
    compensated_pos : `dict` [`str`, `float`]
        Compensated position: uncompensated_pos + compensation_offset
        if compensation_offset is not None, else uncompensated_pos.
        A dict of axis name: position
        keys are x, y, z (um), u, v, w (deg).

    Notes
    -----
    All the parameters are also attributes.
    """

    def __init__(self, uncompensated_pos, compensation_inputs, compensation_offset):
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
    config_dir : `str`, optional
        Directory of configuration files, or None for the standard
        configuration directory (obtained from `_get_default_config_dir`).
        This is provided for unit testing.
    initial_state : `lsst.ts.salobj.State` or `int` (optional)
        The initial state of the CSC.
        Must be `lsst.ts.salobj.State.OFFLINE` unless simulating
        (``simulation_mode != 0``).
    settings_to_apply : `str`, optional
        Settings to apply if ``initial_state`` is `State.DISABLED`
        or `State.ENABLED`.
    simulation_mode : `int` (optional)
        Simulation mode. Allowed values:

        * 0: regular operation.
        * 1: simulation: use a mock low level controller.

    Raises
    ------
    ValueError
        If ``initial_state != lsst.ts.salobj.State.OFFLINE``
        and not simulating (``simulation_mode = 0``).

    Notes
    -----
    **Error Codes**

    * 1: invalid data read on the telemetry socket

    This CSC is unusual in several respect:

    * It acts as a server (not a client) for a low level controller
      (because that is how the low level controller is written).
    * The low level controller maintains the summary state and detailed state
      (that's why this code inherits from Controller instead of BaseCsc).
    * The simulation mode can only be set at construction time.
    """

    valid_simulation_modes = [0, 1]
    version = __version__

    def __init__(
        self,
        index,
        config_dir=None,
        initial_state=salobj.State.OFFLINE,
        settings_to_apply="",
        simulation_mode=0,
    ):
        index = enums.SalIndex(index)
        controller_constants = constants.IndexControllerConstants[index]

        # The maximum position limits configured in the low-level controller.
        # The limits specified by the configureLimits command must be
        # within these limits.
        self.max_pos_limits = constants.MAX_POSITION_LIMITS[index]

        # Current position limits; initialize to max limits,
        # but update from configuration reported by the low-level controller.
        self.current_pos_limits = copy.copy(self.max_pos_limits)

        # The move command that is currently running, or None.
        self.move_command = None

        # Task for the current move, if one is being commanded.
        self.move_task = salobj.make_done_future()

        # If the compensation loop starts and there are missing
        # inputs then warn once and set this flag true.
        self.missing_inputs_str = ""

        # Interval between compensation updates.
        # Set in `configure`, but we need an initial value.
        self.compensation_interval = 0.2

        # Maximum time for the longest possible move (sec) with some buffer.
        # Set in `config_callback` but we need an initial value.
        # 61.74 is the value for the mock controller as of 2021-04-15.
        self.max_move_duration = 65

        # The part of the compensation loop that is always safe to cancel.
        self.compensation_wait_task = salobj.make_done_future()

        # The whole compensation loop; to safely cancel this use::
        #
        #     async with self.write_lock:
        #        self.compensation_loop_task.cancel()
        self.compensation_loop_task = salobj.make_done_future()

        # Event set when a telemetry message is received from
        # the low-level controller, after it has been parsed.
        self.telemetry_event = asyncio.Event()

        # Count of number of consecutive enabled/stationary states,
        # up to a maximum value of MAX_WAIT_STOPPED.
        # Reset to 0 when disabled or any command begins.
        self.nstopped = 0

        structs.Config.FRAME_ID = controller_constants.config_frame_id
        structs.Telemetry.FRAME_ID = controller_constants.telemetry_frame_id

        super().__init__(
            name="MTHexapod",
            index=index,
            port=controller_constants.port,
            sync_pattern=controller_constants.sync_pattern,
            CommandCode=enums.CommandCode,
            ConfigClass=structs.Config,
            TelemetryClass=structs.Telemetry,
            config_schema=CONFIG_SCHEMA,
            config_dir=config_dir,
            initial_state=initial_state,
            settings_to_apply=settings_to_apply,
            simulation_mode=simulation_mode,
        )

        # TODO DM-28005: add a suitable Remote from which to get temperature;
        # perhaps something like:
        # self.eas = salobj.Remote(domain=self.domain, name="EAS", include=[?])
        self.mtmount = salobj.Remote(
            domain=self.domain, name="MTMount", include=["target"]
        )
        self.mtrotator = salobj.Remote(
            domain=self.domain, name="MTRotator", include=["target"]
        )

    @property
    def compensation_mode(self):
        """Return True if moves are compensated, False otherwise."""
        return self.evt_compensationMode.data.enabled

    def config_callback(self, server):
        """Called when the low-level controller outputs configuration.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        self.evt_configuration.set_put(
            maxXY=server.config.pos_limits[0],
            minZ=server.config.pos_limits[1],
            maxZ=server.config.pos_limits[2],
            maxUV=server.config.pos_limits[3],
            minW=server.config.pos_limits[4],
            maxW=server.config.pos_limits[5],
            maxVelocityXY=server.config.vel_limits[0],
            maxVelocityUV=server.config.vel_limits[1],
            maxVelocityZ=server.config.vel_limits[2],
            maxVelocityW=server.config.vel_limits[3],
            initialX=server.config.initial_pos[0],
            initialY=server.config.initial_pos[1],
            initialZ=server.config.initial_pos[2],
            initialU=server.config.initial_pos[3],
            initialV=server.config.initial_pos[4],
            initialW=server.config.initial_pos[5],
            pivotX=server.config.pivot[0],
            pivotY=server.config.pivot[1],
            pivotZ=server.config.pivot[2],
            maxDisplacementStrut=server.config.max_displacement_strut,
            maxVelocityStrut=server.config.max_velocity_strut,
            accelerationStrut=server.config.acceleration_strut,
        )
        self.current_pos_limits = base.PositionLimits.from_struct(
            self.evt_configuration.data
        )

        # Compute a simplistic model for the maximum duration of a move (sec).
        # Ignore the distance over which the strut accelerates
        # and decelerates, when estimating the time spent at maximum velocity,
        # as that should be negligible.
        # Note that max_displacement_strut is a half distance (+/- from 0).
        if server.config.max_displacement_strut <= 0:
            self.log.warning(
                f"Reported max_displacement_strut {server.config.max_displacement_strut} <= 0; "
                f"using existing max_move_duration {self.max_move_duration} sec"
            )
            return
        if server.config.max_velocity_strut <= 0:
            self.log.warning(
                f"Reported max_velocity_strut {server.config.acceleration_strut} <= 0; "
                f"using existing max_move_duration {self.max_move_duration} sec"
            )
            return
        if server.config.acceleration_strut <= 0:
            self.log.warning(
                f"Reported strut acceleration {server.config.acceleration_strut} <= 0; "
                f"using existing max_move_duration {self.max_move_duration} sec"
            )
            return

        telemetry_interval = 0.2
        accel_duration = (
            server.config.max_velocity_strut / server.config.acceleration_strut
        )
        vel_duration = (
            server.config.max_displacement_strut / server.config.max_velocity_strut
        )
        self.max_move_duration = 2.1 * (
            telemetry_interval + accel_duration + vel_duration
        )
        self.log.info(f"max_move_duration={self.max_move_duration}")

    async def configure(self, config):
        self.compensation_interval = config.compensation_interval
        subconfig_name = {
            enums.SalIndex.CAMERA_HEXAPOD: "camera_config",
            enums.SalIndex.M2_HEXAPOD: "m2_config",
        }[self.salinfo.index]
        subconfig = types.SimpleNamespace(**getattr(config, subconfig_name))
        self.compensation = compensation.Compensation(
            elevation_coeffs=subconfig.elevation_coeffs,
            azimuth_coeffs=subconfig.azimuth_coeffs,
            rotation_coeffs=subconfig.rotation_coeffs,
            temperature_coeffs=subconfig.temperature_coeffs,
            min_temperature=subconfig.min_temperature,
            max_temperature=subconfig.max_temperature,
        )

    def connect_callback(self, server):
        super().connect_callback(server)
        if not self.server.connected:
            self.disable_compensation()

    async def compensation_loop(self):
        """Apply compensation at regular intervals.

        The algorithm is to repeat the following sequence:

        * Wait until it is time to apply compensation
          (see `compensation_wait` for details).
        * Apply the compensation offset.

        If a compensation update fails then compensation is disabled.
        """
        if not self._has_uncompensated_position():
            self.log.error("Compensation failed; no position has been commanded")
            return

        while self.summary_state == salobj.State.ENABLED:
            self.compensation_wait_task = asyncio.create_task(self.compensation_wait())
            await self.compensation_wait_task
            try:
                self.log.debug("Apply compensation")
                uncompensated_pos = self._get_uncompensated_position()
                await self._move(
                    uncompensated_pos=uncompensated_pos,
                    sync=1,
                    start_compensation=False,
                )
            except asyncio.CancelledError:
                # Normal termination. This may be temporary (e.g.
                # when starting a move or offset command) so do not
                # report compensation mode disabled.
                return
            except Exception:
                self.log.exception("Compensation failed; turning off compensation mode")
                self.evt_compensationMode.set_put(enabled=False)
                return

    async def compensation_wait(self):
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

            if self.server.telemetry.enabled_substate == EnabledSubstate.STATIONARY:
                # Axes are still halted; we're done!
                return
            else:
                # This should never happen, because any new move cancels
                # the compensation loop. But just in case...
                self.log.warning(
                    "Found the hexapod moving after waiting the "
                    "compensation interval; waiting again."
                )

    def compute_compensation(self, uncompensated_pos):
        """Check uncompensated and, if relevant, compensated position
        and return compensation information.

        Parameters
        ----------
        uncompensated_pos : `dict` [`str`, `float`]
            Target position (without compensation applied)
            as a dict of axis name: position
            keys are x, y, z (um), u, v, w (deg).

        Returns
        -------
        compensation_info : `CompensationInfo`
            Compensation information.

        Raises
        ------
        salobj.ExpectedError
            If uncompensated or compensated position is out of bounds.
        """
        utils.check_position(
            position=uncompensated_pos,
            limits=self.current_pos_limits,
            ExceptionClass=salobj.ExpectedError,
        )

        compensation_inputs = None
        compensation_offset = None
        compensated_pos = None

        if self.compensation_mode:
            compensation_inputs = self.get_compensation_inputs()
            if compensation_inputs is not None:
                compensation_offset = self.compensation.get_offset(compensation_inputs)
                compensated_pos = uncompensated_pos + compensation_offset
                utils.check_position(
                    position=compensated_pos,
                    limits=self.current_pos_limits,
                    ExceptionClass=salobj.ExpectedError,
                )

        return CompensationInfo(
            uncompensated_pos=uncompensated_pos,
            compensation_inputs=compensation_inputs,
            compensation_offset=compensation_offset,
        )

    def get_compensation_inputs(self):
        """Return the current compensation inputs, or None if not available.

        Log a warning if inputs are missing and the missing list
        does not match the previous warning.

        Returns
        -------
        compensation_inputs : `CompensationInputs` or `None`
            The compensation inputs, if all inputs are available, else `None`.
        """
        mount_target = self.mtmount.evt_target.get()
        missing_inputs = []
        if mount_target is None:
            missing_inputs.append("MTMount.target.elevation, azimuth")

        rotator_target = self.mtrotator.evt_target.get()
        if rotator_target is None:
            missing_inputs.append("MTRotator.target.position")

        # TODO DM-28005: update this code:
        temperature = 0

        if missing_inputs:
            missing_str = ", ".join(missing_inputs)
            if self.missing_inputs_str != missing_str:
                self.log.warning(
                    f"Cannot apply compensation; missing inputs: {missing_str}"
                )
                self.missing_inputs_str = missing_str
            return None

        self.missing_inputs_str = ""

        return base.CompensationInputs(
            elevation=mount_target.elevation,
            azimuth=mount_target.azimuth,
            rotation=rotator_target.position,
            temperature=temperature,
        )

    async def do_configureAcceleration(self, data):
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

    async def do_configureLimits(self, data):
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

    async def do_configureVelocity(self, data):
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

    async def do_move(self, data):
        """Move to a specified position and orientation.

        Check the target before and after compensation (if applied).
        Both the target and the compensated position (if compensating)
        should be in range, so we can turn off compensation at will.
        If compensation mode is off we do not test compensated position,
        as it allows running with invalid compensation coefficients or inputs.
        """
        self.assert_enabled()
        uncompensated_pos = base.Position.from_struct(data)

        # Check the new position _before_ cancelling the current move (if any)
        # and starting a new move.
        self.compute_compensation(uncompensated_pos)
        async with self.write_lock:
            self.move_task.cancel()
            self.compensation_loop_task.cancel()
        self.move_task = asyncio.create_task(
            self._move(
                uncompensated_pos=uncompensated_pos,
                sync=data.sync,
                start_compensation=True,
            )
        )
        await self.move_task

    async def do_offset(self, data):
        """Move by a specified offset in position and orientation.

        See note for do_move regarding checking the target position.
        """
        self.assert_enabled()
        curr_uncompensated_pos = self._get_uncompensated_position()
        offset = base.Position.from_struct(data)
        uncompensated_pos = curr_uncompensated_pos + offset

        # Check the new position _before_ cancelling the current move (if any)
        # and starting a new move.
        self.compute_compensation(uncompensated_pos)
        async with self.write_lock:
            self.move_task.cancel()
            self.compensation_loop_task.cancel()
        self.move_task = asyncio.create_task(
            self._move(
                uncompensated_pos=uncompensated_pos,
                sync=data.sync,
                start_compensation=True,
            )
        )
        await self.move_task

    async def do_setCompensationMode(self, data):
        self.assert_enabled()
        self.evt_compensationMode.set_put(enabled=data.enable)
        async with self.write_lock:
            self.compensation_loop_task.cancel()
            self.move_task.cancel()
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
                sync=1,
                start_compensation=data.enable,
            )
        )

    async def do_setPivot(self, data):
        """Set the coordinates of the pivot point."""
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        await self.run_command(
            code=enums.CommandCode.SET_PIVOTPOINT,
            param1=data.x,
            param2=data.y,
            param3=data.z,
        )

    async def do_stop(self, data):
        """Halt tracking or any other motion."""
        self.assert_enabled()
        async with self.write_lock:
            self.move_task.cancel()
            self.compensation_loop_task.cancel()
        await self.run_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.STOP,
        )

    async def basic_run_command(self, command):
        # Overload of lsst.ts.hexrotcomm.BaseCsc's version
        # that resets the nstopped attribute.
        self.nstopped = 0
        self.log.debug(
            f"send low-level command {enums.CommandCode(command.code)!r}; "
            f"params={command.param1}, {command.param2}, {command.param3}, "
            f"{command.param4}, {command.param5}, {command.param6}"
        )
        await super().basic_run_command(command)
        self.nstopped = 0

    async def start(self):
        await asyncio.gather(self.mtmount.start_task, self.mtrotator.start_task)
        self.evt_compensationMode.set_put(enabled=False)
        await super().start()

    def telemetry_callback(self, server):
        """Called when the low-level controller outputs telemetry.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        did_change = self.evt_summaryState.set_put(summaryState=self.summary_state)
        if did_change and self.summary_state != salobj.State.ENABLED:
            self.disable_compensation()

        # Strangely telemetry.state, offline_substate and enabled_substate
        # are all floats from the controller. But they should only have
        # integer value, so I output them as integers.
        self.evt_controllerState.set_put(
            controllerState=int(server.telemetry.state),
            offlineSubstate=int(server.telemetry.offline_substate),
            enabledSubstate=int(server.telemetry.enabled_substate),
            applicationStatus=[server.telemetry.application_status, 0, 0, 0, 0, 0],
        )

        pos_error = [
            server.telemetry.measured_pos[i] - server.telemetry.commanded_pos[i]
            for i in range(6)
        ]
        self.tel_actuators.set_put(
            calibrated=server.telemetry.strut_encoder_microns,
            raw=server.telemetry.strut_encoder_raw,
        )
        self.tel_application.set_put(
            demand=server.telemetry.commanded_pos,
            position=server.telemetry.measured_pos,
            error=pos_error,
        )
        self.tel_electrical.set_put(
            copleyStatusWordDrive=server.telemetry.status_word,
            copleyLatchingFaultStatus=server.telemetry.latching_fault_status_register,
        )

        in_position = (
            server.telemetry.application_status
            & ApplicationStatus.HEX_MOVE_COMPLETE_MASK
        )
        self.evt_inPosition.set_put(inPosition=in_position)

        self.evt_commandableByDDS.set_put(
            state=bool(
                server.telemetry.application_status
                & ApplicationStatus.DDS_COMMAND_SOURCE
            )
        )

        safety_interlock = (
            server.telemetry.application_status & ApplicationStatus.SAFETY_INTERLOCK
        )
        self.evt_interlock.set_put(
            detail="Engaged" if safety_interlock else "Disengaged"
        )

        if (
            self.evt_controllerState.data.controllerState != ControllerState.ENABLED
            or self.evt_controllerState.data.enabledSubstate
            != EnabledSubstate.STATIONARY
        ):
            self.nstopped = 0
        elif self.nstopped < MAXIMUM_WAIT_STOPPED:
            self.nstopped += 1
        self.telemetry_event.set()

    def make_mock_controller(self, initial_ctrl_state):
        return mock_controller.MockMTHexapodController(
            log=self.log,
            index=self.salinfo.index,
            initial_state=initial_ctrl_state,
            command_port=self.server.command_port,
            telemetry_port=self.server.telemetry_port,
        )

    def disable_compensation(self):
        """Cancel compensation and report compensationMode.enabled=False."""
        # Since this is a synchronous method, it is not safe to cancel the
        # compensation loop (there is no way to obtain the write lock).
        # So cancel the compensation wait task, instead.
        self.compensation_wait_task.cancel()
        self.evt_compensationMode.set_put(enabled=False)

    def _make_position_set_command(self, position):
        """Make a POSITION_SET command for the low-level controller.

        Parameters
        ----------
        position : `Position`
            Desired position.
        """
        utils.check_position(
            position=position,
            limits=self.current_pos_limits,
            ExceptionClass=salobj.ExpectedError,
        )
        command_kwargs = {
            f"param{i+1}": value
            for i, value in enumerate(dataclasses.astuple(position))
        }
        return self.make_command(code=enums.CommandCode.POSITION_SET, **command_kwargs)

    def _has_uncompensated_position(self):
        """Return True if the uncompensated position has been set,
        e.g. by a move command.
        """
        return self.evt_uncompensatedPosition.has_data

    def _get_uncompensated_position(self):
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
        if uncompensated_data is None:
            raise salobj.ExpectedError("No uncompensated position to offset from")
        return base.Position.from_struct(uncompensated_data)

    async def stop_motion(self):
        """Stop motion and wait for it to stop.

        Raises:
            asyncio.CancelledError if not in enabled state.
        """
        if self.server.telemetry.state != ControllerState.ENABLED:
            raise asyncio.CancelledError("Not enabled")

        # If already stopped then we are done, but give the current command
        # (if any) time to influence telemetry before deciding.
        is_stopped = await self.wait_stopped(max_ntelem=2)
        if is_stopped:
            return

        # Stop the current motion, unless it is already being stopped.
        if (
            self.server.telemetry.enabled_substate
            != EnabledSubstate.CONTROLLED_STOPPING
        ):
            await self.run_command(
                code=enums.CommandCode.SET_ENABLED_SUBSTATE,
                param1=enums.SetEnabledSubstateParam.STOP,
            )

        # Wait for stop.
        await self.wait_stopped()

    async def wait_stopped(self, min_nstopped=2, max_ntelem=None):
        """Wait for the current motion, if any, to stop.

        Parameters
        ----------
        min_nstopped : `int`, optional
            Minimum number of consecutive stopped states.
            Must be positive and should probably be at least 2, so that
            any command you started has a chance to affect telemetry.
        max_ntelem : `int` or `None`, optional
            The maximum number of telemetry messages to wait for.
            If None then no limit and this method always returns True.

        Returns
        -------
        is_stopped : `bool`
            Is motion stopped? Always True unless max_nstatus is not None.

        Raises:
            asyncio.CancelledError if not in enabled state.
        """
        if min_nstopped < 1:
            raise ValueError(f"min_nstopped={min_nstopped} must be positive")
        if max_ntelem is not None and max_ntelem < 1:
            raise ValueError(f"max_ntelem={max_ntelem} must be positive or None")

        if self.server.telemetry.state != ControllerState.ENABLED:
            raise asyncio.CancelledError()

        ntelem = 0
        while self.nstopped < min_nstopped:
            self.telemetry_event.clear()
            await self.telemetry_event.wait()
            if self.server.telemetry.state != ControllerState.ENABLED:
                raise asyncio.CancelledError()
            ntelem += 1
            if max_ntelem is not None and ntelem > max_ntelem:
                return False

        return True

    async def _move(self, uncompensated_pos, sync, start_compensation):
        """Command a move and output appropriate events.

        Parameters
        ----------
        uncompensated_pos : `dict` [`str`, `float`]
            Target position (without compensation applied)
            as a dict of axis name: position
            keys are x, y, z (um), u, v, w (deg).
        sync : `bool`
            Should this be a synchronized move? Usually True.
        start_compensation : `bool`
            If True and if compensation mode is enabled,
            then start the compensation loop at the end
            (on success or failure, but not if cancelled).
            Ignored if compensation mode is disabled.
        """
        try:
            compensation_info = self.compute_compensation(uncompensated_pos)

            # Stop the current motion, if any, and wait for it to stop.
            await asyncio.wait_for(self.stop_motion(), timeout=MAXIMUM_STOP_TIME)

            # Command the new motion.
            cmd1 = self._make_position_set_command(compensation_info.compensated_pos)
            cmd2 = self.make_command(
                code=enums.CommandCode.SET_ENABLED_SUBSTATE,
                param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
                param2=sync,
            )
            await self.run_multiple_commands(cmd1, cmd2)

            self.evt_uncompensatedPosition.set_put(**vars(uncompensated_pos))
            self.evt_compensatedPosition.set_put(
                **vars(compensation_info.compensated_pos)
            )
            if compensation_info.compensation_offset is not None:
                self.evt_compensationOffset.set_put(
                    elevation=compensation_info.compensation_inputs.elevation,
                    azimuth=compensation_info.compensation_inputs.azimuth,
                    rotation=compensation_info.compensation_inputs.rotation,
                    temperature=compensation_info.compensation_inputs.temperature,
                    **vars(compensation_info.compensation_offset),
                )
        except asyncio.CancelledError:
            raise
        except Exception:
            # This move failed; restart the compensation loop anyway,
            # if it is wanted.
            if start_compensation and self.compensation_mode:
                self.compensation_loop_task = asyncio.create_task(
                    self.compensation_loop()
                )
            raise

        if start_compensation and self.compensation_mode:
            self.compensation_loop_task = asyncio.create_task(self.compensation_loop())
