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
import pathlib
import types

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from lsst.ts.idl.enums.MTHexapod import EnabledSubstate, ApplicationStatus
from . import __version__
from . import base
from . import compensation
from . import constants
from . import enums
from . import mock_controller
from . import structs
from . import utils


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

        # If the compensation loop starts and there are missing
        # inputs then warn once and set this flag true.
        self.missing_inputs_str = ""

        # Interval between compensation updates.
        # Set in `configure`, but we need something now.
        self.compensation_interval = 0.2

        self.compensation_wait_task = salobj.make_done_future()

        structs.Config.FRAME_ID = controller_constants.config_frame_id
        structs.Telemetry.FRAME_ID = controller_constants.telemetry_frame_id

        schema_path = pathlib.Path(__file__).parents[4] / "schema" / "MTHexapod.yaml"
        super().__init__(
            name="MTHexapod",
            index=index,
            port=controller_constants.port,
            sync_pattern=controller_constants.sync_pattern,
            CommandCode=enums.CommandCode,
            ConfigClass=structs.Config,
            TelemetryClass=structs.Telemetry,
            schema_path=schema_path,
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

    def bump_compensation_loop(self, wait_first):
        """Stop and, if appropriate, restart the compensation loop.

        Parameters
        ----------
        wait_first : `bool`
            If True then wait_first to apply the first compensation correction.
            This is appropriate for do_move and do_offset
            (since they perform a compensated move, if appropriate),
            but not for do_setCompensationMode.
        """
        self.compensation_wait_task.cancel()
        if self.compensation_mode:
            asyncio.create_task(self.compensation_loop(wait_first=wait_first))

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
            self.stop_compensation()

    async def compensation_loop(self, wait_first):
        """Apply compensation at regular intervals.

        Parameters
        ----------
        wait_first : `bool`
            Wait for the first iteration?
            Set True if called by the move or offset command,
            set False if called by the setCompensationMode command.

        Notes
        -----
        The interval between compensation updates is a configuration parameter.

        This will skip a compensation update if the hexapod is moving
        (and log a debug-level message). That will be common after
        a large move.
        """
        self.compensation_wait_task.cancel()
        do_wait = wait_first
        while self.summary_state == salobj.State.ENABLED:
            if do_wait:
                self.compensation_wait_task = asyncio.create_task(
                    asyncio.sleep(self.compensation_interval)
                )
                await self.compensation_wait_task
                if self.summary_state != salobj.State.ENABLED:
                    return
            else:
                do_wait = True

            # Apply a compensation move, if movement is allowed.
            if self.server.telemetry.enabled_substate != EnabledSubstate.STATIONARY:
                # Cast the float value for nicer output
                enabled_substate = EnabledSubstate(
                    self.server.telemetry.enabled_substate
                )
                self.log.debug(
                    f"Skip compensation; enabled_substate={enabled_substate!r}"
                )
                continue
            if not self._has_uncompensated_position():
                self.log.error("Compensation failed; no position has been commanded")
                return
            try:
                self.log.debug("Apply compensation")
                uncompensated_pos = self._get_uncompensated_position()
                await self._move(uncompensated_pos=uncompensated_pos, sync=1)
            except asyncio.CancelledError:
                # Normal termination. This may be temporary (e.g.
                # when starting a move or offset command) so do not
                # report compensation mode disabled.
                return
            except Exception:
                self.log.exception("Compensation failed; turning off compensation mode")
                self.evt_compensationMode.set_put(enabled=False)
                return

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
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        uncompensated_pos = base.Position.from_struct(data)
        utils.check_position(
            position=uncompensated_pos,
            limits=self.current_pos_limits,
            ExceptionClass=salobj.ExpectedError,
        )
        self.bump_compensation_loop(wait_first=True)
        await self._move(uncompensated_pos=uncompensated_pos, sync=data.sync)

    async def do_offset(self, data):
        """Move by a specified offset in position and orientation.

        See note for do_move regarding checking the target position.
        """
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        curr_uncompensated_pos = self._get_uncompensated_position()
        offset = base.Position.from_struct(data)
        uncompensated_pos = curr_uncompensated_pos + offset
        utils.check_position(
            position=uncompensated_pos,
            limits=self.current_pos_limits,
            ExceptionClass=salobj.ExpectedError,
        )
        self.bump_compensation_loop(wait_first=True)
        await self._move(uncompensated_pos=uncompensated_pos, sync=data.sync)

    async def do_setCompensationMode(self, data):
        self.assert_enabled()
        self.evt_compensationMode.set_put(enabled=data.enable)
        if data.enable:
            if self._has_uncompensated_position():
                self.bump_compensation_loop(wait_first=False)
            else:
                self.compensation_wait_task.cancel()
        else:
            self.stop_compensation()
            try:
                uncompensated_pos = self._get_uncompensated_position()
            except salobj.ExpectedError:
                self.log.info("There is no compensation offset to remove")
            else:
                self.log.info("Removing the current compensation offset")
                await self._move(uncompensated_pos=uncompensated_pos, sync=1)

    async def do_setPivot(self, data):
        """Set the coordinates of the pivot point.
        """
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        await self.run_command(
            code=enums.CommandCode.SET_PIVOTPOINT,
            param1=data.x,
            param2=data.y,
            param3=data.z,
        )

    async def do_stop(self, data):
        """Halt tracking or any other motion.
        """
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        await self.run_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.STOP,
        )

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
            self.stop_compensation()

        # Strangely telemetry.state, offline_substate and enabled_substate
        # are all floats from the controller. But they should only have
        # integer value, so I output them as integers.
        self.evt_controllerState.set_put(
            controllerState=int(server.telemetry.state),
            offlineSubstate=int(server.telemetry.offline_substate),
            enabledSubstate=int(server.telemetry.enabled_substate),
            applicationStatus=server.telemetry.application_status,
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

        actuator_in_position = tuple(
            status & ApplicationStatus.HEX_MOVE_COMPLETE_MASK
            for status in server.telemetry.application_status
        )
        self.evt_actuatorInPosition.set_put(inPosition=actuator_in_position)
        self.evt_inPosition.set_put(inPosition=all(actuator_in_position))

        self.evt_commandableByDDS.set_put(
            state=bool(
                server.telemetry.application_status[0]
                & ApplicationStatus.DDS_COMMAND_SOURCE
            )
        )

        safety_interlock = (
            server.telemetry.application_status[0] & ApplicationStatus.SAFETY_INTERLOCK
        )
        self.evt_interlock.set_put(
            detail="Engaged" if safety_interlock else "Disengaged"
        )

    def make_mock_controller(self, initial_ctrl_state):
        return mock_controller.MockMTHexapodController(
            log=self.log,
            index=self.salinfo.index,
            initial_state=initial_ctrl_state,
            command_port=self.server.command_port,
            telemetry_port=self.server.telemetry_port,
        )

    def stop_compensation(self):
        """Stop the compensation loop."""
        self.compensation_wait_task.cancel()
        self.compensate_position = False
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

    async def _move(self, uncompensated_pos, sync):
        """Command a move and output appropriate events.

        Parameters
        ----------
        uncompensated_pos : `dict` [`str`, `float`]
            Target position (without compensation applied)
            as a dict of axis name: position
            keys are x, y, z (um), u, v, w (deg).
        sync : `bool`
            Should this be a synchronized move? Usually True.
        """
        compensation_offset = None
        if self.compensation_mode:
            compensation_input = self.get_compensation_inputs()
            if compensation_input is not None:
                compensation_offset = self.compensation.get_offset(compensation_input)

        if compensation_offset is not None:
            compensated_pos = uncompensated_pos + compensation_offset
        else:
            compensated_pos = uncompensated_pos

        cmd1 = self._make_position_set_command(compensated_pos)
        cmd2 = self.make_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
            param2=sync,
        )
        await self.run_multiple_commands(cmd1, cmd2)

        self.evt_uncompensatedPosition.set_put(**vars(uncompensated_pos))
        self.evt_compensatedPosition.set_put(**vars(compensated_pos))
        if compensation_offset is not None:
            self.evt_compensationOffset.set_put(
                elevation=compensation_input.elevation,
                azimuth=compensation_input.azimuth,
                rotation=compensation_input.rotation,
                temperature=compensation_input.temperature,
                **vars(compensation_offset),
            )
