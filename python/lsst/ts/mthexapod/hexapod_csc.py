# This file is part of ts_mthexapod.
#
# Developed for the LSST Data Management System.
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
import copy
import pathlib
import types

import numpy as np

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from lsst.ts.idl.enums.MTHexapod import EnabledSubstate, ApplicationStatus
from . import constants
from . import enums
from . import structs
from . import compensation
from . import utils
from . import mock_controller


class ControllerConstants:
    """Constants needed to communicate with an MTHexapod controller.
    """

    def __init__(self, sync_pattern, config_frame_id, telemetry_frame_id):
        self.sync_pattern = sync_pattern
        self.config_frame_id = config_frame_id
        self.telemetry_frame_id = telemetry_frame_id


# Dict of SalIndex: ControllerConstants
IndexControllerConstants = {
    enums.SalIndex.CAMERA_HEXAPOD: ControllerConstants(
        sync_pattern=constants.CAM_SYNC_PATTERN,
        config_frame_id=enums.FrameId.CAM_CONFIG,
        telemetry_frame_id=enums.FrameId.CAM_TELEMETRY,
    ),
    enums.SalIndex.M2_HEXAPOD: ControllerConstants(
        sync_pattern=constants.M2_SYNC_PATTERN,
        config_frame_id=enums.FrameId.M2_CONFIG,
        telemetry_frame_id=enums.FrameId.M2_TELEMETRY,
    ),
}


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

    def __init__(
        self,
        index,
        config_dir=None,
        initial_state=salobj.State.OFFLINE,
        simulation_mode=0,
    ):
        index = enums.SalIndex(index)
        controller_constants = IndexControllerConstants[index]
        self.xy_max_limit = constants.XY_MAX_LIMIT[index - 1]
        self.z_min_limit = constants.Z_MIN_LIMIT[index - 1]
        self.z_max_limit = constants.Z_MAX_LIMIT[index - 1]
        self.uv_max_limit = constants.UV_MAX_LIMIT[index - 1]
        self.w_min_limit = constants.W_MIN_LIMIT[index - 1]
        self.w_max_limit = constants.W_MAX_LIMIT[index - 1]
        self._axis_names = ("x", "y", "z", "u", "v", "w")

        structs.Config.FRAME_ID = controller_constants.config_frame_id
        structs.Telemetry.FRAME_ID = controller_constants.telemetry_frame_id

        schema_path = pathlib.Path(__file__).parents[4] / "schema" / "MTHexapod.yaml"
        super().__init__(
            name="MTHexapod",
            index=index,
            sync_pattern=controller_constants.sync_pattern,
            CommandCode=enums.CommandCode,
            ConfigClass=structs.Config,
            TelemetryClass=structs.Telemetry,
            schema_path=schema_path,
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

    async def configure(self, config):
        subconfig_name = {
            enums.SalIndex.CAMERA_HEXAPOD: "camera_config",
            enums.SalIndex.M2_HEXAPOD: "m2_config",
        }[self.salinfo.index]
        subconfig = types.SimpleNamespace(**getattr(config, subconfig_name))
        self.compensation = compensation.Compensation(
            elevation_coeffs=subconfig.elevation_coeffs,
            temperature_coeffs=subconfig.temperature_coeffs,
            min_temperature=subconfig.min_temperature,
            max_temperature=subconfig.max_temperature,
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
        utils.check_positive_value(
            data.maxXY, "maxXY", self.xy_max_limit, ExceptionClass=salobj.ExpectedError
        )
        utils.check_negative_value(
            data.minZ, "minZ", self.z_min_limit, ExceptionClass=salobj.ExpectedError
        )
        utils.check_positive_value(
            data.maxZ, "maxZ", self.z_max_limit, ExceptionClass=salobj.ExpectedError
        )
        utils.check_positive_value(
            data.maxUV, "maxUV", self.uv_max_limit, ExceptionClass=salobj.ExpectedError
        )
        utils.check_negative_value(
            data.minW, "minW", self.w_min_limit, ExceptionClass=salobj.ExpectedError
        )
        utils.check_positive_value(
            data.maxW, "maxW", self.w_max_limit, ExceptionClass=salobj.ExpectedError
        )
        await self.run_command(
            code=enums.CommandCode.CONFIG_LIMITS,
            param1=data.maxXY,
            param2=data.minZ,
            param3=data.maxZ,
            param4=data.maxUV,
            param5=data.minW,
            param6=data.maxW,
        )

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
        """
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        cmd1 = self._make_position_set_command(data)
        cmd2 = self.make_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
            param2=data.sync,
        )
        await self.run_multiple_commands(cmd1, cmd2)
        position = [getattr(data, name) for name in self._axis_names]
        self.put_target_event(uncompensated_position=position)

    async def do_moveWithCompensation(self, data):
        """Move to a specified position and orientation,
        with compensation for telescope elevation, azimuth and temperature.
        """
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        compensation_offsets = self.compensation.get_offsets(
            azimuth=data.azimuth,
            elevation=data.elevation,
            temperature=data.temperature,
        )
        uncompensated_position = [getattr(data, name) for name in self._axis_names]
        compensated_position = np.add(uncompensated_position, compensation_offsets)
        for i, name in enumerate(self._axis_names):
            setattr(data, name, compensated_position[i])
        cmd1 = self._make_position_set_command(data)
        cmd2 = self.make_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
            param2=data.sync,
        )
        await self.run_multiple_commands(cmd1, cmd2)
        self.put_target_event(
            uncompensated_position=uncompensated_position,
            compensated_position=compensated_position,
            elevation=data.elevation,
            azimuth=data.azimuth,
            temperature=data.temperature,
        )

    async def do_offset(self, data):
        """Move by a specified offset in position and orientation.
        """
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        cmd1 = self._make_offset_set_command(data)
        absolute_position = [getattr(cmd1, f"param{i+1}") for i in range(6)]
        cmd2 = self.make_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
            param2=data.sync,
        )
        await self.run_multiple_commands(cmd1, cmd2)
        self.put_target_event(uncompensated_position=absolute_position)

    async def do_pivot(self, data):
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

    def telemetry_callback(self, server):
        """Called when the low-level controller outputs telemetry.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        self.evt_summaryState.set_put(summaryState=self.summary_state)
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

    def put_target_event(
        self,
        uncompensated_position,
        compensated_position=None,
        elevation=0,
        azimuth=0,
        temperature=0,
    ):
        """Output the target event.

        Parameters
        ----------
        uncompensated_position : `List` [`float`]
            Uncompensated x, y, z (um), u, v, w (deg)
        compensated_position : `List` [`float`] or None, optional
            Commpensated x, y, z (um), u, v, w (deg).
            If None then assumed equal to ``uncompensated_position``
            and the compensated flag is set False.
        elevation : `float`, optional
            Telescope elevation (deg).
            Only relevant if compensated_position is not None.
        azimuth : `float`, optional
            Telescope azimuth (deg).
            Only relevant if compensated_position is not None.
        temperature : `float`, optional
            Ambient temperature (C).
            Only relevant if compensated_position is not None.
        """
        uncomp_kwargs = {
            "uncompensated" + name.upper(): uncompensated_position[i]
            for i, name in enumerate(self._axis_names)
        }
        if compensated_position is None:
            compensated_position = uncompensated_position
            compensated = False
        else:
            compensated = True
        comp_kwargs = {
            name: compensated_position[i] for i, name in enumerate(self._axis_names)
        }
        self.evt_target.set_put(
            compensated=compensated,
            elevation=elevation,
            azimuth=azimuth,
            temperature=temperature,
            **uncomp_kwargs,
            **comp_kwargs,
            force_output=True,
        )

    def _make_position_set_command(self, data):
        """Make a POSITION_SET command for the low-level controller.

        Parameters
        ----------
        data : ``struct with x, y, z, u, v, w`` fields.
            Data from the ``move`` or ``do_moveWithCompensation`` command.
            May also be data from the ``offset`` command,
            but the fields must be changed to absolute positions.
        """
        utils.check_symmetrical_range(
            data.x,
            "x",
            self.server.config.pos_limits[0],
            ExceptionClass=salobj.ExpectedError,
        )
        utils.check_symmetrical_range(
            data.y,
            "y",
            self.server.config.pos_limits[0],
            ExceptionClass=salobj.ExpectedError,
        )
        utils.check_range(
            data.z,
            "z",
            self.server.config.pos_limits[1],
            self.server.config.pos_limits[2],
            ExceptionClass=salobj.ExpectedError,
        )
        utils.check_symmetrical_range(
            data.u,
            "u",
            self.server.config.pos_limits[3],
            ExceptionClass=salobj.ExpectedError,
        )
        utils.check_symmetrical_range(
            data.v,
            "v",
            self.server.config.pos_limits[3],
            ExceptionClass=salobj.ExpectedError,
        )
        utils.check_range(
            data.w,
            "w",
            self.server.config.pos_limits[4],
            self.server.config.pos_limits[5],
            ExceptionClass=salobj.ExpectedError,
        )
        return self.make_command(
            code=enums.CommandCode.POSITION_SET,
            param1=data.x,
            param2=data.y,
            param3=data.z,
            param4=data.u,
            param5=data.v,
            param6=data.w,
        )

    def _make_offset_set_command(self, data):
        """Make a POSITION_SET offset command for the low-level controller
        using data from the offset or offsetLUT CSC command.
        """
        position_data = copy.copy(data)
        position_data.x += self.server.telemetry.commanded_pos[0]
        position_data.y += self.server.telemetry.commanded_pos[1]
        position_data.z += self.server.telemetry.commanded_pos[2]
        position_data.u += self.server.telemetry.commanded_pos[3]
        position_data.v += self.server.telemetry.commanded_pos[4]
        position_data.w += self.server.telemetry.commanded_pos[5]

        return self._make_position_set_command(position_data)
