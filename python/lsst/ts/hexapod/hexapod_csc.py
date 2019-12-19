# This file is part of ts_hexapod.
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

import argparse
import copy

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from lsst.ts.idl.enums import Hexapod
from . import constants
from . import enums
from . import structs
from . import utils
from . import mock_controller


class ControllerConstants:
    """Constants needed to communicate with a hexapod controller.
    """
    def __init__(self, sync_pattern, config_frame_id, telemetry_frame_id):
        self.sync_pattern = sync_pattern
        self.config_frame_id = config_frame_id
        self.telemetry_frame_id = telemetry_frame_id


# Dict of SalIndex: ControllerConstants
IndexControllerConstants = {
    enums.SalIndex.CAM_HEXAPOD: ControllerConstants(sync_pattern=constants.CAM_SYNC_PATTERN,
                                                    config_frame_id=enums.FrameId.CAM_CONFIG,
                                                    telemetry_frame_id=enums.FrameId.CAM_TELEMETRY),
    enums.SalIndex.M2_HEXAPOD: ControllerConstants(sync_pattern=constants.M2_SYNC_PATTERN,
                                                   config_frame_id=enums.FrameId.M2_CONFIG,
                                                   telemetry_frame_id=enums.FrameId.M2_TELEMETRY),
}


class HexapodCsc(hexrotcomm.BaseCsc):
    """MT hexapod CSC.

    Parameters
    ----------
    index : `SalIndex` or `int`
        SAL index; see `SalIndex` for the allowed values.
    initial_state : `lsst.ts.salobj.State` or `int` (optional)
        The initial state of the CSC. Ignored (other than checking
        that it is a valid value) except in simulation mode,
        because in normal operation the initial state is the current state
        of the controller. This is provided for unit testing.
    simulation_mode : `int` (optional)
        Simulation mode. Allowed values:

        * 0: regular operation.
        * 1: simulation: use a mock low level controller.

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

    def __init__(self,
                 index,
                 initial_state=salobj.State.OFFLINE,
                 simulation_mode=0):
        index = enums.SalIndex(index)
        controller_constants = IndexControllerConstants[index]
        self.xy_max_limit = constants.XY_MAX_LIMIT[index-1]
        self.z_min_limit = constants.Z_MIN_LIMIT[index-1]
        self.z_max_limit = constants.Z_MAX_LIMIT[index-1]
        self.uv_max_limit = constants.UV_MAX_LIMIT[index-1]
        self.w_min_limit = constants.W_MIN_LIMIT[index-1]
        self.w_max_limit = constants.W_MAX_LIMIT[index-1]

        # Set this True or False for an offset or positionSet command.
        # Reset this to None when offset or positionSet are run
        # or when no longer enabled and stationary.
        # Thus it serves two purposes:
        # * Record a value needed by the move and offset commands.
        # * Record the fact that a position has been specified
        self.synchronized_move = None
        structs.Config.FRAME_ID = controller_constants.config_frame_id
        structs.Telemetry.FRAME_ID = controller_constants.telemetry_frame_id

        super().__init__(name="Hexapod",
                         index=index,
                         sync_pattern=controller_constants.sync_pattern,
                         CommandCode=enums.CommandCode,
                         ConfigClass=structs.Config,
                         TelemetryClass=structs.Telemetry,
                         initial_state=initial_state,
                         simulation_mode=simulation_mode)

    # Hexapod-specific commands.
    async def do_move(self, data):
        """Go to the position specified by the most recent ``positionSet``
        or ``offset`` command.
        """
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        if self.synchronized_move is None:
            raise salobj.ExpectedError("Must specify a position with positionSet or offset.")
        sync_move = self.synchronized_move
        self.synchronized_move = None
        await self.run_command(code=enums.CommandCode.SET_ENABLED_SUBSTATE,
                               param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
                               param2=sync_move)

    async def do_moveLUT(self, data):
        """Go to the position specified by the most recent ``positionSet``
        or `offset`` command, with LUT corrections.
        """
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        if self.synchronized_move is None:
            raise salobj.ExpectedError("Must specify a position with positionSet or offset.")
        sync_move = self.synchronized_move
        self.synchronized_move = None
        await self.run_command(code=enums.CommandCode.SET_ENABLED_SUBSTATE,
                               param1=enums.SetEnabledSubstateParam.MOVE_LUT,
                               param2=sync_move,
                               param3=data.az,
                               param4=data.elev,
                               param5=data.temp)

    async def do_offset(self, data):
        """Specify an offset for the ``move`` or ``moveLUT`` command.
        """
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        offset_data = copy.copy(data)
        offset_data.x += self.server.telemetry.commanded_pos[0]
        offset_data.y += self.server.telemetry.commanded_pos[1]
        offset_data.z += self.server.telemetry.commanded_pos[2]
        offset_data.u += self.server.telemetry.commanded_pos[3]
        offset_data.v += self.server.telemetry.commanded_pos[4]
        offset_data.w += self.server.telemetry.commanded_pos[5]

        utils.check_symmetrical_range(offset_data.x, "x", self.server.config.pos_limits[0],
                                      ExceptionClass=salobj.ExpectedError)
        utils.check_symmetrical_range(offset_data.y, "y", self.server.config.pos_limits[0],
                                      ExceptionClass=salobj.ExpectedError)
        utils.check_range(offset_data.z, "z", self.server.config.pos_limits[1],
                          self.server.config.pos_limits[2], ExceptionClass=salobj.ExpectedError)
        utils.check_symmetrical_range(offset_data.u, "u", self.server.config.pos_limits[3],
                                      ExceptionClass=salobj.ExpectedError)
        utils.check_symmetrical_range(offset_data.v, "v", self.server.config.pos_limits[3],
                                      ExceptionClass=salobj.ExpectedError)
        utils.check_range(offset_data.w, "w", self.server.config.pos_limits[4],
                          self.server.config.pos_limits[5], ExceptionClass=salobj.ExpectedError)
        self.synchronized_move = data.sync
        await self.run_command(code=enums.CommandCode.POSITION_SET,
                               param1=offset_data.x,
                               param2=offset_data.y,
                               param3=offset_data.z,
                               param4=offset_data.u,
                               param5=offset_data.v,
                               param6=offset_data.w)

    async def do_pivot(self, data):
        """Set the coordinates of the pivot point."""
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        await self.run_command(code=enums.CommandCode.SET_PIVOTPOINT,
                               param1=data.x,
                               param2=data.y,
                               param3=data.z)

    async def do_positionSet(self, data):
        """Specify a position for the ``move`` or ``moveLUT`` command.
        """
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        utils.check_symmetrical_range(data.x, "x", self.server.config.pos_limits[0],
                                      ExceptionClass=salobj.ExpectedError)
        utils.check_symmetrical_range(data.y, "y", self.server.config.pos_limits[0],
                                      ExceptionClass=salobj.ExpectedError)
        utils.check_range(data.z, "z", self.server.config.pos_limits[1],
                          self.server.config.pos_limits[2],
                          ExceptionClass=salobj.ExpectedError)
        utils.check_symmetrical_range(data.u, "u", self.server.config.pos_limits[3],
                                      ExceptionClass=salobj.ExpectedError)
        utils.check_symmetrical_range(data.v, "v", self.server.config.pos_limits[3],
                                      ExceptionClass=salobj.ExpectedError)
        utils.check_range(data.w, "w", self.server.config.pos_limits[4],
                          self.server.config.pos_limits[5],
                          ExceptionClass=salobj.ExpectedError)
        self.synchronized_move = data.sync
        await self.run_command(code=enums.CommandCode.POSITION_SET,
                               param1=data.x,
                               param2=data.y,
                               param3=data.z,
                               param4=data.u,
                               param5=data.v,
                               param6=data.w)

    async def do_stop(self, data):
        """Halt tracking or any other motion.
        """
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        self.synchronized_move = None
        await self.run_command(code=enums.CommandCode.SET_ENABLED_SUBSTATE,
                               param1=enums.SetEnabledSubstateParam.STOP)

    async def do_test(self, data):
        """Execute the test command. NOT SUPPORTED.
        """
        raise salobj.ExpectedError("Not implemented")
        # self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        # # The test command is unique in that all fields must be left
        # # at their initialized value except sync_pattern
        # # (at least that is what the Vendor's code does).
        # command = structs.Command()
        # command.sync_pattern = structs.ROTATOR_SYNC_PATTERN
        # await self.server.run_command(command)

    def config_callback(self, server):
        """Called when the TCP/IP controller outputs configuration.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        self.evt_configuration.set_put(
            accelerationAccmax=server.config.strut_acceleration,
            limitXYMax=server.config.pos_limits[0],
            limitZMin=server.config.pos_limits[1],
            limitZMax=server.config.pos_limits[2],
            limitUVMax=server.config.pos_limits[3],
            limitWMin=server.config.pos_limits[4],
            limitWMax=server.config.pos_limits[5],
            velocityXYMax=server.config.vel_limits[0],
            velocityRxRyMax=server.config.vel_limits[1],
            velocityZMax=server.config.vel_limits[2],
            velocityRzMax=server.config.vel_limits[3],
            positionX=server.config.initial_pos[0],
            positionY=server.config.initial_pos[1],
            positionZ=server.config.initial_pos[2],
            positionU=server.config.initial_pos[3],
            positionV=server.config.initial_pos[4],
            positionW=server.config.initial_pos[5],
            pivotX=server.config.pivot[0],
            pivotY=server.config.pivot[1],
            pivotZ=server.config.pivot[2],
            elevationRawLUTElevIndex=[int(index) for index in server.config.el_lut_index],
            elevationRawLUTX=server.config.el_lut_x,
            elevationRawLUTY=server.config.el_lut_y,
            elevationRawLUTZ=server.config.el_lut_z,
            elevationRawLUTRx=server.config.el_lut_rx,
            elevationRawLUTRy=server.config.el_lut_ry,
            elevationRawLUTRz=server.config.el_lut_rz,
            azimuthRawLUTAzIndex=[int(index) for index in server.config.az_lut_index],
            azimuthRawLUTX=server.config.az_lut_x,
            azimuthRawLUTY=server.config.az_lut_y,
            azimuthRawLUTZ=server.config.az_lut_z,
            azimuthRawLUTRx=server.config.az_lut_rx,
            azimuthRawLUTRy=server.config.az_lut_ry,
            azimuthRawLUTRz=server.config.az_lut_rz,
            temperatureRawLUTTempIndex=[int(index) for index in server.config.temp_lut_index],
            temperatureRawLUTX=server.config.temp_lut_x,
            temperatureRawLUTY=server.config.temp_lut_y,
            temperatureRawLUTZ=server.config.temp_lut_z,
            temperatureRawLUTRx=server.config.temp_lut_rx,
            temperatureRawLUTRy=server.config.temp_lut_ry,
            temperatureRawLUTRz=server.config.temp_lut_rz,
            strutDisplacementMax=server.config.strut_displacement_max,
            strutVelocityMax=server.config.strut_velocity_max,
        )

    def telemetry_callback(self, server):
        """Called when the TCP/IP controller outputs telemetry.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        if server.telemetry.state != Hexapod.ControllerState.ENABLED or \
                server.telemetry.enabled_substate != Hexapod.EnabledSubstate.STATIONARY:
            self.synchronized_move = None
        self.evt_summaryState.set_put(summaryState=self.summary_state)
        # Strangely telemetry.state, offline_substate and enabled_substate
        # are all floats from the controller. But they should only have
        # integer value, so I output them as integers.
        self.evt_controllerState.set_put(controllerState=int(server.telemetry.state),
                                         offlineSubstate=int(server.telemetry.offline_substate),
                                         enabledSubstate=int(server.telemetry.enabled_substate),
                                         applicationStatus=server.telemetry.application_status)

        pos_error = [server.telemetry.measured_pos[i] - server.telemetry.commanded_pos[i] for i in range(6)]
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

        actuator_in_position = tuple(status & Hexapod.ApplicationStatus.HEX_MOVE_COMPLETE_MASK
                                     for status in server.telemetry.application_status)
        self.evt_actuatorInPosition.set_put(
            inPosition=actuator_in_position,
        )
        self.evt_inPosition.set_put(
            inPosition=all(actuator_in_position),
        )

        self.evt_commandableByDDS.set_put(
            state=bool(server.telemetry.application_status[0] & Hexapod.ApplicationStatus.DDS_COMMAND_SOURCE),
        )

        safety_interlock = server.telemetry.application_status[0] & Hexapod.ApplicationStatus.SAFTEY_INTERLOCK
        self.evt_interlock.set_put(
            detail="Engaged" if safety_interlock else "Disengaged",
        )

    def make_mock_controller(self, initial_ctrl_state):
        return mock_controller.MockMTHexapodController(
            log=self.log,
            index=self.salinfo.index,
            initial_state=initial_ctrl_state,
            command_port=self.server.command_port,
            telemetry_port=self.server.telemetry_port)

    @classmethod
    async def amain(cls):
        """Make a CSC from command-line arguments and run it.
        """
        parser = argparse.ArgumentParser(f"Run {cls.__name__}")
        parser.add_argument("index", type=int,
                            help="Hexapod index; 1=Camera 2=M2")
        parser.add_argument("-s", "--simulate", action="store_true",
                            help="Run in simulation mode?")

        args = parser.parse_args()
        csc = cls(index=args.index, simulation_mode=int(args.simulate))
        await csc.done_task
