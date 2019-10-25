# This file is part of ts_rotator.
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
import asyncio
import copy

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from lsst.ts.idl.enums import Hexapod
from . import constants
from . import enums
from . import structs
from . import utils
from . import mock_controller


# Dict of controller state: CSC state.
# The names match but the numeric values do not.
StateCscState = {
    Hexapod.ControllerState.OFFLINE: salobj.State.OFFLINE,
    Hexapod.ControllerState.STANDBY: salobj.State.STANDBY,
    Hexapod.ControllerState.DISABLED: salobj.State.DISABLED,
    Hexapod.ControllerState.ENABLED: salobj.State.ENABLED,
    Hexapod.ControllerState.FAULT: salobj.State.FAULT,
}

# Dict of CSC summary state: state reported by the low level controller.
# The low level controller's states match CSC summary states in name
# and meaning, but the numeric values differ.
CscStateState = dict((value, key) for key, value in StateCscState.items())


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


class HexapodCsc(salobj.Controller):
    """MT rotator CSC.

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
        self.i = 0

        self._initial_state = salobj.State(initial_state)
        if simulation_mode not in (0, 1):
            raise ValueError(f"simulation_mode = {simulation_mode}; must be 0 or 1")
        self.simulation_mode = simulation_mode
        # Set this True or False for an offset or positionSet command.
        # Reset this to None when offset or positionSet are run
        # or when no longer enabled and stationary.
        # Thus it serves two purposes:
        # * Record a value needed by the move and offset commands.
        # * Record the fact that a position has been specified
        self.synchronized_move = None
        self.server = None
        self.mock_ctrl = None
        structs.Config.FRAME_ID = controller_constants.config_frame_id
        structs.Telemetry.FRAME_ID = controller_constants.telemetry_frame_id
        super().__init__(name="Hexapod", index=index, do_callbacks=True)

        # Dict of enum.CommandCode: Command
        # with constants set to suitable values.
        self.commands = dict()
        for cmd in enums.CommandCode:
            command = hexrotcomm.Command()
            command.cmd = cmd
            command.sync_pattern = controller_constants.sync_pattern
            self.commands[cmd] = command

    @property
    def summary_state(self):
        """Return the current summary state as a salobj.State,
        or OFFLINE if unknown.
        """
        if self.server is None or not self.server.connected:
            return salobj.State.OFFLINE
        return StateCscState.get(int(self.server.telemetry.state), salobj.State.OFFLINE)

    async def start(self):
        await super().start()
        simulating = self.simulation_mode != 0
        host = hexrotcomm.LOCAL_HOST if simulating else None
        self.server = hexrotcomm.CommandTelemetryServer(
            host=host,
            log=self.log,
            ConfigClass=structs.Config,
            TelemetryClass=structs.Telemetry,
            connect_callback=self.connect_callback,
            config_callback=self.config_callback,
            telemetry_callback=self.telemetry_callback,
            use_random_ports=simulating)
        await self.server.start_task
        if simulating:
            initial_ctrl_state = CscStateState[self._initial_state]
            self.mock_ctrl = mock_controller.MockMTHexapodController(
                index=self.salinfo.index,
                log=self.log,
                initial_state=initial_ctrl_state,
                command_port=self.server.command_port,
                telemetry_port=self.server.telemetry_port)
            await self.mock_ctrl.connect_task
        else:
            self.evt_summaryState.set_put(summaryState=salobj.State.OFFLINE)
        # TODO uncomment when inPosition as a boolean field
        # self.evt_inPosition.set_put(inPosition=False, force_output=True)

    async def close_tasks(self):
        if self.mock_ctrl is not None:
            await self.mock_ctrl.close()
        if self.server is not None:
            await self.server.close()

    def assert_summary_state(self, *allowed_states):
        """Assert that the current summary state is as specified.

        Used in do_xxx methods to check that a command is allowed.
        """
        if self.summary_state not in allowed_states:
            raise salobj.ExpectedError(f"Must be in state(s) {allowed_states}, not {self.summary_state}")

    async def run_command(self, cmd, **kwargs):
        command = self.commands[cmd]
        for name, value in kwargs.items():
            if hasattr(command, name):
                setattr(command, name, value)
            else:
                raise ValueError(f"Unknown command argument {name}")
        # Note: increment correctly wraps around
        command.counter += 1
        await self.server.put_command(command)

    # Unsupported standard CSC commnands.
    async def do_abort(self, data):
        raise salobj.ExpectedError("Unsupported command")

    async def do_setSimulationMode(self, data):
        raise salobj.ExpectedError("Unsupported command: "
                                   "simulation mode can only be set when starting the CSC.")

    async def do_setValue(self, data):
        raise salobj.ExpectedError("Unsupported command")

    # Standard CSC commnands.
    async def do_enable(self, data):
        """Execute the enable command."""
        self.assert_summary_state(salobj.State.DISABLED)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.ENABLE)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.ENABLED)

    async def do_disable(self, data):
        """Execute the disable command."""
        self.assert_summary_state(salobj.State.ENABLED)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.DISABLE)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.DISABLED)

    async def do_enterControl(self, data):
        """Execute the enterControl command.
        """
        self.assert_summary_state(salobj.State.OFFLINE)
        if self.server.telemetry.offline_substate != Hexapod.OfflineSubstate.AVAILABLE:
            raise salobj.ExpectedError(
                "Use the engineering interface to put the controller into state OFFLINE/AVAILABLE")
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.ENTER_CONTROL)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.STANDBY)

    async def do_exitControl(self, data):
        self.assert_summary_state(salobj.State.STANDBY)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.EXIT)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.OFFLINE)

    async def do_standby(self, data):
        self.assert_summary_state(salobj.State.DISABLED, salobj.State.FAULT)
        if self.server.telemetry.state == Hexapod.ControllerState.DISABLED:
            await self.run_command(cmd=enums.CommandCode.SET_STATE,
                                   param1=enums.SetStateParam.STANDBY)
        else:
            raise salobj.ExpectedError(
                "Use clearError or the engineering interface to set the state to OFFLINE/PUBLISH_ONLY, "
                "then use the engineering interface to set the state to OFFLINE/AVAILABLE.")
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.STANDBY)

    async def do_start(self, data):
        """Execute the start command.

        Notes
        -----
        This ignores the data, unlike the vendor's CSC code, which writes the
        supplied file name into a file on an nfs-mounted partition.
        I hope we won't need to do that, as it seems complicated.
        """
        if self.summary_state != salobj.State.STANDBY:
            raise salobj.ExpectedError(f"CSC is in {self.summary_state}; must be STANDBY state to start")
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.START)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.DISABLED)

    # Hexapod-specific commands.
    async def do_clearError(self, data):
        """Reset the FAULT state to OFFLINE/PUBLISH_ONLY.

        Unfortunately, after this call you must use the engineering user
        interface to transition the controller from OFFLINE_PUBLISH_ONLY
        to OFFLINE/AVAILABLE before the CSC can control it.
        """
        if self.summary_state != salobj.State.FAULT:
            raise salobj.ExpectedError("Must be in FAULT state.")
        self.assert_enabled_substate(Hexapod.EnabledSubstate.FAULT)
        # Two sequential commands are needed to clear error
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.CLEAR_ERROR)
        await asyncio.sleep(0.9)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.CLEAR_ERROR)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.OFFLINE)

    async def do_configureAcceleration(self, data):
        """Specify the acceleration limit."""
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        utils.check_positive_value(data.accmax, "accmax", constants.MAX_ACCEL_LIMIT,
                                   ExceptionClass=salobj.ExpectedError)
        await self.run_command(cmd=enums.CommandCode.CONFIG_ACCEL,
                               param1=data.accmax)

    async def do_configureElevationRawLUT(self, data):
        """Specify elevation raw lookup table."""
        raise salobj.ExpectedError("Not implemented.")

    async def do_configureAzimuthRawLUT(self, data):
        """Specify azimuth raw lookup table."""
        raise salobj.ExpectedError("Not implemented.")

    async def do_configureTemperatureRawLUT(self, data):
        """Specify temperature raw lookup table."""
        raise salobj.ExpectedError("Not implemented.")

    async def do_configureLimits(self, data):
        """Specify position and rotation limits."""
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        utils.check_positive_value(data.xymax, "xymax", self.xy_max_limit,
                                   ExceptionClass=salobj.ExpectedError)
        utils.check_negative_value(data.zmin, "zmin", self.z_min_limit,
                                   ExceptionClass=salobj.ExpectedError)
        utils.check_positive_value(data.zmax, "zmax", self.z_max_limit,
                                   ExceptionClass=salobj.ExpectedError)
        utils.check_positive_value(data.uvmax, "uvmax", self.uv_max_limit,
                                   ExceptionClass=salobj.ExpectedError)
        utils.check_negative_value(data.wmin, "wmin", self.w_min_limit,
                                   ExceptionClass=salobj.ExpectedError)
        utils.check_positive_value(data.wmax, "wmax", self.w_max_limit,
                                   ExceptionClass=salobj.ExpectedError)
        await self.run_command(cmd=enums.CommandCode.CONFIG_LIMITS,
                               param1=data.xymax,
                               param2=data.zmin,
                               param3=data.zmax,
                               param4=data.uvmax,
                               param5=data.wmin,
                               param6=data.wmax)

    async def do_configureVelocity(self, data):
        """Specify velocity limits."""
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        utils.check_positive_value(data.xymax, "xymax", constants.MAX_LINEAR_VEL_LIMIT,
                                   ExceptionClass=salobj.ExpectedError)
        utils.check_positive_value(data.rxrymax, "rxrymax", constants.MAX_ANGULAR_VEL_LIMIT,
                                   ExceptionClass=salobj.ExpectedError)
        utils.check_positive_value(data.zmax, "zmax", constants.MAX_LINEAR_VEL_LIMIT,
                                   ExceptionClass=salobj.ExpectedError)
        utils.check_positive_value(data.rzmax, "rzmax", constants.MAX_ANGULAR_VEL_LIMIT,
                                   ExceptionClass=salobj.ExpectedError)
        await self.run_command(cmd=enums.CommandCode.CONFIG_VEL,
                               param1=data.xymax,
                               param2=data.rxrymax,
                               param3=data.zmax,
                               param4=data.rzmax)

    async def do_move(self, data):
        """Go to the position specified by the most recent ``positionSet``
        or ``offset`` command.
        """
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        if self.synchronized_move is None:
            raise salobj.ExpectedError("Must specify a position with move or offset.")
        sync_move = self.synchronized_move
        self.synchronized_move = None
        await self.run_command(cmd=enums.CommandCode.SET_ENABLED_SUBSTATE,
                               param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
                               param2=sync_move)

    async def do_moveLUT(self, data):
        """Go to the position specified by the most recent ``positionSet``
        or `offset`` command, with LUT corrections.
        """
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        if self.synchronized_move is None:
            raise salobj.ExpectedError("Must specify a position with move or offset.")
        sync_move = self.synchronized_move
        self.synchronized_move = None
        await self.run_command(cmd=enums.CommandCode.SET_ENABLED_SUBSTATE,
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
        await self.run_command(cmd=enums.CommandCode.POSITION_SET,
                               param1=offset_data.x,
                               param2=offset_data.y,
                               param3=offset_data.z,
                               param4=offset_data.u,
                               param5=offset_data.v,
                               param6=offset_data.w)

    async def do_pivot(self, data):
        """Set the coordinates of the pivot point."""
        self.assert_enabled_substate(Hexapod.EnabledSubstate.STATIONARY)
        await self.run_command(cmd=enums.CommandCode.SET_PIVOTPOINT,
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
        await self.run_command(cmd=enums.CommandCode.POSITION_SET,
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
        await self.run_command(cmd=enums.CommandCode.SET_ENABLED_SUBSTATE,
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

    def assert_enabled_substate(self, substate):
        """Assert the controller is enabled and in the specified substate.
        """
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        if self.server.telemetry.enabled_substate != substate:
            raise salobj.ExpectedError("Low-level controller in substate "
                                       f"{self.server.telemetry.enabled_substate} "
                                       f"instead of {substate!r}")

    def connect_callback(self, server):
        """Called when the server's command or telemetry sockets
        connect or disconnect.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        self.evt_connected.set_put(command=self.server.command_connected,
                                   telemetry=self.server.telemetry_connected)

    def config_callback(self, server):
        """Called when the TCP/IP controller outputs configuration.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        self.evt_settingsApplied.set_put(
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
        self.tel_Actuators.set_put(
            Calibrated=server.telemetry.strut_encoder_microns,
            Raw=server.telemetry.strut_encoder_raw,
        )
        self.tel_Application.set_put(
            Demand=server.telemetry.commanded_pos,
            Position=server.telemetry.measured_pos,
            Error=pos_error,
        )
        self.tel_Electrical.set_put(
            CopleyStatusWordDrive=server.telemetry.status_word,
            CopleyLatchingFaultStatus=server.telemetry.latching_fault_status_register,
        )

        # TODO: the inPosition event has no inPosition field
        # so it cannot be used correctly
        # in_position = all(
        #     status & Hexapod.ApplicationStatus.HEX_MOVE_COMPLETE_MASK
        #     for status in server.telemetry.application_status)
        # self.evt_inPosition.set_put(
        #     inPosition=in_position,
        # )

        self.evt_commandableByDDS.set_put(
            state=bool(server.telemetry.application_status[0] & Hexapod.ApplicationStatus.DDS_COMMAND_SOURCE),
        )

        device_errors = []
        if server.telemetry.application_status[0] & Hexapod.ApplicationStatus.HEX_FOLLOWING_ERROR_MASK:
            device_errors.append("Following Error")
        if server.telemetry.application_status[0] & Hexapod.ApplicationStatus.DRIVE_FAULT:
            device_errors.append("Drive Error")
        if server.telemetry.application_status[0] & Hexapod.ApplicationStatus.EXTEND_LIMIT_SWITCH:
            device_errors.append("Forward Limit Switch")
        if server.telemetry.application_status[0] & Hexapod.ApplicationStatus.RETRACT_LIMIT_SWITCH:
            device_errors.append("Reverse Limit Switch")
        if server.telemetry.application_status[0] & Hexapod.ApplicationStatus.ETHERCAT_PROBLEM:
            device_errors.append("Ethercat Error")
        if server.telemetry.application_status[0] & Hexapod.ApplicationStatus.MOTION_TIMEOUT:
            device_errors.append("Motion timeout")
        if server.telemetry.application_status[0] & Hexapod.ApplicationStatus.SIMULINK_FAULT:
            device_errors.append("Simulink Error")
        device_error_code = ",".join(device_errors)
        self.evt_deviceError.set_put(
            code=device_error_code,
            device="Hexapod",
            severity=1 if device_error_code else 0,
        )

        safety_interlock = server.telemetry.application_status[0] & Hexapod.ApplicationStatus.SAFTEY_INTERLOCK
        self.evt_interlock.set_put(
            detail="Engaged" if safety_interlock else "Disengaged",
        )

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
