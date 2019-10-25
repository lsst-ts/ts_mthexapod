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

import asyncio
import logging
import unittest
import shutil
import time

import asynctest
import numpy as np

from lsst.ts import salobj
from lsst.ts import hexapod
from lsst.ts.idl.enums import Hexapod

STD_TIMEOUT = 5  # timeout for command ack
LONG_TIMEOUT = 30  # timeout for CSCs to start
NODATA_TIMEOUT = 0.1  # timeout for when we expect no new data


class TestHexapodCsc(asynctest.TestCase):
    async def setUp(self):
        salobj.test_utils.set_random_lsst_dds_domain()
        self.csc = None  # set by make_csc
        self.remote = None

    async def tearDown(self):
        close_tasks = []
        if self.csc is not None:
            close_tasks.append(self.csc.close())
        if self.remote is not None:
            close_tasks.append(self.remote.close())
        if close_tasks:
            await asyncio.wait_for(asyncio.gather(*close_tasks), timeout=STD_TIMEOUT)

    async def make_csc(self, index, initial_state, simulation_mode=1,
                       wait_connected=True, log_level=logging.INFO):
        """Create a HexapodCsc and remote and wait for them to start.

        Parameters
        ----------
        initial_state : `lsst.ts.salobj.State` or `int` (optional)
            The initial state of the CSC. Ignored except in simulation mode
            because in normal operation the initial state is the current state
            of the controller.
        simulation_mode : `int` (optional)
            Simulation mode.
        wait_connected : `bool`
            If True then wait for the controller to connect.
        """
        self.csc = hexapod.HexapodCsc(index=index,
                                      initial_state=initial_state,
                                      simulation_mode=simulation_mode)
        if len(self.csc.log.handlers) < 2:
            self.csc.log.addHandler(logging.StreamHandler())
            self.csc.log.setLevel(log_level)
        self.remote = salobj.Remote(domain=self.csc.domain, name="Hexapod", index=index)

        await asyncio.wait_for(asyncio.gather(self.csc.start_task, self.remote.start_task),
                               timeout=LONG_TIMEOUT)
        self.csc.mock_ctrl.log.setLevel(log_level)
        if wait_connected:
            for i in range(3):
                data = await self.remote.evt_connected.next(flush=False, timeout=STD_TIMEOUT)
                if data.command and data.telemetry:
                    print("Connected")
                    break

    async def assert_next_summary_state(self, state, timeout=STD_TIMEOUT):
        data = await self.remote.evt_summaryState.next(flush=False, timeout=timeout)
        self.assertEqual(data.summaryState, state)

    async def assert_next_controller_state(self, controllerState=None,
                                           offlineSubstate=None,
                                           enabledSubstate=None,
                                           timeout=STD_TIMEOUT):
        """Wait for and check the next controllerState event.

        Parameters
        ----------
        controllerState : `lsst.ts.idl.enums.Hexapod.ControllerState`
            Desired controller state.
        offlineSubstate : `lsst.ts.idl.enums.Hexapod.OfflineSubstate`
            Desired offline substate.
        enabledSubstate : `lsst.ts.idl.enums.Hexapod.EnabledSubstate`
            Desired enabled substate.
        timeout : `float`
            Time limit for getting a controllerState event (sec).
        """
        data = await self.remote.evt_controllerState.next(flush=False, timeout=timeout)
        if controllerState is not None:
            self.assertEqual(data.controllerState, controllerState)
        if offlineSubstate is not None:
            self.assertEqual(data.offlineSubstate, offlineSubstate)
        if enabledSubstate is not None:
            self.assertEqual(data.controllerState, controllerState)

    async def test_bin_script(self):
        """Test running from the command line script.
        """
        exe_name = "run_hexapod.py"
        exe_path = shutil.which(exe_name)
        if exe_path is None:
            self.fail(f"Could not find bin script {exe_name}; did you setup or install this package?")

        index = 2
        process = await asyncio.create_subprocess_exec(exe_name, str(index), "--simulate")
        try:
            async with salobj.Domain() as domain:
                remote = salobj.Remote(domain=domain, name="Hexapod")
                summaryState_data = await remote.evt_summaryState.next(flush=False, timeout=60)
                self.assertEqual(summaryState_data.summaryState, salobj.State.OFFLINE)

        finally:
            process.terminate()

    async def test_configure_acceleration(self):
        """Test the configureAcceleration command.
        """
        await self.make_csc(index=1, initial_state=salobj.State.ENABLED)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        initial_limit = data.accelerationAccmax
        print("initial_limit=", initial_limit)
        new_limit = initial_limit - 0.1
        await self.remote.cmd_configureAcceleration.set_start(accmax=new_limit, timeout=STD_TIMEOUT)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        self.assertAlmostEqual(data.accelerationAccmax, new_limit)

        for bad_accmax in (-1, 0, hexapod.MAX_ACCEL_LIMIT + 0.001):
            with self.subTest(bad_accmax=bad_accmax):
                with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                    await self.remote.cmd_configureAcceleration.set_start(accmax=bad_accmax,
                                                                          timeout=STD_TIMEOUT)

    async def test_configure_limits(self):
        """Test the configureLimits command.
        """
        def get_limits(data):
            """Get position limits from a settingsApplied sample."""
            return (data.limitXYMax, data.limitZMin, data.limitZMax,
                    data.limitUVMax, data.limitWMin, data.limitWMax)

        index = 2
        xy_max_limit = hexapod.XY_MAX_LIMIT[index-1]
        z_min_limit = hexapod.Z_MIN_LIMIT[index-1]
        z_max_limit = hexapod.Z_MAX_LIMIT[index-1]
        uv_max_limit = hexapod.UV_MAX_LIMIT[index-1]
        w_min_limit = hexapod.W_MIN_LIMIT[index-1]
        w_max_limit = hexapod.W_MAX_LIMIT[index-1]

        await self.make_csc(index=index, initial_state=salobj.State.ENABLED)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        initial_limits = get_limits(data)
        new_limits = tuple(lim*0.9 for lim in initial_limits)
        await self.remote.cmd_configureLimits.set_start(xymax=new_limits[0],
                                                        zmin=new_limits[1],
                                                        zmax=new_limits[2],
                                                        uvmax=new_limits[3],
                                                        wmin=new_limits[4],
                                                        wmax=new_limits[5],
                                                        timeout=STD_TIMEOUT)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        reported_limits = get_limits(data)
        for i in range(4):
            self.assertAlmostEqual(new_limits[i], reported_limits[i])

        def make_modified_limits(i, value):
            """Make modified limits from initial_limits, setting one element
            to a new value.
            """
            bad_limits = list(initial_limits)
            bad_limits[i] = value
            return tuple(bad_limits)

        for bad_limits in (
            make_modified_limits(0, 0),
            make_modified_limits(0, xy_max_limit + 0.001),
            make_modified_limits(1, z_min_limit - 0.001),
            make_modified_limits(2, z_max_limit + 0.001),
            make_modified_limits(3, 0),
            make_modified_limits(3, uv_max_limit + 0.001),
            make_modified_limits(4, w_min_limit - 0.001),
            make_modified_limits(5, w_max_limit + 0.001),
        ):
            with self.subTest(bad_limits=bad_limits):
                with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                    await self.remote.cmd_configureLimits.set_start(xymax=bad_limits[0],
                                                                    zmin=bad_limits[1],
                                                                    zmax=bad_limits[2],
                                                                    uvmax=bad_limits[3],
                                                                    wmin=bad_limits[4],
                                                                    wmax=bad_limits[5],
                                                                    timeout=STD_TIMEOUT)

    async def test_configure_velocity(self):
        """Test the configureVelocity command.
        """
        def get_velocity_limits(data):
            """Get the velocity limits from a settingsApplied sample."""
            return (data.velocityXYMax, data.velocityRxRyMax, data.velocityZMax, data.velocityRzMax)

        await self.make_csc(index=2, initial_state=salobj.State.ENABLED)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        initial_vel_limits = get_velocity_limits(data)
        new_vel_limits = tuple(lim - 0.01 for lim in initial_vel_limits)
        await self.remote.cmd_configureVelocity.set_start(xymax=new_vel_limits[0],
                                                          rxrymax=new_vel_limits[1],
                                                          zmax=new_vel_limits[2],
                                                          rzmax=new_vel_limits[3],
                                                          timeout=STD_TIMEOUT)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        reported_limits = get_velocity_limits(data)
        for i in range(4):
            self.assertAlmostEqual(new_vel_limits[i], reported_limits[i])

        bad_linear_vel_limit = hexapod.MAX_LINEAR_VEL_LIMIT + 0.001
        bad_angular_vel_limit = hexapod.MAX_ANGULAR_VEL_LIMIT + 0.001
        for bad_vel_limits in (
            (0, initial_vel_limits[1], initial_vel_limits[2], initial_vel_limits[3]),
            (bad_linear_vel_limit, initial_vel_limits[1], initial_vel_limits[2], initial_vel_limits[3]),
            (initial_vel_limits[0], 0, initial_vel_limits[2], initial_vel_limits[3]),
            (initial_vel_limits[0], bad_angular_vel_limit, initial_vel_limits[2], initial_vel_limits[3]),
            (initial_vel_limits[0], initial_vel_limits[1], 0, initial_vel_limits[3]),
            (initial_vel_limits[0], initial_vel_limits[1], bad_linear_vel_limit, initial_vel_limits[3]),
            (initial_vel_limits[0], initial_vel_limits[1], initial_vel_limits[2], 0),
            (initial_vel_limits[0], initial_vel_limits[1], initial_vel_limits[2], bad_angular_vel_limit),
            (0, 0, 0, 0),
            (bad_linear_vel_limit, bad_angular_vel_limit, bad_linear_vel_limit, bad_angular_vel_limit),
        ):
            with self.subTest(bad_vel_limits=bad_vel_limits):
                with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                    await self.remote.cmd_configureVelocity.set_start(xymax=bad_vel_limits[0],
                                                                      rxrymax=bad_vel_limits[1],
                                                                      zmax=bad_vel_limits[2],
                                                                      rzmax=bad_vel_limits[3],
                                                                      timeout=STD_TIMEOUT)

    async def test_standard_state_transitions(self):
        """Test standard CSC state transitions.

        The initial state is STANDBY.
        The standard commands and associated state transitions are:

        * start: STANDBY to DISABLED
        * enable: DISABLED to ENABLED

        * disable: ENABLED to DISABLED
        * standby: DISABLED or FAULT to STANDBY
        * exitControl: STANDBY to OFFLINE (quit)
        """
        await self.make_csc(index=1, initial_state=salobj.State.OFFLINE)
        print("CSC running")

        await self.assert_next_summary_state(salobj.State.OFFLINE)
        await self.check_bad_commands(good_commands=("enterControl", "setLogLevel"))

        # send enterControl; new state is STANDBY
        await self.remote.cmd_enterControl.start(timeout=STD_TIMEOUT)
        # Check CSC summary state directly to make sure it has changed
        # before the command is acknowledged as done.
        self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)
        await self.assert_next_summary_state(salobj.State.STANDBY)
        await self.check_bad_commands(good_commands=("start", "exitControl", "setLogLevel"))

        # send start; new state is DISABLED
        await self.remote.cmd_start.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.DISABLED)
        await self.assert_next_summary_state(salobj.State.DISABLED)
        await self.check_bad_commands(good_commands=("enable", "standby", "setLogLevel"))

        # send enable; new state is ENABLED
        await self.remote.cmd_enable.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.ENABLED)
        await self.assert_next_summary_state(salobj.State.ENABLED)
        await self.check_bad_commands(good_commands=("disable", "setLogLevel",
                                                     "configureVelocity", "configureAcceleration",
                                                     "configureLimits", "configureElevationRawLUT",
                                                     "configureAzimuthRawLUT", "configureTemperatureRawLUT",
                                                     "offset", "pivot", "positionSet", "stop"))

        # send disable; new state is DISABLED
        await self.remote.cmd_disable.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.DISABLED)
        await self.assert_next_summary_state(salobj.State.DISABLED)

        # send standby; new state is STANDBY
        await self.remote.cmd_standby.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)
        await self.assert_next_summary_state(salobj.State.STANDBY)

        # send exitControl; new state is OFFLINE
        await self.remote.cmd_exitControl.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.OFFLINE)
        await self.assert_next_summary_state(salobj.State.OFFLINE)

    async def check_bad_commands(self, bad_commands=None, good_commands=None):
        """Check that bad commands fail.

        Parameters
        ----------
        bad_commands : `List`[`str`] or `None` (optional)
            Names of bad commands to try, or None for all commands.
        good_commands : `List`[`str`] or `None` (optional)
            Names of good commands to skip, or None to skip none.

        Notes
        -----
        If a command appears in both lists it is considered a good command.
        """
        if bad_commands is None:
            bad_commands = self.remote.salinfo.command_names
        if good_commands is None:
            good_commands = ()
        commands = self.remote.salinfo.command_names
        for command in commands:
            print(f"Try bad_command={command}")
            if command in good_commands:
                continue
            with self.subTest(command=command):
                cmd_attr = getattr(self.remote, f"cmd_{command}")
                with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                    await cmd_attr.start(timeout=STD_TIMEOUT)

    async def test_initial_state_offline(self):
        await self.check_initial_state(salobj.State.OFFLINE)

    async def test_initial_state_standby(self):
        await self.check_initial_state(salobj.State.STANDBY)

    async def test_initial_state_disabled(self):
        await self.check_initial_state(salobj.State.DISABLED)

    async def test_initial_state_enabled(self):
        await self.check_initial_state(salobj.State.ENABLED)

    async def check_initial_state(self, initial_state):
        await self.make_csc(index=2, initial_state=initial_state)
        await self.assert_next_summary_state(initial_state)

    async def test_move(self):
        await self.check_move(destination=(300, 400, -300, 0.01, 0.02, -0.015),
                              est_move_duration=1,
                              elaztemp=None)

    async def test_move_lut(self):
        await self.check_move(destination=(500, -300, 200, 0.03, -0.02, 0.03),
                              est_move_duration=1,
                              elaztemp=(32, 44, 15))

    async def check_move(self, destination, est_move_duration,
                         elaztemp):
        """Test point to point motion using the positionSet and move
        or moveLUT commands.

        Parameters
        ----------
        destination : `List` [`float`]
            Destination x, y, z, u, v, w
        est_move_duration : `float`
            Estimated move duration (sec)
        elaztemp : `List` [`float`] or `None`
            Elevation, azimuth and temperature (C) for the moveLUT command.
            If None then call move instead of moveLUT.
        """
        self.assertEqual(len(destination), 6)
        await self.make_csc(index=1, initial_state=salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Hexapod.ControllerState.ENABLED,
                                                enabledSubstate=Hexapod.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        np.testing.assert_allclose(data.Demand, (0,)*6)
        np.testing.assert_allclose(data.Position, (0,)*6)
        await self.remote.cmd_positionSet.set_start(x=destination[0],
                                                    y=destination[1],
                                                    z=destination[2],
                                                    u=destination[3],
                                                    v=destination[4],
                                                    w=destination[5],
                                                    sync=1,  # doesn't matter
                                                    timeout=STD_TIMEOUT)
        t0 = time.time()
        if elaztemp is None:
            await self.remote.cmd_move.start(timeout=STD_TIMEOUT)
        else:
            await self.remote.cmd_moveLUT.set_start(elev=elaztemp[0],
                                                    az=elaztemp[1],
                                                    temp=elaztemp[2],
                                                    timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(
            controllerState=Hexapod.ControllerState.ENABLED,
            enabledSubstate=Hexapod.EnabledSubstate.MOVING_POINT_TO_POINT)
        await self.assert_next_controller_state(
            controllerState=Hexapod.ControllerState.ENABLED,
            enabledSubstate=Hexapod.EnabledSubstate.STATIONARY,
            timeout=STD_TIMEOUT+est_move_duration)
        print(f"Move duration: {time.time() - t0:0.2f} seconds")
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        np.testing.assert_allclose(data.Demand, destination)
        np.testing.assert_allclose(data.Position, destination)

    async def test_offset(self):
        first_destination = (100, 200, -300, 0.01, 0.02, -0.015)
        await self.check_move(destination=first_destination,
                              est_move_duration=1,
                              elaztemp=None)
        offset = (50, -100, 135, 0.005, -0.005, 0.01)
        desired_destination = np.add(first_destination, offset)
        await self.remote.cmd_offset.set_start(x=offset[0],
                                               y=offset[1],
                                               z=offset[2],
                                               u=offset[3],
                                               v=offset[4],
                                               w=offset[5],
                                               sync=1,  # doesn't matter
                                               timeout=STD_TIMEOUT)
        await self.remote.cmd_move.start(timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(
            controllerState=Hexapod.ControllerState.ENABLED,
            enabledSubstate=Hexapod.EnabledSubstate.MOVING_POINT_TO_POINT)
        await self.assert_next_controller_state(
            controllerState=Hexapod.ControllerState.ENABLED,
            enabledSubstate=Hexapod.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        np.testing.assert_allclose(data.Demand, desired_destination)
        np.testing.assert_allclose(data.Position, desired_destination)

    async def test_stop_move(self):
        """Test stopping a point to point move.
        """
        # Command a move that moves all actuators equally
        destination = (0, 0, 1000, 0, 0, 0)
        await self.make_csc(index=1, initial_state=salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Hexapod.ControllerState.ENABLED,
                                                enabledSubstate=Hexapod.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        np.testing.assert_allclose(data.Demand, (0,)*6)
        np.testing.assert_allclose(data.Position, (0,)*6)
        await self.remote.cmd_positionSet.set_start(x=destination[0],
                                                    y=destination[1],
                                                    z=destination[2],
                                                    u=destination[3],
                                                    v=destination[4],
                                                    w=destination[5],
                                                    sync=1,  # doesn't matter
                                                    timeout=STD_TIMEOUT)
        await self.remote.cmd_move.start(timeout=STD_TIMEOUT)
        cmd_lengths = [actuator.end_pos for actuator in self.csc.mock_ctrl.hexapod.actuators]
        await self.assert_next_controller_state(
            controllerState=Hexapod.ControllerState.ENABLED,
            enabledSubstate=Hexapod.EnabledSubstate.MOVING_POINT_TO_POINT)
        await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(controllerState=Hexapod.ControllerState.ENABLED,
                                                enabledSubstate=Hexapod.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        # The Mock controller does not compute position as a function
        # of actuator lengths, so test that motion halted by examining
        # the actuators.
        stopped_lengths = [actuator.end_pos for actuator in self.csc.mock_ctrl.hexapod.actuators]
        for i in range(6):
            self.assertNotAlmostEqual(cmd_lengths[i], stopped_lengths[i])

    async def test_pivot(self):
        """Test the pivot command.
        """
        await self.make_csc(index=1, initial_state=salobj.State.ENABLED)

    def test_bad_simulation_modes(self):
        """Test simulation_mode argument of TestCsc constructor.

        The only allowed values are 0 and 1
        """
        for simulation_mode in (-1, 2, 3):
            with self.assertRaises(ValueError):
                hexapod.HexapodCsc(index=2, simulation_mode=simulation_mode)

    async def test_non_simulation_mode(self):
        ignored_initial_state = salobj.State.FAULT
        async with hexapod.HexapodCsc(index=1, simulation_mode=0, initial_state=ignored_initial_state) as csc:
            self.assertIsNone(csc.mock_ctrl)
            await asyncio.sleep(0.2)
            self.assertFalse(csc.server.command_connected)
            self.assertFalse(csc.server.telemetry_connected)
            self.assertFalse(csc.server.connected)


if __name__ == "__main__":
    unittest.main()
