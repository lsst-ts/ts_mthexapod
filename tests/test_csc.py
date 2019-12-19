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

import unittest
import time

import asynctest
import numpy as np

from lsst.ts import salobj
from lsst.ts import hexapod
from lsst.ts import hexrotcomm
from lsst.ts.idl.enums import Hexapod

STD_TIMEOUT = 5  # timeout for command ack

index_gen = salobj.index_generator(imin=1, imax=2)


class TestHexapodCsc(hexrotcomm.BaseCscTestCase, asynctest.TestCase):

    def basic_make_csc(self, initial_state, simulation_mode=1):
        return hexapod.HexapodCsc(index=next(index_gen),
                                  initial_state=initial_state,
                                  simulation_mode=simulation_mode)

    async def test_bin_script(self):
        """Test running from the command line script.
        """
        await self.check_bin_script(name="Hexapod",
                                    index=next(index_gen),
                                    exe_name="run_hexapod.py")

    async def test_standard_state_transitions(self):
        enabled_commands = ("configureVelocity", "configureAcceleration",
                            "configureLimits", "configureElevationRawLUT",
                            "configureAzimuthRawLUT", "configureTemperatureRawLUT",
                            "offset", "pivot", "positionSet", "stop")
        await self.check_standard_state_transitions(enabled_commands=enabled_commands)

    async def test_configure_acceleration(self):
        """Test the configureAcceleration command.
        """
        await self.make_csc(initial_state=salobj.State.ENABLED)
        data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
        initial_limit = data.accelerationAccmax
        print("initial_limit=", initial_limit)
        new_limit = initial_limit - 0.1
        await self.remote.cmd_configureAcceleration.set_start(accmax=new_limit, timeout=STD_TIMEOUT)
        data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
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
            """Get position limits from a configuration sample."""
            return (data.limitXYMax, data.limitZMin, data.limitZMax,
                    data.limitUVMax, data.limitWMin, data.limitWMax)

        await self.make_csc(initial_state=salobj.State.ENABLED)
        data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
        initial_limits = get_limits(data)
        new_limits = tuple(lim*0.9 for lim in initial_limits)
        await self.remote.cmd_configureLimits.set_start(xymax=new_limits[0],
                                                        zmin=new_limits[1],
                                                        zmax=new_limits[2],
                                                        uvmax=new_limits[3],
                                                        wmin=new_limits[4],
                                                        wmax=new_limits[5],
                                                        timeout=STD_TIMEOUT)
        data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
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
            make_modified_limits(0, self.csc.xy_max_limit + 0.001),
            make_modified_limits(1, self.csc.z_min_limit - 0.001),
            make_modified_limits(2, self.csc.z_max_limit + 0.001),
            make_modified_limits(3, 0),
            make_modified_limits(3, self.csc.uv_max_limit + 0.001),
            make_modified_limits(4, self.csc.w_min_limit - 0.001),
            make_modified_limits(5, self.csc.w_max_limit + 0.001),
        ):
            with self.subTest(bad_limits=bad_limits):
                print(f"bad_limits={bad_limits}")
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
            """Get the velocity limits from a configuration sample."""
            return (data.velocityXYMax, data.velocityRxRyMax, data.velocityZMax, data.velocityRzMax)

        await self.make_csc(initial_state=salobj.State.ENABLED)
        data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
        initial_vel_limits = get_velocity_limits(data)
        new_vel_limits = tuple(lim - 0.01 for lim in initial_vel_limits)
        await self.remote.cmd_configureVelocity.set_start(xymax=new_vel_limits[0],
                                                          rxrymax=new_vel_limits[1],
                                                          zmax=new_vel_limits[2],
                                                          rzmax=new_vel_limits[3],
                                                          timeout=STD_TIMEOUT)
        data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
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

    async def test_move(self):
        await self.check_move(destination=(300, 400, -300, 0.01, 0.02, -0.015),
                              est_move_duration=1,
                              elaztemp=None)

    async def test_move_lut(self):
        await self.check_move(destination=(500, -300, 200, 0.03, -0.02, 0.03),
                              est_move_duration=1,
                              elaztemp=(32, 44, 15))

    async def check_next_position(self, desired_position):
        """Wait for Application telemetry and check the position.

        Parameters
        ----------
        desired_position : `List` [`float`]
            Desired position: x, y, z (µm), rotx, roty, rotz (deg)
        """
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        np.testing.assert_allclose(data.Demand, desired_position)
        # Add slop to accommodate jitter added by the mock controller.
        np.testing.assert_allclose(data.Position[:3], desired_position[:3], atol=1)
        np.testing.assert_allclose(data.Position[3:], desired_position[3:], atol=1e-5)

    async def check_move(self, destination, est_move_duration,
                         elaztemp):
        """Test point to point motion using the positionSet and move
        or moveLUT commands.

        Parameters
        ----------
        destination : `List` [`float`]
            Destination x, y, z (µm), rotx, roty, rotz (deg)
        est_move_duration : `float`
            Estimated move duration (sec)
        elaztemp : `List` [`float`] or `None`
            Elevation, azimuth (deg) and temperature (C)
            for the moveLUT command.
            If None then call move instead of moveLUT.
        """
        self.assertEqual(len(destination), 6)
        await self.make_csc(initial_state=salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Hexapod.ControllerState.ENABLED,
                                                enabledSubstate=Hexapod.EnabledSubstate.STATIONARY)
        await self.check_next_position(desired_position=(0,)*6)
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
        await self.check_next_position(desired_position=destination)

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
        await self.check_next_position(desired_position=desired_destination)

    async def test_stop_move(self):
        """Test stopping a point to point move.
        """
        # Command a move that moves all actuators equally
        destination = (0, 0, 1000, 0, 0, 0)
        await self.make_csc(initial_state=salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Hexapod.ControllerState.ENABLED,
                                                enabledSubstate=Hexapod.EnabledSubstate.STATIONARY)
        await self.check_next_position(desired_position=(0,)*6)
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
        await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        # The Mock controller does not compute position as a function
        # of actuator lengths, so test that motion halted by examining
        # the actuators.
        stopped_lengths = [actuator.end_pos for actuator in self.csc.mock_ctrl.hexapod.actuators]
        for i in range(6):
            self.assertNotAlmostEqual(cmd_lengths[i], stopped_lengths[i])

    async def test_pivot(self):
        """Test the pivot command.
        """
        await self.make_csc(initial_state=salobj.State.ENABLED)


if __name__ == "__main__":
    unittest.main()
