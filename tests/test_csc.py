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
        enabled_commands = ("offset", "pivot", "positionSet", "stop")
        await self.check_standard_state_transitions(enabled_commands=enabled_commands)

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

    async def check_move(self, destination, est_move_duration, elaztemp):
        """Test point to point motion using the positionSet and move
        or moveLUT commands.

        Assumes that the CSC starts with inPosition=False.

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
        data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT)
        self.assertFalse(data.inPosition)
        data = await self.remote.evt_actuatorInPosition.next(flush=False, timeout=STD_TIMEOUT)
        self.assertEqual(tuple(data.inPosition), (False,)*6)
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
        data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT)
        self.assertTrue(data.inPosition)
        # Check that actuatorInPosition returns all in position;
        # this should occur within 6 events (fewer if several actuators
        # finish their move at the same time).
        for i in range(6):
            try:
                data = await self.remote.evt_actuatorInPosition.next(flush=False, timeout=STD_TIMEOUT)
                self.assertIn(True, data.inPosition)
                if False not in data.inPosition:
                    break
            except asyncio.TimeoutError:
                self.fail("actuatorInPosition timed out before all True")
        else:
            self.fail("actuatorInPosition output 6 times, but not all True")

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
