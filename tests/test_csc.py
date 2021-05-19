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

import asyncio
import contextlib
import copy
import dataclasses
import logging
import pathlib
import unittest
import time

import numpy as np

from lsst.ts import salobj
from lsst.ts import mthexapod
from lsst.ts import hexrotcomm
from lsst.ts.idl.enums.MTHexapod import ControllerState, EnabledSubstate

STD_TIMEOUT = 10  # timeout for command ack
EPSILON = 1e-10  # approx allowed error between float and double

ZERO_POSITION = mthexapod.Position(0, 0, 0, 0, 0, 0)

logging.basicConfig()

index_gen = salobj.index_generator(imin=1, imax=2)

local_config_dir = pathlib.Path(__file__).parent / "data" / "config"


class TestHexapodCsc(hexrotcomm.BaseCscTestCase, unittest.IsolatedAsyncioTestCase):
    def basic_make_csc(
        self, initial_state, config_dir=None, settings_to_apply="", simulation_mode=1
    ):
        return mthexapod.HexapodCsc(
            index=next(index_gen),
            initial_state=initial_state,
            settings_to_apply=settings_to_apply,
            simulation_mode=simulation_mode,
            config_dir=config_dir,
        )

    @contextlib.asynccontextmanager
    async def make_csc(
        self,
        initial_state=salobj.State.STANDBY,
        config_dir=None,
        settings_to_apply="",
        simulation_mode=1,
        log_level=None,
    ):
        # TODO DM-28005: add a controller for the temperature
        async with super().make_csc(
            initial_state=initial_state,
            config_dir=config_dir,
            settings_to_apply=settings_to_apply,
            simulation_mode=simulation_mode,
            log_level=log_level,
        ), salobj.Controller(
            name="MTMount",
        ) as self.mtmount_controller, salobj.Controller(
            name="MTRotator",
        ) as self.mtrotator_controller:
            # self.mtmount_controller = mtmount_controller
            # self.mtrotator_controller = mtrotator_controller
            yield

    async def assert_next_application(self, desired_position):
        """Wait for and check the next application telemetry.

        Test desired_position against the ``demand`` field
        (for which tight matching is expected)
        and the ``position`` field (with more slop to accommodate jitter
        added by the mock controller).

        Try three samples before giving up, to avoid a race condition
        (two may suffice, but allowing a third is inexpensive).

        Parameters
        ----------
        desired_position : `Position`
            Desired position
        """
        for i in range(3):
            desired_pos_tuple = dataclasses.astuple(desired_position)
            data = await self.remote.tel_application.next(
                flush=True, timeout=STD_TIMEOUT
            )
            if np.allclose(data.demand, desired_pos_tuple):
                if i > 0:
                    print(f"assert_next_application required {i + 1} iterations")
                break
        else:
            self.fail(
                f"desired_position={desired_pos_tuple} - "
                f"demand={data.demand} too large: "
                f"{np.subtract(desired_pos_tuple, data.demand)}"
            )

        # Add slop to accommodate jitter added by the mock controller.
        self.assert_positions_close(
            data.position, desired_position, pos_atol=1, ang_atol=1e-5
        )

    async def assert_next_compensation(self, compensation_inputs, offset):
        """Wait for and check the next compensation event.

        Parameters
        ----------
        compensation_inputs : `CompensationInputs`
            Compensation inputs.
        offset : `Position`
            Expected compensation offset.
        """
        data = await self.assert_next_sample(self.remote.evt_compensationOffset)
        self.assertAlmostEqual(data.elevation, compensation_inputs.elevation)
        self.assertAlmostEqual(data.azimuth, compensation_inputs.azimuth)
        self.assertAlmostEqual(data.rotation, compensation_inputs.rotation)
        # TODO DM-28005: check specified temperature
        self.assertAlmostEqual(data.temperature, 0)

        for i, name in enumerate(("x", "y", "z", "u", "v", "w")):
            self.assertAlmostEqual(getattr(data, name), getattr(offset, name))

    async def assert_next_compensated_position(self, position):
        """Wait for and check the next uncompensatedPosition event.

        Parameters
        ----------
        position : `Position`
            Expected compensated position.
        """
        data = await self.assert_next_sample(self.remote.evt_compensatedPosition)
        read_position = mthexapod.Position.from_struct(data)

        self.assert_dataclasses_almost_equal(position, read_position)

    async def assert_next_uncompensated_position(self, position):
        """Wait for and check the next uncompensatedPosition event.

        Parameters
        ----------
        position : `Position`
            Expected uncompensated position.
        """
        data = await self.assert_next_sample(self.remote.evt_uncompensatedPosition)
        read_position = mthexapod.Position.from_struct(data)

        self.assert_dataclasses_almost_equal(position, read_position)

    def assert_positions_close(
        self, position1, position2, pos_atol=1e-2, ang_atol=1e-7
    ):
        """Assert that two positions are close.

        Parameters
        ----------
        position1 : `Position` or `List` [`float`]
            First position to check.
        position1 : `Position` or `List` [`float`]
            Second position to check.
        pos_atol : `float`, optional
            Absolute tolerance for x, y, z (um)
        ang_atol : `float`, optional
            Absolute tolerance for u, v, w (deg)
        """
        if isinstance(position1, mthexapod.Position):
            position1_tuple = dataclasses.astuple(position1)
        else:
            position1_tuple = position1
        if isinstance(position2, mthexapod.Position):
            position2_tuple = dataclasses.astuple(position2)
        else:
            position2_tuple = position2
        np.testing.assert_allclose(
            position1_tuple[:3], position2_tuple[:3], atol=pos_atol
        )
        np.testing.assert_allclose(
            position1_tuple[3:], position2_tuple[3:], atol=ang_atol
        )

    async def check_move(
        self,
        uncompensated_position,
        est_move_duration,
        speed_factor=2,
    ):
        """Test point to point motion using the move command.

        Initially expects to see the following events:

        * controllerState with state ENABLED/STATIONARY
        * inPosition event with inPosition=False

        Checks inPosition and uncompensatedPosition events.
        Does not check compensatedPosition or application telemetry,
        since those values depend on whether the move is compensated.

        Parameters
        ----------
        uncompensated_position : `Position`
            Uncompensated position.
        est_move_duration : `float`
            Rough estimate of move duration (sec); used for timeouts,
            so it is much better to make it too big than too small.
        speed_factor : `float`
            Amount by which to scale actuator speeds. Intended to allow
            speeding up moves so tests run more quickly.
        """
        self.set_speed_factor(speed_factor)
        await self.assert_next_sample(
            topic=self.remote.evt_controllerState,
            controllerState=ControllerState.ENABLED,
            enabledSubstate=EnabledSubstate.STATIONARY,
        )
        await self.basic_check_move(
            uncompensated_position=uncompensated_position,
            est_move_duration=est_move_duration,
        )

    async def basic_check_move(self, uncompensated_position, est_move_duration):
        """Test point to point motion using the move command.

        Initially expects to see the following events:

        * inPosition event with inPosition=False

        Checks inPosition and target events.
        Does not check application telemetry, since those values
        depend on whether the move is compensated.

        Parameters
        ----------
        uncompensated_position : `Position`
            Desired position.
        est_move_duration : `float`
            Rough estimate of move duration (sec); used for timeouts,
            so it is much better to make it too big than too small.
        """
        t0 = time.time()
        await self.remote.cmd_move.set_start(
            **vars(uncompensated_position), timeout=STD_TIMEOUT
        )
        await self.assert_next_sample(
            topic=self.remote.evt_controllerState,
            controllerState=ControllerState.ENABLED,
            enabledSubstate=EnabledSubstate.MOVING_POINT_TO_POINT,
        )
        try:
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
                timeout=STD_TIMEOUT + est_move_duration,
            )
        except asyncio.TimeoutError:
            self.fail(
                f"Move timed out in {STD_TIMEOUT+est_move_duration} seconds; "
                f"remaining move time {self.csc.mock_ctrl.hexapod.remaining_time():0.2f}"
            )

        await self.assert_next_sample(self.remote.evt_inPosition, inPosition=False)
        await self.assert_next_sample(self.remote.evt_inPosition, inPosition=True)

        print(f"Move duration: {time.time() - t0:0.2f} seconds")
        await self.assert_next_uncompensated_position(position=uncompensated_position)

    async def check_compensation(
        self, uncompensated_position, compensation_inputs, update_inputs
    ):
        """Check compensation for a given set of of compensation inputs.

        Parameters
        ----------
        uncompensated_position : `list` [`float`]
            Target x, y, z (um), u, v, w (deg).
        compensation_inputs : `CompensationInputs`
            Compensation inputs
        update_inputs : `bool`
            Send these compensation_inputs to the CSC?
            Set False if this has already been done.
        """
        if update_inputs:
            await self.set_compensation_inputs(**vars(compensation_inputs))

        # Wait for the compensation event with the correct inputs
        # (we have to set inputs one at a time, so we may see
        # a compensation event with incorrect inputs)
        data = None
        while (
            data is None
            or abs(data.elevation - compensation_inputs.elevation) > EPSILON
            or abs(data.azimuth - compensation_inputs.azimuth) > EPSILON
            or abs(data.rotation - compensation_inputs.rotation) > EPSILON
            # TODO DM-28005: check specified temperature
        ):
            data = await self.remote.evt_compensationOffset.next(
                flush=False, timeout=STD_TIMEOUT
            )

        # TODO DM-28005: use compensation_inputs directly
        hacked_compensation_inputs = copy.copy(compensation_inputs)
        hacked_compensation_inputs.temperature = 0
        compensation_offset = self.csc.compensation.get_offset(
            hacked_compensation_inputs
        )
        if update_inputs:
            # We set the compensation inputs, so we know compensation
            # should be nonzero in at least one axis.
            nonzero = [
                offset != 0 for offset in dataclasses.astuple(compensation_offset)
            ]
            self.assertIn(True, nonzero)

        reported_compensation_offset = mthexapod.Position.from_struct(data)
        self.assert_dataclasses_almost_equal(
            reported_compensation_offset, compensation_offset
        )

        desired_compensated_position = uncompensated_position + compensation_offset
        await self.assert_next_application(
            desired_position=desired_compensated_position
        )

    async def check_offset(
        self, first_uncompensated_position, offset, est_move_duration
    ):
        """Check an offset.

        Parameters
        ----------
        first_uncompensated_position : `Position`
            Initial position (uncompensated)
        offset : `Position`
            Desired offset (uncompensated)
        est_move_duration : `float`
            Rough estimate of move duration (sec); used for timeouts,
            so it is much better to make it too big than too small.
        """
        await self.check_move(
            uncompensated_position=first_uncompensated_position,
            est_move_duration=1,
        )
        await self.remote.cmd_offset.set_start(**vars(offset), timeout=STD_TIMEOUT)
        await self.assert_next_sample(
            topic=self.remote.evt_controllerState,
            controllerState=ControllerState.ENABLED,
            enabledSubstate=EnabledSubstate.MOVING_POINT_TO_POINT,
        )
        await self.assert_next_sample(
            topic=self.remote.evt_controllerState,
            controllerState=ControllerState.ENABLED,
            enabledSubstate=EnabledSubstate.STATIONARY,
        )

    def limits_to_min_position(self, limits):
        """Return the position corresponding to the minimum position limit.

        Parameters
        ----------
        limits : `PositionLimits`
            Position limits

        Returns
        -------
        position : `Position`
            Position at the lower limit.
        """
        return mthexapod.Position(
            x=-limits.maxXY,
            y=-limits.maxXY,
            z=limits.minZ,
            u=-limits.maxUV,
            v=-limits.maxUV,
            w=limits.minW,
        )

    def limits_to_max_position(self, limits):
        """Return the position corresponding to the maximum position limit.

        Parameters
        ----------
        limits : `PositionLimits`
            Position limits

        Returns
        -------
        position : `Position`
            Position at the upper limit.
        """
        return mthexapod.Position(
            x=limits.maxXY,
            y=limits.maxXY,
            z=limits.maxZ,
            u=limits.maxUV,
            v=limits.maxUV,
            w=limits.maxW,
        )

    async def set_compensation_inputs(
        self, elevation, azimuth, rotation, temperature, timeout=STD_TIMEOUT
    ):
        """Set one or more of the compensation input topics read by the CSC.

        This accepts individual parameters rather than a `CompensationInputs`
        in order to allow specifying partial inputs.
        To call with a `CompensationInputs`, call with::

            **vars(compensation_inputs)

        Parameters
        ----------
        elevation : `float` or `None`
            MTMount target elevation (deg)
        azimuth : `float` or `None`
            MTMount target azimuth (deg)
        rotation : `float` or `None`
            MTRotator target rotation angle (deg)
        temperature : `float` or `None`
            Target temperature (C).
        timeout : `float`
            Time limit (sec) for detecting that the CSC has read
            the new values.

        Warning: the temperature parameter is ignored.
        TODO DM-28005: write the temperature
        """
        if (elevation is None) != (azimuth is None):
            self.fail(
                f"elevation={elevation} and azimuth={azimuth} "
                "must both be numbers, or both be None"
            )
        did_something = False
        if elevation is not None:
            did_something = True
            self.mtmount_controller.evt_target.set_put(
                elevation=elevation, azimuth=azimuth
            )
        if rotation is not None:
            did_something = True
            self.mtrotator_controller.evt_target.set_put(position=rotation)
        if not did_something:
            self.fail("Must specify at least one non-None input")

        t0 = salobj.current_tai()
        while salobj.current_tai() - t0 < timeout:
            await asyncio.sleep(0.1)
            if elevation is not None:
                mtmount_target = self.csc.mtmount.evt_target.get()
                if (
                    mtmount_target is None
                    or abs(mtmount_target.elevation - elevation) > EPSILON
                    or abs(mtmount_target.azimuth - azimuth) > EPSILON
                ):
                    continue
            if rotation is not None:
                mtrotator_target = self.csc.mtrotator.evt_target.get()
                if (
                    mtrotator_target is None
                    or abs(mtrotator_target.position - rotation) > EPSILON
                ):
                    continue
            break
        else:
            self.fail(
                "Timed out waiting for MTMount or MTRotator data to be seen by the CSC"
            )

    def set_speed_factor(self, speed_factor):
        """Multiply the speed of each actuator by a specified factor.

        Useful for speeding up motions and thus test execution times.
        Be careful not to overdo it; moves should last longer than the
        telemetry interval so you reliably get events indicating
        that motion has begun.
        """
        for actuator in self.csc.mock_ctrl.hexapod.actuators:
            actuator.speed *= speed_factor

    async def test_bin_script(self):
        """Test running from the command line script."""
        await self.check_bin_script(
            name="MTHexapod",
            index=next(index_gen),
            exe_name="run_mthexapod.py",
            cmdline_args=["--simulate"],
        )

    async def test_constructor_errors(self):
        for bad_index in (0, 3):
            with self.assertRaises(ValueError):
                mthexapod.HexapodCsc(
                    index=bad_index,
                    initial_state=salobj.State.STANDBY,
                    config_dir=None,
                    simulation_mode=1,
                )

        # Bad simulation_mode, initial_state, and config_dir
        # are tested in ts_hexrotcomm.

    async def test_standard_state_transitions(self):
        async with self.make_csc(initial_state=salobj.State.STANDBY, simulation_mode=1):
            enabled_commands = (
                "configureVelocity",
                "configureAcceleration",
                "configureLimits",
                "move",
                "offset",
                "setCompensationMode",
                "setPivot",
                "stop",
            )
            await self.check_standard_state_transitions(
                enabled_commands=enabled_commands
            )

    async def test_configure_acceleration(self):
        """Test the configureAcceleration command."""
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            initial_limit = data.accelerationStrut
            new_limit = initial_limit - 0.1
            await self.remote.cmd_configureAcceleration.set_start(
                acceleration=new_limit, timeout=STD_TIMEOUT
            )
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(data.accelerationStrut, new_limit)

            for bad_acceleration in (-1, 0, mthexapod.MAX_ACCEL_LIMIT + 0.001):
                with self.subTest(bad_acceleration=bad_acceleration):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_configureAcceleration.set_start(
                            acceleration=bad_acceleration, timeout=STD_TIMEOUT
                        )

    def assert_dataclasses_almost_equal(self, dataclass1, dataclass2):
        """Assert two dataclasses or other instances that support vars
        have the same field names and nearly equal values for each field.
        """
        vars1 = vars(dataclass1)
        vars2 = vars(dataclass2)
        self.assertEqual(vars1, vars(dataclass1))
        self.assertEqual(vars1.keys(), vars2.keys())
        for name in vars1.keys():
            self.assertAlmostEqual(
                vars1[name], vars2[name], msg=f"{type(dataclass1).__name__}.{name}"
            )

    async def test_configure_limits(self):
        """Test the configureLimits command."""
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            self.set_speed_factor(20)

            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            initial_limits = mthexapod.PositionLimits.from_struct(data)

            # Use small limits for our extreme moves,
            # so we don't exceed the actuator length limits.
            new_limits_kwargs = {
                name: value * 0.1 for name, value in vars(initial_limits).items()
            }
            new_limits = mthexapod.PositionLimits(**new_limits_kwargs)
            await self.remote.cmd_configureLimits.set_start(
                **new_limits_kwargs,
                timeout=STD_TIMEOUT,
            )
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            reported_limits = mthexapod.PositionLimits.from_struct(data)
            self.assert_dataclasses_almost_equal(new_limits, reported_limits)

            # Test that we can move to the limits
            good_min_position = self.limits_to_min_position(new_limits)
            good_max_position = self.limits_to_max_position(new_limits)

            await self.basic_check_move(
                uncompensated_position=good_min_position, est_move_duration=2
            )

            await self.basic_check_move(
                uncompensated_position=good_max_position, est_move_duration=2
            )

            # Make sure we cannot move to a position outside the new limits
            for name in good_min_position.field_names():
                bad_min_position = copy.copy(good_min_position)
                setattr(bad_min_position, name, getattr(bad_min_position, name) * 1.01)

                bad_max_position = copy.copy(good_max_position)
                setattr(bad_max_position, name, getattr(bad_max_position, name) * 1.01)

                with salobj.assertRaisesAckError():
                    await self.remote.cmd_move.set_start(
                        **vars(bad_min_position), timeout=STD_TIMEOUT
                    )

                with salobj.assertRaisesAckError():
                    await self.remote.cmd_move.set_start(
                        **vars(bad_max_position), timeout=STD_TIMEOUT
                    )

            # Try setting limits that exceed the allowed values
            for name, value in vars(self.csc.max_pos_limits).items():
                bad_limits = copy.copy(self.csc.max_pos_limits)
                bad_value = value * 1.01
                setattr(bad_limits, name, bad_value)
                with self.subTest(name=name, bad_value=bad_value):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_configureLimits.set_start(
                            **vars(bad_limits),
                            timeout=STD_TIMEOUT,
                        )

    async def test_configure_velocity(self):
        """Test the configureVelocity command."""

        def get_velocity_limits(data):
            """Get the velocity limits from a configuration sample."""
            return (
                data.maxVelocityXY,
                data.maxVelocityUV,
                data.maxVelocityZ,
                data.maxVelocityW,
            )

        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            initial_vel_limits = get_velocity_limits(data)
            new_vel_limits = tuple(lim - 0.01 for lim in initial_vel_limits)
            await self.remote.cmd_configureVelocity.set_start(
                xy=new_vel_limits[0],
                uv=new_vel_limits[1],
                z=new_vel_limits[2],
                w=new_vel_limits[3],
                timeout=STD_TIMEOUT,
            )
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            reported_limits = get_velocity_limits(data)
            for i in range(4):
                self.assertAlmostEqual(new_vel_limits[i], reported_limits[i])

            bad_linear_vel_limit = mthexapod.MAX_LINEAR_VEL_LIMIT + 0.001
            bad_angular_vel_limit = mthexapod.MAX_ANGULAR_VEL_LIMIT + 0.001
            for bad_vel_limits in (
                (
                    0,
                    initial_vel_limits[1],
                    initial_vel_limits[2],
                    initial_vel_limits[3],
                ),
                (
                    bad_linear_vel_limit,
                    initial_vel_limits[1],
                    initial_vel_limits[2],
                    initial_vel_limits[3],
                ),
                (
                    initial_vel_limits[0],
                    0,
                    initial_vel_limits[2],
                    initial_vel_limits[3],
                ),
                (
                    initial_vel_limits[0],
                    bad_angular_vel_limit,
                    initial_vel_limits[2],
                    initial_vel_limits[3],
                ),
                (
                    initial_vel_limits[0],
                    initial_vel_limits[1],
                    0,
                    initial_vel_limits[3],
                ),
                (
                    initial_vel_limits[0],
                    initial_vel_limits[1],
                    bad_linear_vel_limit,
                    initial_vel_limits[3],
                ),
                (
                    initial_vel_limits[0],
                    initial_vel_limits[1],
                    initial_vel_limits[2],
                    0,
                ),
                (
                    initial_vel_limits[0],
                    initial_vel_limits[1],
                    initial_vel_limits[2],
                    bad_angular_vel_limit,
                ),
                (0, 0, 0, 0),
                (
                    bad_linear_vel_limit,
                    bad_angular_vel_limit,
                    bad_linear_vel_limit,
                    bad_angular_vel_limit,
                ),
            ):
                with self.subTest(bad_vel_limits=bad_vel_limits):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_configureVelocity.set_start(
                            xy=bad_vel_limits[0],
                            uv=bad_vel_limits[1],
                            z=bad_vel_limits[2],
                            w=bad_vel_limits[3],
                            timeout=STD_TIMEOUT,
                        )

    async def test_move_no_compensation_no_compensation_inputs(self):
        """Test move with compensation disabled when the CSC has
        no compensation inputs (which it should allow).
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            config_dir=local_config_dir,
            settings_to_apply="valid.yaml",
            simulation_mode=1,
        ):
            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )

            uncompensated_position = mthexapod.Position(
                300, 400, -300, 0.01, 0.02, -0.015
            )
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )
            await self.assert_next_compensated_position(uncompensated_position)
            await self.assert_next_application(desired_position=uncompensated_position)

    async def test_move_no_compensation_with_compensation_inputs(self):
        """Test move with compensation disabled when the CSC has
        compensation inputs (which it should ignore).
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            config_dir=local_config_dir,
            settings_to_apply="valid.yaml",
            simulation_mode=1,
        ):
            await self.set_compensation_inputs(
                elevation=45, azimuth=-50, rotation=88, temperature=23
            )

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )

            uncompensated_position = mthexapod.Position(
                300, 400, -300, 0.01, 0.02, -0.015
            )
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )
            await self.assert_next_compensated_position(uncompensated_position)
            await self.assert_next_application(desired_position=uncompensated_position)

    async def test_move_with_compensation_with_initial_compensation_inputs(self):
        """Test move with compensation enabled."""
        async with self.make_csc(
            config_dir=local_config_dir,
            initial_state=salobj.State.ENABLED,
            settings_to_apply="valid.yaml",
            simulation_mode=1,
        ):
            compensation_inputs_list = (
                mthexapod.CompensationInputs(
                    elevation=32, azimuth=44, rotation=-5, temperature=15
                ),
                mthexapod.CompensationInputs(
                    elevation=65, azimuth=44, rotation=-5, temperature=15
                ),
                mthexapod.CompensationInputs(
                    elevation=32, azimuth=190, rotation=-5, temperature=15
                ),
                mthexapod.CompensationInputs(
                    elevation=32, azimuth=44, rotation=20, temperature=15
                ),
                mthexapod.CompensationInputs(
                    elevation=32, azimuth=44, rotation=-5, temperature=-30
                ),
            )
            await self.set_compensation_inputs(**vars(compensation_inputs_list[0]))

            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )
            await self.remote.cmd_setCompensationMode.set_start(
                enable=True, timeout=STD_TIMEOUT
            )
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=True
            )

            await self.assert_next_application(desired_position=ZERO_POSITION)

            uncompensated_position = mthexapod.Position(
                500, -300, 200, 0.03, -0.02, 0.03
            )
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )

            update_inputs = False
            for compensation_inputs in compensation_inputs_list:
                await self.check_compensation(
                    uncompensated_position=uncompensated_position,
                    compensation_inputs=compensation_inputs,
                    update_inputs=update_inputs,
                )
                update_inputs = True

            # Test disabling compensation with setCompensationMode
            await self.remote.cmd_setCompensationMode.set_start(
                enable=False, timeout=STD_TIMEOUT
            )
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )

    async def test_move_with_compensation_no_initial_compensation_inputs(self):
        """Test move with compensation enabled but no compensation inputs.

        This should act like an uncompensated move.
        """
        async with self.make_csc(
            config_dir=local_config_dir,
            initial_state=salobj.State.ENABLED,
            settings_to_apply="valid.yaml",
            simulation_mode=1,
        ):
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )
            await self.remote.cmd_setCompensationMode.set_start(
                enable=True, timeout=STD_TIMEOUT
            )
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=True
            )

            uncompensated_position = mthexapod.Position(
                500, -300, 200, 0.03, -0.02, 0.03
            )
            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )
            await self.assert_next_application(desired_position=uncompensated_position)

            # Test disabling compensation by sending the CSC
            # out of the enabled state.
            await self.remote.cmd_disable.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )

    async def test_move_interrupt_move_after_delay(self):
        """Test that one move can interrupt another
        after the first move is reported to have begun.
        """
        positions_data = (
            (0, 0, -1000, 0, 0, 0),
            (0, 0, 1000, 0, 0, 0),
            (0, 0, -400, 0, 0, 0),
        )
        positions = [mthexapod.Position(*data) for data in positions_data]
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            config_dir=local_config_dir,
            settings_to_apply="valid.yaml",
            simulation_mode=1,
        ):
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(
                self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            isfirst = True
            for position in positions:
                print("command a move")
                await self.remote.cmd_move.set_start(
                    **vars(position), timeout=STD_TIMEOUT
                )
                await self.assert_next_uncompensated_position(position)
                if isfirst:
                    isfirst = False
                else:
                    # The new move should halt the old move
                    await self.assert_next_sample(
                        self.remote.evt_controllerState,
                        controllerState=ControllerState.ENABLED,
                        enabledSubstate=EnabledSubstate.STATIONARY,
                    )
                # Wait for the new move to begin
                await self.assert_next_sample(
                    self.remote.evt_controllerState,
                    controllerState=ControllerState.ENABLED,
                    enabledSubstate=EnabledSubstate.MOVING_POINT_TO_POINT,
                )

            # Make sure the commanded position is indeed the last position.
            desired_position = positions[-1]
            self.assert_positions_close(
                self.csc.mock_ctrl.telemetry.commanded_pos, desired_position
            )

            # Wait for the last move to finish and check that we are at the
            # desired position.
            await self.assert_next_sample(
                self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_application.next(
                flush=True, timeout=STD_TIMEOUT
            )
            self.assert_positions_close(data.demand, desired_position)

    async def test_move_interrupt_move_immediately(self):
        """Test that one move can interrupt another right away."""
        positions_data = (
            (0, 0, -1000, 0, 0, 0),
            (0, 0, 1000, 0, 0, 0),
            (0, 0, -400, 0, 0, 0),
        )
        positions = [mthexapod.Position(*data) for data in positions_data]
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            config_dir=local_config_dir,
            settings_to_apply="valid.yaml",
            simulation_mode=1,
        ):
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(
                self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            move_tasks = []
            for position in positions:
                print("command a move")
                move_tasks.append(
                    asyncio.create_task(
                        self.remote.cmd_move.set_start(
                            **vars(position), timeout=STD_TIMEOUT
                        )
                    )
                )
                # Give the CSC a chance to start processing the command
                await asyncio.sleep(0)

            # Wait for the final move task to finish
            await move_tasks[-1]
            # The other move tasks should also be done
            # (if I was quick enough then they should have raised an AckError,
            # but it's hard to run the test fast enough for that)
            for task in move_tasks[0:-1]:
                assert task.done()

            # Give the mock controller telemetry loop some time
            await asyncio.sleep(self.csc.mock_ctrl.telemetry_interval * 3)

            # Make sure the commanded position is indeed the last position.
            desired_position = positions[-1]
            self.assert_positions_close(
                self.csc.mock_ctrl.telemetry.commanded_pos, desired_position
            )

            # Do not test the controllerState event because it is
            # uncertain how many transitions will have occurred
            # during the consecutive moves.

    async def test_offset_no_compensation(self):
        """Test offset with compensation disabled."""
        first_uncompensated_position = mthexapod.Position(
            100, 200, -300, 0.01, 0.02, -0.015
        )
        offset = mthexapod.Position(50, -100, 135, 0.005, -0.005, 0.01)
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            config_dir=local_config_dir,
            settings_to_apply="valid.yaml",
            simulation_mode=1,
        ):
            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.check_offset(
                first_uncompensated_position=first_uncompensated_position,
                offset=offset,
                est_move_duration=1,
            )
            desired_uncompensated_position = np.add(
                first_uncompensated_position, offset
            )
            await self.assert_next_application(
                desired_position=desired_uncompensated_position
            )

    async def test_offset_with_compensation(self):
        """Test offset with compensation enabled."""
        first_uncompensated_position = mthexapod.Position(
            100, 200, -300, 0.01, 0.02, -0.015
        )
        offset = mthexapod.Position(50, -100, 135, 0.005, -0.005, 0.01)
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            config_dir=local_config_dir,
            settings_to_apply="valid.yaml",
            simulation_mode=1,
        ):
            compensation_inputs_list = (
                mthexapod.CompensationInputs(
                    elevation=32, azimuth=44, rotation=-5, temperature=15
                ),
                mthexapod.CompensationInputs(
                    elevation=65, azimuth=44, rotation=-5, temperature=15
                ),
            )
            await self.set_compensation_inputs(**vars(compensation_inputs_list[0]))

            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )
            await self.remote.cmd_setCompensationMode.set_start(
                enable=True, timeout=STD_TIMEOUT
            )
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=True
            )

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.check_offset(
                first_uncompensated_position=first_uncompensated_position,
                offset=offset,
                est_move_duration=1,
            )

            uncompensated_position = np.add(first_uncompensated_position, offset)

            update_inputs = False
            for compensation_inputs in compensation_inputs_list:
                await self.check_compensation(
                    uncompensated_position=uncompensated_position,
                    compensation_inputs=compensation_inputs,
                    update_inputs=update_inputs,
                )
                update_inputs = True

    async def test_set_pivot(self):
        """Test the setPivot command."""
        axis_names = ("x", "y", "z")

        def get_pivot(config):
            return {
                name: getattr(config, f"pivot{name.upper()}") for name in axis_names
            }

        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            initial_config = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            old_pivot = get_pivot(initial_config)

            commanded_pivot = {name: val + 10 for name, val in old_pivot.items()}
            await self.remote.cmd_setPivot.set_start(
                **commanded_pivot, timeout=STD_TIMEOUT
            )

            new_config = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            new_pivot = {
                name: getattr(new_config, f"pivot{name.upper()}") for name in axis_names
            }
            for name in axis_names:
                self.assertAlmostEqual(new_pivot[name], commanded_pivot[name])

    async def test_stop_move_after_delay(self):
        """Test stopping a move after giving it time to start."""
        # Command a move that moves all actuators equally
        position = mthexapod.Position(0, 0, 1000, 0, 0, 0)
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_sample(
                topic=self.remote.evt_softwareVersions,
                cscVersion=mthexapod.__version__,
                subsystemVersions="",
            )
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.remote.cmd_move.set_start(**vars(position), timeout=STD_TIMEOUT)
            cmd_lengths = [
                actuator.end_position
                for actuator in self.csc.mock_ctrl.hexapod.actuators
            ]
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.MOVING_POINT_TO_POINT,
            )
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            await self.remote.tel_application.next(flush=True, timeout=STD_TIMEOUT)
            # The Mock controller does not compute position as a function
            # of actuator lengths, so test that motion halted by examining
            # the actuators.
            stopped_lengths = [
                actuator.end_position
                for actuator in self.csc.mock_ctrl.hexapod.actuators
            ]
            for i in range(6):
                self.assertNotAlmostEqual(cmd_lengths[i], stopped_lengths[i])

    async def test_stop_move_immediately(self):
        """Test that stop can interrupt a move right away."""
        position = copy.copy(ZERO_POSITION)
        position.z = 1000
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            config_dir=local_config_dir,
            settings_to_apply="valid.yaml",
            simulation_mode=1,
        ):
            self.csc.log.level = 10
            await self.assert_next_sample(
                topic=self.remote.evt_compensationMode, enabled=False
            )

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(
                self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            move_task = asyncio.create_task(
                self.remote.cmd_move.set_start(**vars(position), timeout=STD_TIMEOUT)
            )

            # Give the CSC a chance to start processing the command
            await asyncio.sleep(0.1)

            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            await asyncio.sleep(0)
            self.assertTrue(move_task.done())

            # Give the mock controller telemetry loop some time
            await asyncio.sleep(self.csc.mock_ctrl.telemetry_interval * 3)

            # Make sure the commanded position is indeed the last position
            # (in other words: that the move command was actually sent
            # to the low-level controller).
            self.assert_positions_close(
                self.csc.mock_ctrl.telemetry.commanded_pos, position
            )

            # Make sure the controller is stopped
            self.assertEqual(
                self.csc.mock_ctrl.telemetry.state,
                ControllerState.ENABLED,
            )
            self.assertEqual(
                self.csc.mock_ctrl.telemetry.enabled_substate,
                EnabledSubstate.STATIONARY,
            )

            # Do not test the controllerState event because it is
            # uncertain how many transitions will have occurred.


if __name__ == "__main__":
    unittest.main()
