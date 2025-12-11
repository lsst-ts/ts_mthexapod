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
import math
import pathlib
import time
import types
import typing
import unittest

import numpy as np
import pytest
import yaml
from numpy.testing import assert_allclose

from lsst.ts import hexrotcomm, mthexapod, salobj, utils
from lsst.ts.xml.enums.MTHexapod import ControllerState, EnabledSubstate
from lsst.ts.xml.sal_enums import State

STD_TIMEOUT = 10  # timeout for command ack
EPSILON = 1e-10  # approx allowed error between float and double

ZERO_POSITION = mthexapod.Position(0, 0, 0, 0, 0, 0)

logging.basicConfig()

TEST_CONFIG_DIR = pathlib.Path(__file__).parent / "data" / "config"


class TestHexapodCsc(hexrotcomm.BaseCscTestCase, unittest.IsolatedAsyncioTestCase):
    def basic_make_csc(
        self,
        initial_state: salobj.State,
        config_dir: pathlib.Path | None = None,
        override: str = "",
        simulation_mode: int = 1,
    ) -> mthexapod.HexapodCsc:
        return mthexapod.HexapodCsc(
            index=1,
            initial_state=initial_state,
            override=override,
            simulation_mode=simulation_mode,
            config_dir=config_dir,
        )

    @contextlib.asynccontextmanager
    async def make_csc(
        self,
        initial_state: salobj.State = salobj.State.STANDBY,
        config_dir: pathlib.Path = TEST_CONFIG_DIR,
        override: str = "",
        simulation_mode: int = 1,
        log_level: int | None = None,
    ) -> mthexapod.HexapodCsc:
        async with (
            super().make_csc(
                initial_state=initial_state,
                config_dir=config_dir,
                override=override,
                simulation_mode=simulation_mode,
                log_level=log_level,
            ),
            salobj.Controller(
                name="MTMount",
            ) as self.mtmount_controller,
            salobj.Controller(
                name="MTRotator",
            ) as self.mtrotator_controller,
            salobj.Controller(
                name="ESS",
                index=121,
            ) as self.ess_temperature_controller,
        ):
            # self.mtmount_controller = mtmount_controller
            # self.mtrotator_controller = mtrotator_controller
            # self.ess_temperature_controller = ess_temperature_controller
            yield

    async def assert_next_application(self, desired_position: mthexapod.Position) -> None:
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
            data = await self.remote.tel_application.next(flush=True, timeout=STD_TIMEOUT)
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
        self.assert_positions_close(data.position, desired_position, pos_atol=1, ang_atol=1e-5)

    async def assert_next_compensation(
        self,
        compensation_inputs: mthexapod.CompensationInputs,
        offset: mthexapod.Position,
    ) -> None:
        """Wait for and check the next compensation event.

        Parameters
        ----------
        compensation_inputs : `CompensationInputs`
            Compensation inputs.
        offset : `Position`
            Expected compensation offset.
        """
        data = await self.assert_next_sample(self.remote.evt_compensationOffset)
        assert data.elevation == pytest.approx(compensation_inputs.elevation)
        assert data.azimuth == pytest.approx(compensation_inputs.azimuth)
        assert data.rotation == pytest.approx(compensation_inputs.rotation)
        assert data.temperature == pytest.approx(compensation_inputs.temperature)

        for i, name in enumerate(("x", "y", "z", "u", "v", "w")):
            assert getattr(data, name) == pytest.approx(getattr(offset, name))

    async def assert_next_compensated_position(self, position: mthexapod.Position) -> None:
        """Wait for and check the next uncompensatedPosition event.

        Parameters
        ----------
        position : `Position`
            Expected compensated position.
        """
        data = await self.assert_next_sample(self.remote.evt_compensatedPosition)
        read_position = mthexapod.Position.from_struct(data)

        self.assert_dataclasses_almost_equal(position, read_position)

    async def assert_next_uncompensated_position(self, position: mthexapod.Position) -> None:
        """Wait for and check the next uncompensatedPosition event.

        Parameters
        ----------
        position : `Position`
            Expected uncompensated position.
        """
        data = await self.assert_next_sample(self.remote.evt_uncompensatedPosition)
        read_position = mthexapod.Position.from_struct(data)

        self.assert_dataclasses_almost_equal(position, read_position)

    async def assert_initial_compensation_values(self) -> None:
        """Check that the CSC resets compensation information when enabled."""

        compensation_offset = await self.assert_next_sample(
            topic=self.remote.evt_compensationOffset,
        )
        compensation_position = await self.assert_next_sample(
            topic=self.remote.evt_compensatedPosition,
        )
        uncompensation_position = await self.assert_next_sample(
            topic=self.remote.evt_uncompensatedPosition,
        )

        names = mthexapod.base.Position.field_names()

        assert math.isnan(compensation_offset.elevation)
        assert math.isnan(compensation_offset.azimuth)
        assert math.isnan(compensation_offset.rotation)
        assert math.isnan(compensation_offset.temperature)
        for name in names:
            assert math.isnan(getattr(compensation_offset, name))

        for name in names:
            assert math.isnan(getattr(compensation_position, name))

        for name in names:
            assert math.isnan(getattr(uncompensation_position, name))

    def assert_positions_close(
        self,
        position1: mthexapod.Position | tuple[float, float, float, float, float, float],
        position2: mthexapod.Position | tuple[float, float, float, float, float, float],
        pos_atol: float = 1e-2,
        ang_atol: float = 1e-7,
    ) -> None:
        """Assert that two positions are close.

        Parameters
        ----------
        position1 : `Position` or `list` [`float`]
            First position to check.
        position1 : `Position` or `list` [`float`]
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
        assert_allclose(position1_tuple[:3], position2_tuple[:3], atol=pos_atol)
        assert_allclose(position1_tuple[3:], position2_tuple[3:], atol=ang_atol)

    async def check_move(
        self,
        uncompensated_position: mthexapod.Position,
        est_move_duration: float,
        speed_factor: float = 2.0,
    ) -> None:
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

    async def basic_check_move(
        self, uncompensated_position: mthexapod.Position, est_move_duration: float
    ) -> None:
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
        await self.remote.cmd_move.set_start(**vars(uncompensated_position), timeout=STD_TIMEOUT)
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
                f"Move timed out in {STD_TIMEOUT + est_move_duration} seconds; "
                f"remaining move time {self.csc.mock_ctrl.hexapod.remaining_time():0.2f}"
            )

        await self.assert_next_sample(self.remote.evt_inPosition, inPosition=False)
        await self.assert_next_sample(self.remote.evt_inPosition, inPosition=True)

        print(f"Move duration: {time.time() - t0:0.2f} seconds")
        await self.assert_next_uncompensated_position(position=uncompensated_position)

    async def check_compensation(
        self,
        uncompensated_position: mthexapod.Position,
        compensation_inputs: mthexapod.CompensationInputs,
        update_inputs: bool,
    ) -> None:
        """Check compensation for a given set of of compensation inputs.

        Parameters
        ----------
        uncompensated_position : `Position`
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
            or abs(data.temperature - compensation_inputs.temperature) > EPSILON
        ):
            data = await self.remote.evt_compensationOffset.next(flush=False, timeout=STD_TIMEOUT)

        compensation_offset = self.csc.compensation.get_offset(compensation_inputs)
        if update_inputs:
            # We set the compensation inputs, so we know compensation
            # should be nonzero in at least one axis.
            nonzero = [offset != 0 for offset in dataclasses.astuple(compensation_offset)]
            assert True in nonzero

        reported_compensation_offset = mthexapod.Position.from_struct(data)
        self.assert_dataclasses_almost_equal(reported_compensation_offset, compensation_offset)

        desired_compensated_position = uncompensated_position + compensation_offset
        await self.assert_next_application(desired_position=desired_compensated_position)

    async def check_offset(
        self,
        first_uncompensated_position: mthexapod.Position,
        offset: mthexapod.Position,
        est_move_duration: float,
    ) -> None:
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

    def get_compensation_timestamps(self) -> tuple[float, float]:
        """Get private_sndStamp of most recent compensationOffset and
        compensatedPosition events.
        """
        offset_data = self.remote.evt_compensationOffset.get()
        assert offset_data is not None
        comp_pos_data = self.remote.evt_compensatedPosition.get()
        assert comp_pos_data is not None
        return (offset_data.private_sndStamp, comp_pos_data.private_sndStamp)

    def limits_to_min_position(self, limits: mthexapod.PositionLimits) -> mthexapod.Position:
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

    def limits_to_max_position(self, limits: mthexapod.PositionLimits) -> mthexapod.Position:
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
        self,
        elevation: float | None,
        azimuth: float | None,
        rotation: float | None,
        temperature: float | None,
        timeout: float = STD_TIMEOUT,
    ) -> None:
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
        """
        if (elevation is None) != (azimuth is None):
            self.fail(f"elevation={elevation} and azimuth={azimuth} must both be numbers, or both be None")
        did_something = False
        if elevation is not None:
            did_something = True
            await self.mtmount_controller.evt_target.set_write(elevation=elevation, azimuth=azimuth)
        if rotation is not None:
            did_something = True
            await self.mtrotator_controller.evt_target.set_write(position=rotation)
        if temperature is not None:
            did_something = True

            temperatureItem = [np.nan] * 16
            temperatureItem[0:6] = [temperature] * 6
            await self.ess_temperature_controller.tel_temperature.set_write(temperatureItem=temperatureItem)

        if not did_something:
            self.fail("Must specify at least one non-None input")

        t0 = utils.current_tai()
        while utils.current_tai() - t0 < timeout:
            await asyncio.sleep(0.1)
            if elevation is not None:
                # Workaround the mypy check
                assert azimuth is not None

                mtmount_target = self.csc.mtmount.evt_target.get()
                if mtmount_target is None or not np.allclose(
                    [mtmount_target.elevation, mtmount_target.azimuth],
                    [elevation, azimuth],
                    atol=EPSILON,
                    equal_nan=True,
                ):
                    continue
            if rotation is not None:
                mtrotator_target = self.csc.mtrotator.evt_target.get()
                if mtrotator_target is None or not np.allclose(
                    mtrotator_target.position,
                    rotation,
                    atol=EPSILON,
                    equal_nan=True,
                ):
                    continue
            break
        else:
            self.fail("Timed out waiting for MTMount or MTRotator data to be seen by the CSC")

    def set_speed_factor(self, speed_factor: float) -> None:
        """Multiply the speed of each actuator by a specified factor.

        Useful for speeding up motions and thus test execution times.
        Be careful not to overdo it; moves should last longer than the
        telemetry interval so you reliably get events indicating
        that motion has begun.
        """
        for actuator in self.csc.mock_ctrl.hexapod.actuators:
            actuator.speed *= speed_factor

    async def test_bin_script(self) -> None:
        """Test running from the command line script."""
        for idx in (1, 2):
            await self.check_bin_script(
                name="MTHexapod",
                index=idx,
                exe_name="run_mthexapod",
                cmdline_args=["--simulate"],
            )

    async def test_constructor_errors(self) -> None:
        for bad_index in (0, 3):
            with pytest.raises(ValueError):
                mthexapod.HexapodCsc(
                    index=bad_index,
                    initial_state=salobj.State.STANDBY,
                    config_dir=None,
                    simulation_mode=1,
                )

        # Bad simulation_mode, initial_state, and config_dir
        # are tested in ts_hexrotcomm.

    async def test_standard_state_transitions(self) -> None:
        async with self.make_csc(initial_state=salobj.State.STANDBY, simulation_mode=1):
            enabled_commands = (
                "configureVelocity",
                "configureAcceleration",
                "configureLimits",
                "move",
                "moveInSteps",
                "offset",
                "offsetInSteps",
                "setCompensationMode",
                "setPivot",
                "stop",
            )
            await self.check_standard_state_transitions(
                enabled_commands=enabled_commands,
            )

    async def test_bad_configurations(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY,
            simulation_mode=1,
        ):
            with pytest.raises(salobj.AckError, match="Configuration error: Missing camera selection."):
                await self.remote.cmd_start.set_start(
                    configurationOverride="bad_no_compensation_while_exposing.yaml",
                    timeout=STD_TIMEOUT,
                )

    async def test_configure_acceleration(self) -> None:
        """Test the configureAcceleration command."""
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
            initial_limit = data.accelerationStrut

            self.remote.evt_configuration.flush()

            new_limit = initial_limit - 0.1
            await self.remote.cmd_configureAcceleration.set_start(acceleration=new_limit, timeout=STD_TIMEOUT)
            data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
            assert data.accelerationStrut == pytest.approx(new_limit)

            for bad_acceleration in (-1, 0, mthexapod.MAX_ACCEL_LIMIT + 0.001):
                with self.subTest(bad_acceleration=bad_acceleration):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_configureAcceleration.set_start(
                            acceleration=bad_acceleration, timeout=STD_TIMEOUT
                        )

    def assert_dataclasses_almost_equal(
        self, dataclass1: types.SimpleNamespace, dataclass2: types.SimpleNamespace
    ) -> None:
        """Assert two dataclasses or other instances that support vars
        have the same field names and nearly equal values for each field.
        """
        vars1 = vars(dataclass1)
        vars2 = vars(dataclass2)
        assert vars1 == vars(dataclass1)
        assert vars1.keys() == vars2.keys()
        for name in vars1.keys():
            assert vars1[name] == pytest.approx(vars2[name]), f"{type(dataclass1).__name__}.{name}"

    async def test_configure_limits(self) -> None:
        """Test the configureLimits command."""
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            await self.assert_initial_compensation_values()

            self.set_speed_factor(20)

            data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
            initial_limits = mthexapod.PositionLimits.from_struct(data)

            self.remote.evt_configuration.flush()

            # Use small limits for our extreme moves,
            # so we don't exceed the actuator length limits.
            new_limits_kwargs = {name: value * 0.1 for name, value in vars(initial_limits).items()}
            new_limits = mthexapod.PositionLimits(**new_limits_kwargs)
            await self.remote.cmd_configureLimits.set_start(
                **new_limits_kwargs,
                timeout=STD_TIMEOUT,
            )
            data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
            reported_limits = mthexapod.PositionLimits.from_struct(data)
            self.assert_dataclasses_almost_equal(new_limits, reported_limits)

            # Test that we can move to the limits
            good_min_position = self.limits_to_min_position(new_limits)
            good_max_position = self.limits_to_max_position(new_limits)

            await self.basic_check_move(uncompensated_position=good_min_position, est_move_duration=2)

            await self.basic_check_move(uncompensated_position=good_max_position, est_move_duration=2)

            # Make sure we cannot move to a position outside the new limits
            for name in good_min_position.field_names():
                with self.subTest(name=name):
                    bad_min_position = copy.copy(good_min_position)
                    setattr(bad_min_position, name, getattr(bad_min_position, name) * 1.01)

                    bad_max_position = copy.copy(good_max_position)
                    setattr(bad_max_position, name, getattr(bad_max_position, name) * 1.01)

                    with salobj.assertRaisesAckError():
                        await self.remote.cmd_move.set_start(**vars(bad_min_position), timeout=STD_TIMEOUT)

                    with salobj.assertRaisesAckError():
                        await self.remote.cmd_move.set_start(**vars(bad_max_position), timeout=STD_TIMEOUT)

            # Try setting limits that exceed the allowed values
            for name, value in vars(self.csc.max_pos_limits).items():
                with self.subTest(name=name):
                    bad_limits = copy.copy(self.csc.max_pos_limits)
                    bad_value = value * 1.01
                    setattr(bad_limits, name, bad_value)
                    with self.subTest(name=name, bad_value=bad_value):
                        with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                            await self.remote.cmd_configureLimits.set_start(
                                **vars(bad_limits),
                                timeout=STD_TIMEOUT,
                            )

    async def test_configure_velocity(self) -> None:
        """Test the configureVelocity command."""

        def get_velocity_limits(
            data: salobj.BaseMsgType,
        ) -> tuple[float, float, float, float]:
            """Get the velocity limits from a configuration sample."""
            return (
                data.maxVelocityXY,
                data.maxVelocityZ,
                data.maxVelocityUV,
                data.maxVelocityW,
            )

        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
            initial_vel_limits = get_velocity_limits(data)
            new_vel_limits = tuple(lim - 0.01 for lim in initial_vel_limits)

            self.remote.evt_configuration.flush()

            await self.remote.cmd_configureVelocity.set_start(
                xy=new_vel_limits[0],
                z=new_vel_limits[1],
                uv=new_vel_limits[2],
                w=new_vel_limits[3],
                timeout=STD_TIMEOUT,
            )
            data = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
            reported_limits = get_velocity_limits(data)
            assert_allclose(new_vel_limits, reported_limits, atol=1e-7)

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

    async def test_electrical_telemetry(self) -> None:
        """Test motor current and velocity with a simple move.

        Note that the mock controller always reports
        copleyStatusWordDrive and copleyLatchingFaultStatus as zero.

        Also test the timestamp field of the actuator telemetry topic.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)

            data = await self.remote.tel_actuators.next(flush=True, timeout=STD_TIMEOUT)
            tai = utils.current_tai()
            # No need to be picky; it just needs to be close.
            assert data.timestamp == pytest.approx(tai, abs=0.5)

            expected_bus_voltage = [mthexapod.mock_controller.BUS_VOLTAGE] * 3
            data = await self.remote.tel_electrical.next(flush=True, timeout=STD_TIMEOUT)
            assert_allclose(data.motorCurrent, [0] * 6)
            assert_allclose(data.busVoltage, expected_bus_voltage)

            uncompensated_position = mthexapod.Position(0, 0, 1000, 0, 0, 0)
            await self.remote.cmd_move.set_start(**vars(uncompensated_position), timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.MOVING_POINT_TO_POINT,
            )
            data = await self.remote.tel_electrical.next(flush=True, timeout=STD_TIMEOUT)
            np.testing.assert_array_less([0] * 6, np.abs(data.motorCurrent))
            assert_allclose(data.busVoltage, expected_bus_voltage)

    async def test_get_mount_elevation_azimuth_use_telemetry(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            assert self.csc._get_mount_elevation_azimuth(None, None, None) is None

            # Use the telemetry data
            await self.mtmount_controller.tel_elevation.set_write(actualPosition=0.1)
            await self.csc.mtmount.tel_elevation.next(flush=True, timeout=STD_TIMEOUT)

            await self.mtmount_controller.tel_azimuth.set_write(actualPosition=0.2)
            await self.csc.mtmount.tel_azimuth.next(flush=True, timeout=STD_TIMEOUT)

            assert self.csc._get_mount_elevation_azimuth(
                None,
                self.csc.mtmount.tel_elevation.get(),
                self.csc.mtmount.tel_azimuth.get(),
            ) == (0.1, 0.2)

    async def test_get_rotator_position_use_telemetry(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            assert self.csc._get_rotator_position(None, None) is None

            # Use the telemetry data
            await self.mtrotator_controller.tel_rotation.set_write(actualPosition=0.4)
            await self.csc.mtrotator.tel_rotation.next(flush=True, timeout=STD_TIMEOUT)

            assert self.csc._get_rotator_position(None, self.csc.mtrotator.tel_rotation.get()) == 0.4

    async def test_get_temperature_use_telemetry(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            # No data yet
            assert self.csc._get_temperature() == 0.0

            # New telemetry data
            await self.set_compensation_inputs(elevation=0, azimuth=0, rotation=0, temperature=23)

            assert self.csc._get_temperature() == 23.0

            # Disable the temperature LUT
            self.csc.config.camera_config["enable_lut_temperature"] = False

            assert self.csc._get_temperature() == 0.0

    async def test_check_position(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            # Normal condition
            position_normal = mthexapod.Position(0.0, 0.0, -10000.0, -0.22, 0.0, 0.0)
            self.csc._check_position(position_normal)

            # Should fail because of the delta strut length is out of range
            with self.assertRaises(salobj.ExpectedError):
                position_bad = mthexapod.Position(0.0, 0.0, -10000.0, -0.29, 0.0, 0.0)
                self.csc._check_position(position_bad)

            with self.assertRaises(salobj.ExpectedError):
                position_bad = mthexapod.Position(0.0, 0.0, -10000.0, 0.29, 0.0, 0.0)
                self.csc._check_position(position_bad)

    async def test_move_no_compensation_no_compensation_inputs(self) -> None:
        """Test move with compensation disabled when the CSC has
        no compensation inputs (which it should allow).
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_initial_compensation_values()
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)

            uncompensated_position = mthexapod.Position(300, 400, -300, 0.01, 0.02, -0.015)
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )
            await self.assert_next_compensated_position(uncompensated_position)
            await self.assert_next_application(desired_position=uncompensated_position)

    async def test_move_in_steps(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_initial_compensation_values()

            self.remote.evt_inPosition.flush()

            # Assign the step size
            await self.remote.cmd_moveInSteps.set_start(
                x=100.0, z=500.0, v=0.1, stepSizeZ=79.0, stepSizeUV=0.013
            )

            await self.check_in_position_event_and_position_telemetry(
                pos_atol=1e-1,
                ang_atol=1e-6,
            )

            # Overwrite the step size from the configuration
            await self.remote.cmd_moveInSteps.set_start(x=200.0, z=1000.0, overwriteStepSizeFromConfig=True)

            await self.check_in_position_event_and_position_telemetry(
                pos_atol=1e-1,
                ang_atol=1e-6,
            )

    async def check_in_position_event_and_position_telemetry(
        self,
        pos_atol: float = 1e-2,
        ang_atol: float = 1e-7,
    ) -> None:
        await self.assert_next_sample(
            self.remote.evt_inPosition,
            timeout=STD_TIMEOUT,
            inPosition=False,
        )
        await self.assert_next_sample(
            self.remote.evt_inPosition,
            timeout=STD_TIMEOUT,
            inPosition=True,
        )

        data = await self.remote.tel_application.next(flush=True, timeout=STD_TIMEOUT)
        self.assert_positions_close(
            data.position,
            data.demand,
            pos_atol=pos_atol,
            ang_atol=ang_atol,
        )

    async def test_offset_in_steps(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_initial_compensation_values()

            self.remote.evt_inPosition.flush()

            # Assign the step size

            # First offset
            await self.remote.cmd_offsetInSteps.set_start(
                x=100.0, z=500.0, v=0.1, stepSizeZ=79.0, stepSizeUV=0.013
            )

            await self.check_in_position_event_and_position_telemetry(
                pos_atol=1e-1,
                ang_atol=1e-6,
            )

            # Second offset
            await self.remote.cmd_offsetInSteps.set_start(
                x=-10.0, z=-100.0, v=0.009, stepSizeZ=79.0, stepSizeUV=0.013
            )

            await self.check_in_position_event_and_position_telemetry(
                pos_atol=1e-1,
                ang_atol=1e-6,
            )

            # Overwrite the step size from the configuration
            await self.remote.cmd_offsetInSteps.set_start(
                x=-10.0, z=-100.0, v=0.009, overwriteStepSizeFromConfig=True
            )

            await self.check_in_position_event_and_position_telemetry(
                pos_atol=1e-1,
                ang_atol=1e-6,
            )

    async def test_move_no_compensation_with_compensation_inputs(self) -> None:
        """Test move with compensation disabled when the CSC has
        compensation inputs (which it should ignore).
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_initial_compensation_values()
            await self.set_compensation_inputs(elevation=45, azimuth=-50, rotation=88, temperature=23)

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)

            uncompensated_position = mthexapod.Position(300, 400, -300, 0.01, 0.02, -0.015)
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )
            await self.assert_next_compensated_position(uncompensated_position)
            await self.assert_next_application(desired_position=uncompensated_position)

    async def test_move_with_compensation_with_initial_compensation_inputs(
        self,
    ) -> None:
        """Test move with compensation enabled."""
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_initial_compensation_values()
            compensation_inputs_list = (
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=65, azimuth=44, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=32, azimuth=190, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=20, temperature=15),
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=-5, temperature=-30),
            )
            await self.set_compensation_inputs(**vars(compensation_inputs_list[0]))

            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)
            await self.remote.cmd_setCompensationMode.set_start(enable=True, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=True)

            await self.assert_next_application(desired_position=ZERO_POSITION)

            uncompensated_position = mthexapod.Position(500, -300, 200, 0.03, -0.02, 0.03)
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )

            update_inputs = False
            for compensation_inputs in compensation_inputs_list:
                with self.subTest(compensation_inputs=compensation_inputs):
                    await self.check_compensation(
                        uncompensated_position=uncompensated_position,
                        compensation_inputs=compensation_inputs,
                        update_inputs=update_inputs,
                    )
                    update_inputs = True

            # Set each input to NaN in turn.
            # This should prevent compensation updates
            # and should update bad_inputs_str
            # input_name is the name of the item in CompensationInputs;
            # input_descr is the description of the field that the CSC uses,
            # which includes the name of the CSC, event, and field.
            prev_bad_inputs_str = self.csc.bad_inputs_str
            for input_name, input_descr in (
                ("elevation", "MTMount.elevation"),
                ("azimuth", "MTMount.azimuth"),
                ("rotation", "MTRotator.position"),
            ):
                with self.subTest(input_name=input_name, input_descr=input_descr):
                    nan_compensation_inputs = copy.copy(compensation_inputs)
                    setattr(nan_compensation_inputs, input_name, math.nan)
                    await self.set_compensation_inputs(**vars(nan_compensation_inputs))
                    t0 = utils.current_tai()
                    while prev_bad_inputs_str == self.csc.bad_inputs_str:
                        if utils.current_tai() - t0 > STD_TIMEOUT:
                            self.fail("Timed out waiting for bad_inputs_str to be updated")
                        await asyncio.sleep(0.1)
                    assert input_descr in self.csc.bad_inputs_str
                    prev_bad_inputs_str = self.csc.bad_inputs_str

            # Try finite inputs again; this should work.
            await self.check_compensation(
                uncompensated_position=uncompensated_position,
                compensation_inputs=compensation_inputs_list[-2],
                update_inputs=True,
            )

            # Test disabling compensation with setCompensationMode
            await self.remote.cmd_setCompensationMode.set_start(enable=False, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)

    async def test_move_with_compensation_no_initial_compensation_inputs(self) -> None:
        """Test move with compensation enabled but no compensation inputs.

        This should act like an uncompensated move.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_initial_compensation_values()
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)
            await self.remote.cmd_setCompensationMode.set_start(enable=True, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=True)

            uncompensated_position = mthexapod.Position(500, -300, 200, 0.03, -0.02, 0.03)
            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )

            # Test disabling compensation by sending the CSC
            # out of the enabled state.
            await self.remote.cmd_disable.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)

    async def test_move_interrupt_move_after_delay(self) -> None:
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
            override="",
            simulation_mode=1,
        ):
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)
            await self.assert_initial_compensation_values()

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(
                self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            isfirst = True
            for position in positions:
                await self.remote.cmd_move.set_start(**vars(position), timeout=STD_TIMEOUT)
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
            self.assert_positions_close(self.csc.mock_ctrl.telemetry.commanded_pos, desired_position)

            # Wait for the last move to finish and check that we are at the
            # desired position.
            await self.assert_next_sample(
                self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_application.next(flush=True, timeout=STD_TIMEOUT)
            self.assert_positions_close(data.demand, desired_position)

    async def test_move_interrupt_move_immediately(self) -> None:
        """Test that one move can interrupt another right away."""
        positions_data = (
            (0, 0, -1000, 0, 0, 0),
            (0, 0, 1000, 0, 0, 0),
            (0, 0, -400, 0, 0, 0),
        )
        positions = [mthexapod.Position(*data) for data in positions_data]
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(
                self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            move_tasks = []
            for position in positions:
                self.csc.move_command_received_event.clear()
                move_tasks.append(
                    asyncio.create_task(self.remote.cmd_move.set_start(**vars(position), timeout=STD_TIMEOUT))
                )
                # Wait for the move to be triggered.
                await asyncio.wait_for(self.csc.move_command_received_event.wait(), timeout=STD_TIMEOUT)

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
            self.assert_positions_close(self.csc.mock_ctrl.telemetry.commanded_pos, desired_position)

            # Do not test the controllerState event because it is
            # uncertain how many transitions will have occurred
            # during the consecutive moves.

    async def test_offset_no_compensation(self) -> None:
        """Test offset with compensation disabled."""
        first_uncompensated_position = mthexapod.Position(100, 200, -300, 0.01, 0.02, -0.015)
        offset = mthexapod.Position(50, -100, 135, 0.005, -0.005, 0.01)
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_initial_compensation_values()
            await self.check_offset(
                first_uncompensated_position=first_uncompensated_position,
                offset=offset,
                est_move_duration=1,
            )
            desired_uncompensated_position = np.add(first_uncompensated_position, offset)
            await self.assert_next_application(desired_position=desired_uncompensated_position)

    async def test_offset_with_compensation(self) -> None:
        """Test offset with compensation enabled."""
        first_uncompensated_position = mthexapod.Position(100, 200, -300, 0.01, 0.02, -0.015)
        offset = mthexapod.Position(50, -100, 135, 0.005, -0.005, 0.01)
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            await self.assert_initial_compensation_values()
            compensation_inputs_list = (
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=65, azimuth=44, rotation=-5, temperature=15),
            )
            await self.set_compensation_inputs(**vars(compensation_inputs_list[0]))

            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)
            await self.remote.cmd_setCompensationMode.set_start(enable=True, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=True)

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.check_offset(
                first_uncompensated_position=first_uncompensated_position,
                offset=offset,
                est_move_duration=1,
            )

            uncompensated_position = np.add(first_uncompensated_position, offset)

            update_inputs = False
            for compensation_inputs in compensation_inputs_list:
                with self.subTest(compensation_inputs=compensation_inputs):
                    await self.check_compensation(
                        uncompensated_position=uncompensated_position,
                        compensation_inputs=compensation_inputs,
                        update_inputs=update_inputs,
                    )
                    update_inputs = True

            # Make a tiny change to compensation inputs -- not enough
            # to trigger a compensation offset.
            old_comp_times = self.get_compensation_timestamps()
            new_compensation_inputs = copy.copy(compensation_inputs)
            new_compensation_inputs.elevation += 1e-7
            await self.set_compensation_inputs(**vars(new_compensation_inputs))
            sleep_time = self.csc.mock_ctrl.telemetry_interval * 5
            await asyncio.sleep(sleep_time)
            new_comp_times = self.get_compensation_timestamps()
            assert old_comp_times == new_comp_times

            # Stop the compensation loop task
            await self.remote.cmd_stop.set_start(timeout=STD_TIMEOUT)

    async def test_set_pivot(self) -> None:
        """Test the setPivot command."""
        axis_names = ("x", "y", "z")

        def get_pivot(config: types.SimpleNamespace) -> dict:
            return {name: getattr(config, f"pivot{name.upper()}") for name in axis_names}

        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            initial_config = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
            old_pivot = get_pivot(initial_config)

            self.remote.evt_configuration.flush()

            commanded_pivot = {name: val + 10 for name, val in old_pivot.items()}
            await self.remote.cmd_setPivot.set_start(**commanded_pivot, timeout=STD_TIMEOUT)

            new_config = await self.remote.evt_configuration.next(flush=False, timeout=STD_TIMEOUT)
            new_pivot = {name: getattr(new_config, f"pivot{name.upper()}") for name in axis_names}
            for name in axis_names:
                assert new_pivot[name] == pytest.approx(commanded_pivot[name])

    async def test_stop_move_after_delay(self) -> None:
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
            cmd_lengths = [actuator.end_position for actuator in self.csc.mock_ctrl.hexapod.actuators]
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
            stopped_lengths = [actuator.end_position for actuator in self.csc.mock_ctrl.hexapod.actuators]
            assert not np.allclose(cmd_lengths, stopped_lengths, atol=1e-7)

    async def test_stop_move_immediately(self) -> None:
        """Test that stop can interrupt a move right away."""
        position = copy.copy(ZERO_POSITION)
        position.z = 1000
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            self.csc.log.level = 10
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)

            await self.assert_next_application(desired_position=ZERO_POSITION)
            await self.assert_next_sample(
                self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            move_task = asyncio.create_task(
                self.remote.cmd_move.set_start(**vars(position), timeout=STD_TIMEOUT)
            )

            # Wait for the low-level controller to start processing the move
            t0 = utils.current_tai()
            position_tuple = dataclasses.astuple(position)
            while True:
                await asyncio.sleep(0.05)
                if np.allclose(self.csc.mock_ctrl.telemetry.commanded_pos, position_tuple):
                    break
                dt = utils.current_tai() - t0
                assert dt < 0.5, "Timed out waiting for commanded position to be updated"

            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            await asyncio.sleep(0)
            assert move_task.done()

            # Give the mock controller telemetry loop some time
            await asyncio.sleep(self.csc.mock_ctrl.telemetry_interval * 3)

            # Make sure the commanded position is still the last position
            self.assert_positions_close(self.csc.mock_ctrl.telemetry.commanded_pos, position)

            # Make sure the controller is stopped
            assert self.csc.mock_ctrl.telemetry.state == ControllerState.ENABLED
            assert self.csc.mock_ctrl.telemetry.enabled_substate == EnabledSubstate.STATIONARY

            # Do not test the controllerState event because it is
            # uncertain how many transitions will have occurred.

    async def test_filter_offset(self) -> None:
        initial_filter = "r_03"

        async with (
            self.cccamera_controller(initial_filter=initial_filter) as cccamera,
            self.make_csc(
                initial_state=salobj.State.ENABLED,
                override="with_filter_offset.yaml",
                simulation_mode=1,
            ),
        ):
            compensation_inputs = mthexapod.CompensationInputs(
                elevation=32, azimuth=44, rotation=-5, temperature=15
            )
            await self.set_compensation_inputs(**vars(compensation_inputs))

            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)
            await self.assert_initial_compensation_values()
            await self.remote.cmd_setCompensationMode.set_start(enable=True, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=True)

            await self.assert_next_application(desired_position=ZERO_POSITION)

            uncompensated_position = mthexapod.Position(500, -300, 200, 0.03, -0.02, 0.03)
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )

            # Test with the initial filter.
            update_inputs = False
            await self.check_compensation(
                uncompensated_position=uncompensated_position,
                compensation_inputs=compensation_inputs,
                update_inputs=update_inputs,
            )

            with open(TEST_CONFIG_DIR / "with_filter_offset.yaml") as fp:
                with_filter_offset_config = yaml.safe_load(fp.read())
                filter_offset = with_filter_offset_config["camera_config"]["filter_offsets"]
            # now test with the other available filters
            compensated_position_initial_filter = await self.remote.tel_application.next(
                flush=True, timeout=STD_TIMEOUT
            )

            for filter_name in ["i_06", "g_01", "u_02", "y_04", "z_03"]:
                with self.subTest(filter_name=filter_name):
                    self.remote.evt_compensatedPosition.flush()
                    self.remote.evt_inPosition.flush()
                    await cccamera.evt_endSetFilter.set_write(filterName=filter_name)
                    await self.assert_next_sample(self.remote.evt_compensatedPosition)

                    await self.assert_next_sample(
                        self.remote.evt_inPosition,
                        timeout=STD_TIMEOUT,
                        inPosition=False,
                    )
                    await self.assert_next_sample(
                        self.remote.evt_inPosition,
                        timeout=STD_TIMEOUT,
                        inPosition=True,
                    )

                    data = await self.remote.tel_application.next(flush=True, timeout=STD_TIMEOUT)
                    compensated_position_initial_filter.position[2] += filter_offset[filter_name]["z_offset"]
                    self.assert_positions_close(
                        compensated_position_initial_filter.position,
                        data.position,
                        pos_atol=1,
                        ang_atol=1e-5,
                    )
                    compensated_position_initial_filter.position[2] -= filter_offset[filter_name]["z_offset"]

            # Send a filter that is not in the list and check CSC goes to Fault
            self.remote.evt_summaryState.flush()
            await cccamera.evt_endSetFilter.set_write(filterName="NotAFilter")
            await self.assert_next_sample(
                self.remote.evt_summaryState,
                summaryState=State.FAULT,
                flush=False,
                timeout=STD_TIMEOUT,
            )

            # Enable the CSC and ensure filter offset still works.
            await cccamera.evt_endSetFilter.set_write(filterName=initial_filter)

            self.remote.evt_controllerState.flush()
            self.remote.evt_compensationOffset.flush()
            self.remote.evt_compensatedPosition.flush()
            self.remote.evt_uncompensatedPosition.flush()

            await salobj.set_summary_state(
                self.remote,
                salobj.State.ENABLED,
                override="with_filter_offset.yaml",
            )

            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)

            await self.assert_initial_compensation_values()

            await self.remote.cmd_setCompensationMode.set_start(enable=True, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=True)

            await self.assert_next_application(desired_position=ZERO_POSITION)

            uncompensated_position = mthexapod.Position(500, -300, 200, 0.03, -0.02, 0.03)
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )

            # Test with the initial filter.
            update_inputs = False
            await self.check_compensation(
                uncompensated_position=uncompensated_position,
                compensation_inputs=compensation_inputs,
                update_inputs=update_inputs,
            )

            # now test with the other available filters
            compensated_position_initial_filter = await self.remote.tel_application.next(
                flush=True, timeout=STD_TIMEOUT
            )

            for filter_name in ["i_06", "g_01", "u_02", "y_04", "z_03"]:
                with self.subTest(filter_name=filter_name):
                    self.remote.evt_compensatedPosition.flush()
                    self.remote.evt_inPosition.flush()
                    await cccamera.evt_endSetFilter.set_write(filterName=filter_name)
                    await self.assert_next_sample(self.remote.evt_compensatedPosition)

                    await self.assert_next_sample(
                        self.remote.evt_inPosition,
                        timeout=STD_TIMEOUT,
                        inPosition=False,
                    )
                    await self.assert_next_sample(
                        self.remote.evt_inPosition,
                        timeout=STD_TIMEOUT,
                        inPosition=True,
                    )

                    data = await self.remote.tel_application.next(flush=True, timeout=STD_TIMEOUT)
                    compensated_position_initial_filter.position[2] += filter_offset[filter_name]["z_offset"]
                    self.assert_positions_close(
                        compensated_position_initial_filter.position,
                        data.position,
                        pos_atol=1,
                        ang_atol=1e-5,
                    )
                    compensated_position_initial_filter.position[2] -= filter_offset[filter_name]["z_offset"]

    @contextlib.asynccontextmanager
    async def cccamera_controller(
        self, initial_filter: str
    ) -> typing.AsyncGenerator[salobj.Controller, None]:
        async with salobj.Controller("CCCamera") as cccamera:
            await cccamera.evt_endSetFilter.set_write(filterName=initial_filter)
            yield cccamera

    async def test_idle_time_monitor(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.ENABLED,
            override="",
            simulation_mode=1,
        ):
            # Flush the event
            self.remote.evt_controllerState.flush()

            # Task should be running
            assert self.csc.idle_time_monitor_task.done() is False

            # Controller shoudld be put into the idle after the timeout
            self.csc.config.camera_config["no_movement_idle_time"] = 9.0
            await asyncio.sleep(10.0)

            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.STANDBY,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )

            # Do the movement
            await self.remote.cmd_move.set_start(x=10)

            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )

            # The monitor task should be done after disabling the CSC
            await salobj.set_summary_state(self.remote, salobj.State.DISABLED)
            await asyncio.sleep(2.0)

            assert self.csc.idle_time_monitor_task.done() is True

    async def test_apply_compensation_while_exposing(self) -> None:
        initial_filter = "r_03"
        async with (
            self.cccamera_controller(initial_filter=initial_filter) as cccamera,
            self.make_csc(
                initial_state=salobj.State.ENABLED,
                override="",
                simulation_mode=1,
            ),
        ):
            await self.assert_initial_compensation_values()
            compensation_inputs_list = (
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=65, azimuth=44, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=32, azimuth=190, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=20, temperature=15),
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=-5, temperature=-30),
            )
            await self.set_compensation_inputs(**vars(compensation_inputs_list[0]))

            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)
            await self.remote.cmd_setCompensationMode.set_start(enable=True, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=True)

            await self.assert_next_application(desired_position=ZERO_POSITION)

            uncompensated_position = mthexapod.Position(500, -300, 200, 0.03, -0.02, 0.03)
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )

            update_inputs = False
            for shutter_detailed_state in mthexapod.ShutterDetailedState:
                await cccamera.evt_shutterDetailedState.set_write(substate=shutter_detailed_state)
                for compensation_inputs in compensation_inputs_list:
                    with self.subTest(compensation_inputs=compensation_inputs):
                        await self.check_compensation(
                            uncompensated_position=uncompensated_position,
                            compensation_inputs=compensation_inputs,
                            update_inputs=update_inputs,
                        )
                        update_inputs = True

            # Try finite inputs again; this should work.
            await self.check_compensation(
                uncompensated_position=uncompensated_position,
                compensation_inputs=compensation_inputs_list[-2],
                update_inputs=True,
            )

            # Test disabling compensation with setCompensationMode
            await self.remote.cmd_setCompensationMode.set_start(enable=False, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)

    async def test_no_compensation_while_exposing(self) -> None:
        initial_filter = "r_03"
        async with (
            self.cccamera_controller(initial_filter=initial_filter) as cccamera,
            self.make_csc(
                initial_state=salobj.State.ENABLED,
                override="no_compensation_while_exposing.yaml",
                simulation_mode=1,
            ),
        ):
            # Start with no shutter state to simulate the condition where
            # it could not retrieve historical data.
            await self.assert_initial_compensation_values()
            compensation_inputs_list = (
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=65, azimuth=44, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=32, azimuth=190, rotation=-5, temperature=15),
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=20, temperature=15),
                mthexapod.CompensationInputs(elevation=32, azimuth=44, rotation=-5, temperature=-30),
            )
            await self.set_compensation_inputs(**vars(compensation_inputs_list[0]))

            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)
            await self.remote.cmd_setCompensationMode.set_start(enable=True, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=True)

            await self.assert_next_application(desired_position=ZERO_POSITION)

            uncompensated_position = mthexapod.Position(500, -300, 200, 0.03, -0.02, 0.03)
            await self.check_move(
                uncompensated_position=uncompensated_position,
                est_move_duration=1,
            )

            for shutter_detailed_state in (
                mthexapod.enums.ShutterDetailedState.OPEN,
                mthexapod.enums.ShutterDetailedState.OPENING,
                mthexapod.enums.ShutterDetailedState.CLOSING,
            ):
                await cccamera.evt_shutterDetailedState.set_write(substate=shutter_detailed_state)
                update_inputs = False
                for compensation_inputs in compensation_inputs_list:
                    with self.subTest(compensation_inputs=compensation_inputs):
                        with pytest.raises(asyncio.TimeoutError):
                            self.remote.evt_compensationOffset.flush()
                            await self.check_compensation(
                                uncompensated_position=uncompensated_position,
                                compensation_inputs=compensation_inputs,
                                update_inputs=update_inputs,
                            )
                        update_inputs = True

            await cccamera.evt_shutterDetailedState.set_write(
                substate=mthexapod.enums.ShutterDetailedState.CLOSED
            )
            # should receive the initial compensation since we
            # started compensation with no filter information
            # and then immediately opened the shutter.
            self.assert_next_sample(topic=self.remote.evt_compensationOffset)

            # skip the first compensation because it is too small
            # compared to the last one.
            for compensation_inputs in compensation_inputs_list[1:]:
                with self.subTest(compensation_inputs=compensation_inputs):
                    self.remote.evt_compensationOffset.flush()
                    await self.check_compensation(
                        uncompensated_position=uncompensated_position,
                        compensation_inputs=compensation_inputs,
                        update_inputs=update_inputs,
                    )
                    update_inputs = True

            # Try finite inputs again; this should work.
            await self.check_compensation(
                uncompensated_position=uncompensated_position,
                compensation_inputs=compensation_inputs_list[-2],
                update_inputs=True,
            )

            # Test disabling compensation with setCompensationMode
            await self.remote.cmd_setCompensationMode.set_start(enable=False, timeout=STD_TIMEOUT)
            await self.assert_next_sample(topic=self.remote.evt_compensationMode, enabled=False)
