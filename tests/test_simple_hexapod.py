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
import math
import unittest

import astropy.units as u
import numpy as np
import pytest
from lsst.ts import mthexapod, utils


class SimpleHexapodTestCase(unittest.IsolatedAsyncioTestCase):
    def setUp(self) -> None:
        np.random.seed(47)

    def test_constructor(self) -> None:
        max_length = 10e6  # big enough to not be a problem
        min_length = -max_length
        speed = 5e6

        base_positions = [np.random.normal(size=3) for i in range(6)]
        mirror_positions = [np.random.normal(size=3) for i in range(6)]
        pivot = np.random.normal(size=3)
        model = mthexapod.SimpleHexapod(
            base_positions=base_positions,
            mirror_positions=mirror_positions,
            pivot=pivot,
            min_length=min_length,
            max_length=max_length,
            speed=speed,
        )
        np.testing.assert_equal(model.base_positions, base_positions)
        np.testing.assert_equal(model.neutral_mirror_positions, mirror_positions)
        np.testing.assert_equal(model.neutral_pivot, pivot)
        np.testing.assert_equal(model.cmd_pos, np.zeros(3))
        np.testing.assert_equal(model.cmd_xyzrot, np.zeros(3))
        np.testing.assert_equal(model.cmd_mirror_positions, mirror_positions)
        for actuator in model.actuators:
            assert actuator.min_position == pytest.approx(min_length)
            assert actuator.max_position == pytest.approx(max_length)
            assert actuator.speed == pytest.approx(speed)
            assert actuator.position() == pytest.approx(0)
        relative_actuator_lengths = model.compute_actuator_lengths(
            mirror_positions, absolute=False
        )
        np.testing.assert_allclose(relative_actuator_lengths, np.zeros(6), atol=1e-7)
        absolute_actuator_lengths = model.compute_actuator_lengths(
            mirror_positions, absolute=True
        )
        np.testing.assert_allclose(
            absolute_actuator_lengths, model.neutral_actuator_lengths, atol=1e-7
        )

    def test_constructor_errors(self) -> None:
        # Use default position limits large enough to not be a problem
        max_length = 10e6
        min_length = -max_length
        speed = 5e6
        base_positions = [np.random.normal(size=3) for i in range(6)]
        # Use a large mean for the mirror positions normal distribution
        # so we can reliably test min_position and max_position
        mirror_positions = [np.random.normal(loc=1e6, size=3) for i in range(6)]
        pivot = np.random.normal(size=3)

        def bad_positions() -> tuple:
            return (
                [np.random.normal(size=3) for i in range(5)],
                [np.random.normal(size=3) for i in range(7)],
                [np.random.normal(size=3) for i in range(5)] + [(1, 2)],
            )

        for bad_base_positions in bad_positions():
            with pytest.raises(ValueError):
                mthexapod.SimpleHexapod(
                    base_positions=bad_base_positions,
                    mirror_positions=mirror_positions,
                    pivot=pivot,
                    min_length=min_length,
                    max_length=max_length,
                    speed=speed,
                )

        for bad_mirror_positions in bad_positions():
            with pytest.raises(ValueError):
                mthexapod.SimpleHexapod(
                    base_positions=base_positions,
                    mirror_positions=bad_mirror_positions,
                    pivot=pivot,
                    min_length=min_length,
                    max_length=max_length,
                    speed=speed,
                )

        for bad_pivot in ((1, 2), (1, 2, 3, 4)):
            with pytest.raises(ValueError):
                mthexapod.SimpleHexapod(
                    base_positions=base_positions,
                    mirror_positions=mirror_positions,
                    pivot=bad_pivot,
                    min_length=min_length,
                    max_length=max_length,
                    speed=speed,
                )

        for bad_min_length, bad_max_length in (
            (5, 5),  # illegal range
            (-1e99, -0.001),  # max too small
            (0.001, 2e99),  # min too big
        ):
            with pytest.raises(ValueError):
                mthexapod.SimpleHexapod(
                    base_positions=base_positions,
                    mirror_positions=mirror_positions,
                    pivot=pivot,
                    min_length=bad_min_length,
                    max_length=bad_max_length,
                    speed=speed,
                )
        for bad_speed in (0, -1):
            with pytest.raises(ValueError):
                mthexapod.SimpleHexapod(
                    base_positions=base_positions,
                    mirror_positions=mirror_positions,
                    pivot=pivot,
                    min_length=min_length,
                    max_length=max_length,
                    speed=bad_speed,
                )

    def test_make_zigzag_model(self) -> None:
        # Arbitrary but reasonable values. Lengths are in microns
        # because that is what the hexapod controller uses.
        base_radius = 1.2e6
        mirror_radius = 2.3e6
        mirror_z = 0.81e6
        pivot = (0, 0, 0.3e6)
        base_angle0 = 10
        max_length = 10e6  # big enough to not be a problem
        min_length = -max_length
        speed = 5e6

        model = mthexapod.SimpleHexapod.make_zigzag_model(
            base_radius=base_radius,
            mirror_radius=mirror_radius,
            mirror_z=mirror_z,
            base_angle0=base_angle0,
            pivot=pivot,
            min_length=min_length,
            max_length=max_length,
            speed=speed,
        )
        desired_base_angles = [
            base_angle0,
            base_angle0 + 120,
            base_angle0 + 120,
            base_angle0 + 240,
            base_angle0 + 240,
            base_angle0,
        ]
        for base_position, desired_base_angle in zip(
            model.base_positions, desired_base_angles
        ):
            assert base_position[2] == pytest.approx(0)
            meas_base_radius = math.hypot(base_position[0], base_position[1])
            assert meas_base_radius == pytest.approx(base_radius)
            base_angle = math.atan2(base_position[1], base_position[0]) * u.rad
            utils.assert_angles_almost_equal(base_angle, desired_base_angle)

        np.testing.assert_equal(model.base_positions[0], model.base_positions[5])
        np.testing.assert_equal(model.base_positions[1], model.base_positions[2])
        np.testing.assert_equal(model.base_positions[3], model.base_positions[4])

        mirror_angle0 = base_angle0 + 60
        desired_mirror_angles = [
            mirror_angle0,
            mirror_angle0,
            mirror_angle0 + 120,
            mirror_angle0 + 120,
            mirror_angle0 + 240,
            mirror_angle0 + 240,
        ]
        for mirror_position, desired_mirror_angle in zip(
            model.neutral_mirror_positions, desired_mirror_angles
        ):
            assert mirror_position[2] == pytest.approx(mirror_z)
            meas_mirror_radius = math.hypot(mirror_position[0], mirror_position[1])
            assert meas_mirror_radius == pytest.approx(mirror_radius)
            mirror_angle = math.atan2(mirror_position[1], mirror_position[0]) * u.rad
            utils.assert_angles_almost_equal(mirror_angle, desired_mirror_angle)
        np.testing.assert_equal(
            model.neutral_mirror_positions[0], model.neutral_mirror_positions[1]
        )
        np.testing.assert_equal(
            model.neutral_mirror_positions[2], model.neutral_mirror_positions[3]
        )
        np.testing.assert_equal(
            model.neutral_mirror_positions[4], model.neutral_mirror_positions[5]
        )

        for neutral_mirror_position, cmd_mirror_position in zip(
            model.neutral_mirror_positions, model.cmd_mirror_positions
        ):
            np.testing.assert_equal(neutral_mirror_position, cmd_mirror_position)

        np.testing.assert_equal(model.neutral_pivot, pivot)

        for actuator in model.actuators:
            assert actuator.min_position == pytest.approx(min_length)
            assert actuator.max_position == pytest.approx(max_length)
            assert actuator.speed == pytest.approx(speed)
            assert actuator.position() == pytest.approx(0)

        relative_actuator_lengths = model.compute_actuator_lengths(
            model.neutral_mirror_positions, absolute=False
        )
        np.testing.assert_allclose(relative_actuator_lengths, np.zeros(6), atol=1e-7)
        absolute_actuator_lengths = model.compute_actuator_lengths(
            model.neutral_mirror_positions, absolute=True
        )
        np.testing.assert_allclose(
            absolute_actuator_lengths, model.neutral_actuator_lengths, atol=1e-7
        )

    async def test_move_translate(self) -> None:
        # Arbitrary but reasonable values. Lengths are in microns
        # because that is what the hexapod controller uses.
        base_radius = 1.2e6
        mirror_radius = 2.3e6
        mirror_z = 0.81e6
        # Use an arbitrary pivot that is not centered to catch code
        # that assumes a centered pivot.
        pivot = (0.23e6, -4e6, 0.3e6)
        base_angle0 = 10
        max_length = 10e6  # big enough to not be a problem
        min_length = -max_length
        speed = 1e6  # make the moves go quickly

        model = mthexapod.SimpleHexapod.make_zigzag_model(
            base_radius=base_radius,
            mirror_radius=mirror_radius,
            mirror_z=mirror_z,
            base_angle0=base_angle0,
            pivot=pivot,
            min_length=min_length,
            max_length=max_length,
            speed=speed,
        )
        neutral_lengths = [actuator.position() for actuator in model.actuators]

        # A null move should not move anything.
        model.move((0, 0, 0), (0, 0, 0))
        np.testing.assert_allclose(model.cmd_pos, (0, 0, 0), atol=1e-7)
        np.testing.assert_allclose(model.cmd_xyzrot, (0, 0, 0), atol=1e-7)
        lengths = [actuator.position() for actuator in model.actuators]
        np.testing.assert_allclose(neutral_lengths, lengths, atol=1e-7)
        for cmd_mirror_position, neutral_mirror_position in zip(
            model.cmd_mirror_positions, model.neutral_mirror_positions
        ):
            np.testing.assert_allclose(
                cmd_mirror_position, neutral_mirror_position, atol=1e-7
            )

        # A translation should move the mirror end of all actuators
        # by the amount of the translation.
        translation = (500, -3020, 2500)
        model.move(translation, (0, 0, 0))
        await self.check_move(model)

        np.testing.assert_allclose(model.cmd_pos, translation, atol=1e-7)
        np.testing.assert_allclose(model.cmd_xyzrot, (0, 0, 0), atol=1e-7)
        for cmd_mirror_position, neutral_mirror_position in zip(
            model.cmd_mirror_positions, model.neutral_mirror_positions
        ):
            desired_mirror_position = neutral_mirror_position + translation
            np.testing.assert_allclose(
                cmd_mirror_position, desired_mirror_position, atol=1e-7
            )

    async def test_move_rotate_about_x_axis(self) -> None:
        await self.check_move_rotate_about_one_axis(axis=0)

    async def test_move_rotate_about_y_axis(self) -> None:
        await self.check_move_rotate_about_one_axis(axis=1)

    async def test_move_rotate_about_z_axis(self) -> None:
        await self.check_move_rotate_about_one_axis(axis=2)

    async def check_move_rotate_about_one_axis(self, axis: int) -> None:
        """Check translation and rotation about a single axis.

        Parameters
        ----------
        axis : `int`
            Axis about which to rotate; one of (0, 1, 2)
        """
        assert axis in (0, 1, 2)
        # Arbitrary but reasonable values. Lengths are in microns
        # because that is what the hexapod controller uses.
        base_radius = 2.1e6
        mirror_radius = 1.6e6
        mirror_z = 1.2e6
        # Use an arbitrary pivot that is not centered to catch code
        # that assumes a centered pivot.
        pivot = (-0.12e6, 0.2e6, -0.3e6)
        base_angle0 = -5
        max_length = 10e6  # big enough to not be a problem
        min_length = -max_length
        speed = 1e6  # make the moves go quickly

        model = mthexapod.SimpleHexapod.make_zigzag_model(
            base_radius=base_radius,
            mirror_radius=mirror_radius,
            mirror_z=mirror_z,
            base_angle0=base_angle0,
            pivot=pivot,
            min_length=min_length,
            max_length=max_length,
            speed=speed,
        )
        translation = (-400, 300, 1260)
        rot_angle = 5.1  # degrees
        rotation = np.roll((rot_angle, 0, 0), axis)
        model.move(translation, rotation)
        np.testing.assert_allclose(model.cmd_pos, translation, atol=1e-7)
        np.testing.assert_allclose(model.cmd_xyzrot, rotation, atol=1e-7)
        for cmd_mirror_position, neutral_mirror_position in zip(
            model.cmd_mirror_positions, model.neutral_mirror_positions
        ):
            # Compute position of mirror end relative to the pivot.
            # (This is not affected by translation).
            relative_unrotated_mirror_position = neutral_mirror_position - pivot
            # Compute position of translated and rotated mirror end
            # relative to the pivot.
            relative_mirror_position = cmd_mirror_position - np.add(translation, pivot)
            # Rotation does not affect the position along the specified axis
            assert relative_unrotated_mirror_position[axis] == pytest.approx(
                relative_mirror_position[axis]
            )
            # Rotation is about the translated pivot point.
            # Check that the vector from the pivot to each mirror point:
            # * Is not shifted along the axis of rotation by the rotation.
            # * Has the same length in the plane of rotation before and after
            #   rotation.
            # * Rotates by the expected amount in the plane of rotation.
            axis1 = (axis + 1) % 3
            axis2 = (axis + 2) % 3
            unrotated_len = math.hypot(
                relative_unrotated_mirror_position[axis1],
                relative_unrotated_mirror_position[axis2],
            )
            rotated_len = math.hypot(
                relative_mirror_position[axis1], relative_mirror_position[axis2]
            )
            assert unrotated_len == pytest.approx(rotated_len)
            unrotated_angle_rad = (
                math.atan2(
                    relative_unrotated_mirror_position[axis2],
                    relative_unrotated_mirror_position[axis1],
                )
                * u.rad
            )
            rotated_angle_rad = (
                math.atan2(
                    relative_mirror_position[axis2], relative_mirror_position[axis1]
                )
                * u.rad
            )
            delta_angle = rotated_angle_rad - unrotated_angle_rad
            utils.assert_angles_almost_equal(delta_angle, rot_angle)

        await self.check_move(model)

    async def check_move(self, model: mthexapod.SimpleHexapod) -> None:
        """Check the remaining_time and moving methods."""
        assert model.moving()
        margin = 0.02
        await asyncio.sleep(model.remaining_time() - margin)
        assert model.moving()
        await asyncio.sleep(margin * 2)
        assert not model.moving()
