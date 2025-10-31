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

import math
import unittest

import numpy as np
from lsst.ts import mthexapod


class UtilsTestCase(unittest.TestCase):
    def xyiter(self) -> tuple[tuple, ...]:
        """Return a list of xypos, angle (deg), desired rotated xypos."""
        return (
            ((1, 0), 0, (1, 0)),
            ((1, 0), 30, (0.5 * math.sqrt(3), 0.5)),
            ((1, 0), 90, (0, 1)),
            ((-1, 0), 90, (0, -1)),
            ((1, 0), 180, (-1, 0)),
            ((1, 0), -90, (0, -1)),
            ((0, 1), 0, (0, 1)),
            ((0, 1), 90, (-1, 0)),
            ((0, -1), 90, (1, 0)),
            ((0, 1), 180, (0, -1)),
            ((0, 1), -90, (1, 0)),
        )

    def test_rot2d(self) -> None:
        for xypos, angle, desired_rotxy in self.xyiter():
            with self.subTest(xypos=xypos, angle=angle):
                rotxy = mthexapod.rot2d(xypos, angle * mthexapod.RAD_PER_DEG)
                np.testing.assert_allclose(rotxy, desired_rotxy, atol=1e-10)

    def test_rot_about_x(self) -> None:
        # Positive rotation is y to z
        xpos = 1
        for (ypos, zpos), angle, (desired_roty, desired_rotz) in self.xyiter():
            pos = (xpos, ypos, zpos)
            desired_rotpos = (xpos, desired_roty, desired_rotz)
            with self.subTest(pos=pos, angle=angle):
                rotpos = mthexapod.rot_about_x(pos, angle * mthexapod.RAD_PER_DEG)
                np.testing.assert_allclose(rotpos, desired_rotpos, atol=1e-10)

    def test_rot_about_y(self) -> None:
        # Positive rotation is z to x
        ypos = 1
        for (zpos, xpos), angle, (desired_rotz, desired_rotx) in self.xyiter():
            pos = (xpos, ypos, zpos)
            desired_rotpos = (desired_rotx, ypos, desired_rotz)
            with self.subTest(pos=pos, angle=angle):
                rotpos = mthexapod.rot_about_y(pos, angle * mthexapod.RAD_PER_DEG)
                np.testing.assert_allclose(rotpos, desired_rotpos, atol=1e-10)

    def test_rot_about_z(self) -> None:
        # Positive rotation is x to y
        zpos = 1
        for (xpos, ypos), angle, (desired_rotx, desired_roty) in self.xyiter():
            pos = (xpos, ypos, zpos)
            desired_rotpos = (desired_rotx, desired_roty, zpos)
            with self.subTest(pos=pos, angle=angle):
                rotpos = mthexapod.rot_about_z(pos, angle * mthexapod.RAD_PER_DEG)
                np.testing.assert_allclose(rotpos, desired_rotpos, atol=1e-10)

    def test_get_next_position(self) -> None:
        position_current = mthexapod.Position(1.0, -3.0, 4.0, -10.0, 2.0, 3.0)
        position_target = mthexapod.Position(1.0, -10.0, 5.0, -7.0, -1.0, -2.5)

        # No step size, should go to the target directly
        position_next = mthexapod.get_next_position(position_current, position_target, 0.0, 0.0, 0.0, 0.0)

        assert position_next == position_target

        # Step size is too big, saturate to the target
        position_next = mthexapod.get_next_position(
            position_current, position_target, 100.0, 110.0, 120.0, 130.0
        )

        assert position_next == position_target

        # Normal condition
        position_next_1 = mthexapod.get_next_position(position_current, position_target, 3.0, 0.0, 1.0, 2.0)

        assert position_next_1 == mthexapod.Position(1.0, -6.0, 5.0, -9.0, 1.0, 1.0)

        position_next_2 = mthexapod.get_next_position(position_next_1, position_target, 3.0, 0.0, 1.0, 2.0)

        assert position_next_2 == mthexapod.Position(1.0, -9.0, 5.0, -8.0, 0.0, -1.0)

        position_next_3 = mthexapod.get_next_position(position_next_2, position_target, 3.0, 0.0, 1.0, 2.0)

        assert position_next_3 == position_target
