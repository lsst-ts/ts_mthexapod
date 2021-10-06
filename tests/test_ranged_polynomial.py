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

import unittest

import numpy as np
import pytest

from lsst.ts import mthexapod


class RangedPolynomialTestCase(unittest.TestCase):
    def test_constructor_errors(self):
        # No coefficients
        with pytest.raises(ValueError):
            mthexapod.RangedPolynomial(coeffs=[], min_x=0, max_x=1)

        # Invalid range (min_x > max_x):
        for min_x in (-100, -0.001, 0, 0.001, 100):
            for dx in (-10, -0.001):
                bad_max_x = min_x + dx
                with pytest.raises(ValueError):
                    mthexapod.RangedPolynomial(coeffs=[0], min_x=min_x, max_x=bad_max_x)

    def test_values(self):
        coeffs = [-0.5, 0.4, -0.3, 0.2]
        min_x = -10
        max_x = 10
        poly = mthexapod.RangedPolynomial(coeffs=coeffs, min_x=min_x, max_x=max_x)
        for x in np.linspace(min_x, max_x, num=10, endpoint=True):
            pred_value = coeffs[0] + x * (coeffs[1] + x * (coeffs[2] + x * coeffs[3]))
            self.assertAlmostEqual(poly(x), pred_value)

        # Check that there is a smooth transition at the borders
        self.assertAlmostEqual(poly(min_x - 1e-5), poly(min_x + 1e-5), delta=1e-3)
        self.assertAlmostEqual(poly(max_x + 1e-5), poly(max_x - 1e-5), delta=1e-3)

        # Check that the out-of-range values are linear
        for x in np.linspace(min_x - 10, min_x, num=10, endpoint=True):
            pred_value = poly(min_x) + (x - min_x) * coeffs[1]
            self.assertAlmostEqual(poly(x), pred_value)

        for x in np.linspace(max_x, max_x + 10, num=10, endpoint=True):
            pred_value = poly(max_x) + (x - max_x) * coeffs[1]
            self.assertAlmostEqual(poly(x), pred_value)

    def test_one_coeff(self):
        """Test that a ranged polynomial with only one coffficient
        is constant everywhere.
        """
        coeffs = [1.3]
        min_x = -10
        max_x = 10
        poly = mthexapod.RangedPolynomial(coeffs=coeffs, min_x=min_x, max_x=max_x)
        for x in (min_x - 1, min_x, 0, max_x, max_x + 1):
            self.assertAlmostEqual(poly(x), coeffs[0])
