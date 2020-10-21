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

import numpy as np

from lsst.ts import hexapod


class FourierSeriesTestCase(unittest.TestCase):
    def test_constructor_errors(self):
        # No coefficients
        with self.assertRaises(ValueError):
            hexapod.FourierSeries(coeffs=[])

    def test_values(self):
        coeffs = [-0.5, 0.4, -0.3, 0.2, 0.1, -0.15]
        poly = hexapod.FourierSeries(coeffs=coeffs)
        angarr = np.linspace(-360, 360, num=50, endpoint=True)
        pred_values = (
            coeffs[0]
            + coeffs[1] * np.sin(np.radians(angarr))
            + coeffs[2] * np.cos(np.radians(angarr))
            + coeffs[3] * np.sin(2 * np.radians(angarr))
            + coeffs[4] * np.cos(2 * np.radians(angarr))
            + coeffs[5] * np.sin(3 * np.radians(angarr))
        )
        np.testing.assert_allclose(poly(angarr), pred_values)
        for ang, pred_value in zip(angarr, pred_values):
            self.assertAlmostEqual(poly(ang), pred_value)

    def test_one_coeff(self):
        """Test that a fourier series with only one coffficient
        is constant everywhere.
        """
        coeffs = [1.3]
        poly = hexapod.FourierSeries(coeffs=coeffs)
        xarr = np.linspace(-360, 360, num=50)
        np.testing.assert_allclose(poly(xarr), coeffs[0])


if __name__ == "__main__":
    unittest.main()
