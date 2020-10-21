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

import itertools
import unittest

import numpy as np

from lsst.ts import hexapod


class CompensationTestCase(unittest.TestCase):
    def test_constructor_errors(self):
        kwargs = dict(
            elevation_coeffs=[[0]] * 6,
            temperature_coeffs=[[0, 0]] * 6,
            min_temperature=-20,
            max_temperature=20,
        )
        # Make sure these arguments work
        hexapod.Compensation(**kwargs)

        # Omit an argument
        for name in kwargs:
            bad_kwargs = kwargs.copy()
            del bad_kwargs[name]
            with self.assertRaises(TypeError):
                hexapod.Compensation(**bad_kwargs)

        # Wrong number of coefficients
        for name, ncoeff in itertools.product(
            ("elevation_coeffs", "temperature_coeffs"), (0, 1, 5, 7, 8),
        ):
            bad_kwargs = kwargs.copy()
            bad_kwargs[name] = [[0]] * ncoeff
            with self.assertRaises(ValueError):
                hexapod.Compensation(**bad_kwargs)

        # min_temperature >= max_temperature
        for delta in (0, 0.001, 1):
            bad_kwargs = kwargs.copy()
            bad_kwargs["min_temperature"] = bad_kwargs["max_temperature"] + delta
            with self.assertRaises(ValueError):
                hexapod.Compensation(**bad_kwargs)

    def test_get_offsets(self):
        elevation_coeffs = [
            [0.11, 0.12, 0.013, 0.0014],
            [0.21, 0.22, 0.023],
            [0.31, 0.32],
            [0.41, 0.42],
            [0.51, 0.52],
            [0.61, 0.62],
        ]
        temperature_coeffs = [
            [0.11, -0.12, 0.13],
            [0.21, -0.22, 0.23],
            [0.31, -0.32, 0.33, 0.034],
            [0.41, -0.42, 0.43, 0.044, 0.0045],
            [0.51, -0.52, 0.53],
            [0.61, -0.62, 0.63],
        ]
        min_temperature = -20
        max_temperature = 25
        compensation = hexapod.Compensation(
            elevation_coeffs=elevation_coeffs,
            temperature_coeffs=temperature_coeffs,
            min_temperature=min_temperature,
            max_temperature=max_temperature,
        )
        elevation_polynomials = [
            np.polynomial.Polynomial(coeffs) for coeffs in elevation_coeffs
        ]
        temperature_polynomials = [
            hexapod.RangedPolynomial(
                coeffs, min_x=min_temperature, max_x=max_temperature
            )
            for coeffs in temperature_coeffs
        ]
        for azimuth, elevation, temperature in itertools.product(
            (0, 33, -50, 375),
            (0, 42, 85, 90),
            (-50, min_temperature, 0, max_temperature, 70),
        ):
            offset = compensation.get_offsets(
                elevation=elevation, azimuth=azimuth, temperature=temperature,
            )
            desired_offset = [
                elevation_polynomials[i](elevation)
                + temperature_polynomials[i](temperature)
                for i in range(6)
            ]
            np.testing.assert_allclose(offset, desired_offset)

        for bad_elevation in (-0.001, 90.001):
            with self.assertRaises(ValueError):
                compensation.get_offsets(
                    elevation=bad_elevation, azimuth=25, temperature=0,
                )


if __name__ == "__main__":
    unittest.main()
