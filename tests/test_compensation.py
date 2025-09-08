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

import itertools
import unittest

import numpy as np
import pytest
from lsst.ts import mthexapod
from functools import partial
from numpy.polynomial.polynomial import polyval2d


class CompensationTestCase(unittest.TestCase):
    def test_constructor_errors(self) -> None:
        kwargs = dict(
            elevation_rotation_coeffs=[[
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]
            ]] * 6,
            azimuth_coeffs=[[0, 0]] * 6,
            temperature_coeffs=[[0, 0, 0, 0]] * 6,
            min_temperature=-20,
            max_temperature=20,
        )
        # Make sure these arguments work
        mthexapod.Compensation(**kwargs)

        # Omit an argument
        for name in kwargs:
            bad_kwargs = kwargs.copy()
            del bad_kwargs[name]
            with pytest.raises(TypeError):
                mthexapod.Compensation(**bad_kwargs)

        # Wrong number of coefficients
        for name, ncoeff in itertools.product(
            (
                "elevation_rotation_coeffs",
                "azimuth_coeffs",
                "temperature_coeffs",
            ),
            (0, 1, 5, 7, 8),
        ):
            bad_kwargs = kwargs.copy()
            bad_kwargs[name] = [[0]] * ncoeff
            with pytest.raises(ValueError):
                mthexapod.Compensation(**bad_kwargs)

        # min_temperature >= max_temperature
        for delta in (0, 0.001, 1):
            bad_kwargs = kwargs.copy()
            bad_kwargs["min_temperature"] = bad_kwargs["max_temperature"] + delta  # type: ignore[operator]
            with pytest.raises(ValueError):
                mthexapod.Compensation(**bad_kwargs)

    def test_get_offset(self) -> None:
        elevation_rotation_coeffs = [
            [
                [0.11, 0.12, 0.013], 
                [0.0014, 0.1, 0.0],
                [0.0, 0.0, 0.0],
            ],
            [0.21, 0.22, 0.023],
            [0.31, 0.32, -0.024],
            [0.41, 0.42],
            [0.51, 0.52],
            [0.61, 0.62],
        ]
        azimuth_coeffs = [
            [0.11, 0.12],
            [0.21, 0.22, -0.011],
            [0.31, 0.32, 0.013, 0.0014],
            [0.41, 0.42, 0.023],
            [0.51, 0.52],
            [0.61, 0.62],
        ]
        temperature_coeffs = [
            [0.11, -0.12, 0.13],
            [0.21, -0.22, 0.23],
            [0.31, -0.32, 0.33],
            [0.41, -0.42, 0.43],
            [0.51, -0.52, 0.53, 0.034],
            [0.61, -0.62, 0.63, 0.044, 0.0045],
        ]
        min_temperature = -20
        max_temperature = 25
        compensation = mthexapod.Compensation(
            elevation_rotation_coeffs=elevation_rotation_coeffs,
            azimuth_coeffs=azimuth_coeffs,
            temperature_coeffs=temperature_coeffs,
            min_temperature=min_temperature,
            max_temperature=max_temperature,
        )
        elevation_rotation_polynomials = [
            partial(polyval2d, coeffs=coeffs)
            for coeffs in elevation_rotation_coeffs
        ]
        azimuth_polynomials = [
            np.polynomial.Polynomial(coeffs) for coeffs in azimuth_coeffs
        ]
        temperature_polynomials = [
            mthexapod.RangedPolynomial(
                coeffs, min_x=min_temperature, max_x=max_temperature
            )
            for coeffs in temperature_coeffs
        ]
        for elevation, azimuth, rotation, temperature in itertools.product(
            (0, 42, 85, 90),
            (0, 33, 359.999),
            (0, -179.999, 179.999),
            (-50, min_temperature, 0, max_temperature, 70),
        ):
            comp_inputs = mthexapod.CompensationInputs(
                elevation=elevation,
                azimuth=azimuth,
                rotation=rotation,
                temperature=temperature,
            )
            offset = compensation.get_offset(comp_inputs)
            for i, (name, offset_value) in enumerate(vars(offset).items()):
                offset_list = [
                    elevation_rotation_polynomials[i](elevation, rotation),
                    azimuth_polynomials[i](azimuth),
                    temperature_polynomials[i](temperature),
                ]
                predicted_offset = sum(offset_list)
                assert offset_value == pytest.approx(predicted_offset)

        for bad_elevation in (-0.001, 90.001):
            with pytest.raises(ValueError):
                mthexapod.CompensationInputs(
                    elevation=bad_elevation,
                    azimuth=25,
                    rotation=5,
                    temperature=0,
                )
