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

__all__ = ["Compensation"]

import numpy as np

from . import base
from functools import partial
from .ranged_polynomial import RangedPolynomial

NUM_AXES = 6  # x, y, z, u, v, w


class Compensation:
    """Compute hexapod compensation for elevation, azimuth, camera rotation,
    and temperature.

    The compensation is computed as offsets for x, y, z, u, v, w.

    Parameters
    ----------
    elevation_coeffs : `list` [`list` [`float`]]
        Elevation `CosinePolynomial` coefficients,
        as a sequence of 6 coefficient sequences, for x, y, z, u, v, w.
        Each coefficient sequence must contain at least one element.
        Here is an example showing valid, though unrealistic, values::

            elevation_coeffs=[
                [0.11, 0.012],
                [0.21],
                [0.31, 0.032, -0.0033],
                [0.000042, 0.000042],
                [0.000052, 0.000052],
                [0.000062],
            ]

    azimuth_coeffs  : `list` [`list` [`float`]]
        Azimuth coefficients, with the same format as ``elevation_coeffs``.
    rotation_coeffs  : `list` [`list` [`float`]]
        Rotation coefficients, with the same format as ``elevation_coeffs``.
    temperature_coeffs  : `list` [`list` [`float`]]
        Temperature `RangedPolynomial` coefficients, with the same format
        as ``elevation_coeffs``.
    min_temperature : `float`
        Minimum temperature for which ``temperature_coeffs`` is valid.
    max_temperature : `float`
        Maximum temperature for which ``temperature_coeffs`` is valid.

    Raises
    ------
    ValueError
        If ``elevation_coeffs``, ``azimuth_coeffs``, ``rotation_coeffs``,
        or ``temperature_coeffs`` is not a sequence of 6 items,
        or if any item is not a sequence of floats with at least 1 element.
    """

    def __init__(
        self,
        *,
        elevation_rotation_coeffs: list[list[[list[float]]]],
        azimuth_coeffs: list[list[float]],
        temperature_coeffs: list[list[float]],
        min_temperature: float,
        max_temperature: float,
    ) -> None:
        for name, coeffs in (
            ("elevation_rotation", elevation_rotation_coeffs),
            ("azimuth", azimuth_coeffs),
            ("temperature", temperature_coeffs),
        ):
            if len(coeffs) != 6:
                raise ValueError(f"{name}={coeffs} must be 6 lists of coefficients")

        self.elevation_rotation_polys = [
            partial(polyval2d, coeffs=elevation_rotation_coeffs[i])
            for i in range(NUM_AXES)
        ]
        self.azimuth_polys = [np.polynomial.Polynomial(azimuth_coeffs[i]) for i in range(NUM_AXES)]
        self.temperature_polys = [
            RangedPolynomial(
                coeffs=temperature_coeffs[i],
                min_x=min_temperature,
                max_x=max_temperature,
            )
            for i in range(NUM_AXES)
        ]

    def get_offset(self, inputs: base.CompensationInputs) -> base.Position:
        """Get compensation offset.

        Parameters
        ----------
        inputs : `CompensationInputs`
            Inputs for the compensation model.

        Returns
        -------
        offset : `Position`
            Compensation offsets, such that::

                compensated position = uncompensated position + offset.

        Raises
        ------
        ValueError
            If elevation not in range [0, 90].
        """

        offsets = [
            self.elevation_rotation_polys[i](inputs.elevation, inputs.rotation)  # type: ignore[attr-defined]
            + self.azimuth_polys[i](inputs.azimuth)  # type: ignore[attr-defined]
            + self.temperature_polys[i](inputs.temperature)  # type: ignore[attr-defined]
            for i in range(NUM_AXES)
        ]
        return base.Position(*offsets)
