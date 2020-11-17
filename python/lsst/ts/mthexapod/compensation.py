# This file is part of ts_mthexapod.
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

__all__ = ["Compensation"]

import numpy as np

from .ranged_polynomial import RangedPolynomial

NUM_AXES = 6  # x, y, z, u, v, w


class Compensation:
    """Compute hexapod compensation for elevation, azimuth, and temperature.

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
        If ``elevation_coeffs``, ``azimuth_coeffs`` or ``temperature_coeffs``
        is not a sequence of 6 items, or if any item is not a sequence
        of floats with at least 1 element.
    """

    def __init__(
        self, *, elevation_coeffs, temperature_coeffs, min_temperature, max_temperature,
    ):
        if len(elevation_coeffs) != NUM_AXES:
            raise ValueError(
                f"elevation_coeffs={elevation_coeffs} must be 6 lists of coefficients"
            )
        if len(temperature_coeffs) != NUM_AXES:
            raise ValueError(
                f"temperature_coeffs={temperature_coeffs} must be 6 lists of coefficients"
            )
        self.elevation_polys = [
            np.polynomial.Polynomial(elevation_coeffs[i]) for i in range(NUM_AXES)
        ]
        self.temperature_polys = [
            RangedPolynomial(
                coeffs=temperature_coeffs[i],
                min_x=min_temperature,
                max_x=max_temperature,
            )
            for i in range(NUM_AXES)
        ]

    def get_offsets(self, elevation, azimuth, temperature):
        """Get compensation offsets.

        Parameters
        ----------
        elevation : `float`
            Telescope elevation (deg). Must be in range [0, 90].
        azimuth : `float`
            Telescope azimuth (deg). There are no range limits;
            azimuth is wrapped as needed.
        temperature : `float`
            Ambient temperature (C). There are no range limits;
            see `RangedPolynomial` for details.

        Returns
        -------
        offsets : `list` [`float`]
            Offsets for x, y, z (um), u, v, w (deg), such that
            compensated value = nominal value + offsets.

        Raises
        ------
        ValueError
            If elevation not in range [0, 90].
        """
        if elevation < 0 or elevation > 90:
            raise ValueError(f"elevation={elevation} must be in range [0, 90]")
        return [
            self.elevation_polys[i](elevation) + self.temperature_polys[i](temperature)
            for i in range(NUM_AXES)
        ]
