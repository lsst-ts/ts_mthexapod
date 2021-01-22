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

__all__ = ["FourierSeries"]

import itertools
import math

import numpy as np

RAD_PER_DEG = math.pi / 180.0


class FourierSeries:
    """A real-valued Fourier series.

    The equation is::

        f(angle) = C0 + C1 sin(angle) + C2 cos(angle)
                   + C3 sin(2 angle) + C4 cos(2 angle) + ...

    Parameters
    ----------
    coeffs : `list` [`float`]
        Polynomial coefficients C0, C1, ...
    """

    def __init__(self, coeffs):
        if len(coeffs) < 1:
            raise ValueError(f"coeffs={coeffs} must contain at least one element")
        self.coeffs = coeffs

        def return_one(data):
            return 1

        # Functions for C0, C1, C2, C3, ...
        function_iter = itertools.cycle([np.sin, np.cos])
        self.functions = [return_one] + [
            next(function_iter) for i in range(len(coeffs) - 1)
        ]
        # Angle multipliers for C0, C1, C2, C3
        self.angle_multipliers = [
            ((i + 1) // 2) * RAD_PER_DEG for i in range(len(coeffs))
        ]

    def __call__(self, angle):
        """Compute the value of the function.

        Parameters
        ----------
        angle : `float`
            Angle, in degrees.
        """
        result = 0
        for coeff, function, angle_multiplier in zip(
            self.coeffs, self.functions, self.angle_multipliers
        ):
            result += coeff * function(angle * angle_multiplier)
        return result
