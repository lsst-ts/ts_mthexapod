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

__all__ = ["RangedPolynomial"]

import numpy as np


class RangedPolynomial:
    """A polynomial that is linear outside a specified range.

    This model is suitable for systems which are mostly linear, with minor
    nonlinear corrections. The idea is to prevent unrealistic values outside
    the range of values over which the polynomial coefficients were fit.

    The equation is:
    ```
    y(x) = C0 + C1 x + C2 x^2 + ...   for min_x <= x <= max_x
    y(x) = y(min_y) + C1 (x - min_x)  for x < min_x
    y(x) = y(max_x) + C1 (x - max_x)  for x > max_x
    ```

    Parameters
    ----------
    coeffs : `list` [`float`]
        Polynomial coefficients C0, C1, ...
        Must contain at least one element.
    min_x : `float`
        Minimum x below which the result is linear.
    max_x : `float`
        Maximum x above which the result is linear.

    Raises
    ------
    ValueError
        If coeffs has no elements or min_x >= max_x.
    """

    def __init__(self, coeffs: list[float], min_x: float, max_x: float) -> None:
        if len(coeffs) < 1:
            raise ValueError(f"coeffs={coeffs} must contain at least one element")
        if min_x >= max_x:
            raise ValueError(f"min_x {min_x} >= max_x {max_x}")
        linear_coeff = coeffs[1] if len(coeffs) > 1 else 0
        self.coeffs = coeffs
        self.min_x = min_x
        self.max_x = max_x
        self._inrange_poly = np.polynomial.polynomial.Polynomial(self.coeffs)
        self._min_poly = np.polynomial.polynomial.Polynomial([self._inrange_poly(self.min_x), linear_coeff])
        self._max_poly = np.polynomial.polynomial.Polynomial([self._inrange_poly(self.max_x), linear_coeff])

    def __call__(self, x: float) -> float:
        """Compute the value of the function.

        Parameters
        ----------
        x : `float`
            Input value.
        """
        if self.min_x <= x <= self.max_x:
            return self._inrange_poly(x)
        elif x < self.min_x:
            return self._min_poly(x - self.min_x)
        return self._max_poly(x - self.max_x)
