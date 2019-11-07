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

__all__ = ["check_positive_value", "check_negative_value", "check_range", "check_symmetrical_range",
           "rot2d", "rot_about_x", "rot_about_y", "rot_about_z", "RAD_PER_DEG"]

import math

import numpy as np

RAD_PER_DEG = math.pi/180


def check_positive_value(value, name, max_value, ExceptionClass=ValueError):
    """Check that a numeric value is in range 0 < value <= max_value.

    Parameters
    ----------
    value : `float`
        Value to check.
    name : `str`
        Name of value.
    max_value : `float`
        Maximum allowed value of the named field (inclusive).
    ExceptionClass
        Class of exception to raise on error.
    """
    if not 0 < value <= max_value:
        raise ExceptionClass(f"{name}={value} not in range [0, {max_value}]")


def check_negative_value(value, name, min_value, ExceptionClass=ValueError):
    """Check that a numeric value is in range min_value <= value < 0.

    Parameters
    ----------
    value : `float`
        Value to check.
    name : `str`
        Name of value.
    min_value : `float`
        Minimum allowed value of the named field (inclusive).
    ExceptionClass
        Class of exception to raise on error.
    """
    if not min_value <= value < 0:
        raise ExceptionClass(f"{name}={value} not in range [{min_value}, 0]")


def check_range(value, name, min_value, max_value, ExceptionClass=ValueError):
    """Check that a numeric value is range min_value <= value <= max_value.

    Parameters
    ----------
    value : `float`
        Value to check.
    name : `str`
        Name of value.
    min_value : `float`
        Minimum allowed value of the named field (inclusive).
    max_value : `float`
        Maximum allowed value of the named field (inclusive).
    ExceptionClass
        Class of exception to raise on error.
    """
    if value < min_value or value > max_value:
        raise ExceptionClass(f"{name}={value} not in range [{min_value}, {max_value}]")


def check_symmetrical_range(value, name, max_value, ExceptionClass=ValueError):
    """Check that a numeric value is in range -max_value <= value <= max_value.

    Parameters
    ----------
    value : `float`
        Value to check.
    name : `str`
        Name of value.
    name : `str`
        Name of ``data`` field to check.
    max_value : `float`
        Maximum allowed value of the named field (inclusive).
    ExceptionClass
        Class of exception to raise on error.
    """
    if not -max_value <= value <= max_value:
        raise ExceptionClass(f"{name}={value} not in range [-{max_value}, {max_value}]")


def rot2d(xypos, ang):
    """Rotate a 2-d position by the specified angle.

    Parameters
    ----------
    xypos : `List` [`float`]
        x, y position.
    ang : `float`
        Angle in radians.

    Returns
    -------
    rotated_xypos : `List` [`float`]
        Rotated x,y position.
    """
    x, y = xypos
    sinAng = math.sin(ang)
    cosAng = math.cos(ang)

    return (cosAng * x - sinAng * y,
            sinAng * x + cosAng * y)


def rot_about_x(xyzpos, ang):
    """Rotate a 3-d position about the x axis.

    Positive rotation is from y to z (the usual right-hand rule).

    Parameters
    ----------
    xyzpos : `numpy.ndarray`
        x, y, z position.
    ang : `float`
        Angle in radians.

    Returns
    -------
    rotpos : `numpy.ndarray`
        Rotated x,y,z position.
    """
    x, y, z = xyzpos
    roty, rotz = rot2d((y, z), ang)
    return np.array((x, roty, rotz), dtype=float)


def rot_about_y(xyzpos, ang):
    """Rotate a 3-d position about the y axis.

    Positive rotation is from from z to x (the usual right-hand rule).

    Parameters
    ----------
    xyzpos : `numpy.ndarray`
        x, y, z position.
    ang : `float`
        Angle in radians.

    Returns
    -------
    rotpos : `numpy.ndarray`
        Rotated x,y,z position.
    """
    x, y, z = xyzpos
    rotz, rotx = rot2d((z, x), ang)
    return np.array((rotx, y, rotz), dtype=float)


def rot_about_z(xyzpos, ang):
    """Rotate a 3-d position about the z axis.

    Positive rotation is from x to y (the usual right-hand rule).

    Parameters
    ----------
    xyzpos : `List` [`float`]
        x, y, z position.
    ang : `float`
        Angle in radians.

    Returns
    -------
    rotpos : `numpy.ndarray`
        Rotated x,y,z position.
    """
    x, y, z = xyzpos
    rotx, roty = rot2d((x, y), ang)
    return np.array((rotx, roty, z), dtype=float)
