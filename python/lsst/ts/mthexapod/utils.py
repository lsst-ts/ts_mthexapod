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

__all__ = [
    "check_positive_value",
    "check_negative_value",
    "check_range",
    "check_symmetrical_range",
    "check_position",
    "check_new_position_limits",
    "rot2d",
    "rot_about_x",
    "rot_about_y",
    "rot_about_z",
    "get_next_position",
    "RAD_PER_DEG",
    "UM_TO_M",
]

import math
import typing

import numpy as np
import numpy.typing

from .base import Position, PositionLimits

RAD_PER_DEG = math.pi / 180
UM_TO_M = 1e-6


def check_positive_value(
    value: float,
    name: str,
    max_value: float,
    ExceptionClass: typing.Callable[[str], Exception] = ValueError,
) -> None:
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


def check_negative_value(
    value: float,
    name: str,
    min_value: float,
    ExceptionClass: typing.Callable[[str], Exception] = ValueError,
) -> None:
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


def check_range(
    value: float,
    name: str,
    min_value: float,
    max_value: float,
    ExceptionClass: typing.Callable[[str], Exception] = ValueError,
) -> None:
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


def check_symmetrical_range(
    value: float,
    name: str,
    max_value: float,
    ExceptionClass: typing.Callable[[str], Exception] = ValueError,
) -> None:
    """Check that a numeric value is in range -max_value <= value <= max_value.

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
    if not -max_value <= value <= max_value:
        raise ExceptionClass(f"{name}={value} not in range [-{max_value}, {max_value}]")


def check_position(
    position: Position,
    limits: PositionLimits,
    ExceptionClass: typing.Callable[[str], Exception] = ValueError,
) -> None:
    """Raise ExceptionClass if a position is not within limits.

    Parameters
    ----------
    position : `Position`
        Position to check.
    limits : `PositionLimits`
        Position limits.
    ExceptionClass : `Exception`, optional
        Exception class to raise.
    """
    check_symmetrical_range(
        value=position.x,
        name="x",
        max_value=limits.maxXY,
        ExceptionClass=ExceptionClass,
    )
    check_symmetrical_range(
        value=position.y,
        name="y",
        max_value=limits.maxXY,
        ExceptionClass=ExceptionClass,
    )
    check_range(
        value=position.z,
        name="z",
        min_value=limits.minZ,
        max_value=limits.maxZ,
        ExceptionClass=ExceptionClass,
    )
    check_symmetrical_range(
        value=position.u,
        name="u",
        max_value=limits.maxUV,
        ExceptionClass=ExceptionClass,
    )
    check_symmetrical_range(
        value=position.v,
        name="v",
        max_value=limits.maxUV,
        ExceptionClass=ExceptionClass,
    )
    check_range(
        value=position.w,
        name="w",
        min_value=limits.minW,
        max_value=limits.maxW,
        ExceptionClass=ExceptionClass,
    )


def check_new_position_limits(
    limits: PositionLimits,
    max_limits: PositionLimits,
    ExceptionClass: typing.Callable[[str], Exception] = ValueError,
) -> None:
    """Raise ExceptionClass if proposed new position limits are not with range
    of the maximum allowed position limits.
    """
    check_positive_value(
        value=limits.maxXY,
        name="maxXY",
        max_value=max_limits.maxXY,
        ExceptionClass=ExceptionClass,
    )
    check_negative_value(
        value=limits.minZ,
        name="minZ",
        min_value=max_limits.minZ,
        ExceptionClass=ExceptionClass,
    )
    check_positive_value(
        value=limits.maxZ,
        name="maxZ",
        max_value=max_limits.maxZ,
        ExceptionClass=ExceptionClass,
    )
    check_positive_value(
        value=limits.maxUV,
        name="maxUV",
        max_value=max_limits.maxUV,
        ExceptionClass=ExceptionClass,
    )
    check_negative_value(
        value=limits.minW,
        name="minW",
        min_value=max_limits.minW,
        ExceptionClass=ExceptionClass,
    )
    check_positive_value(
        value=limits.maxW,
        name="maxW",
        max_value=max_limits.maxW,
        ExceptionClass=ExceptionClass,
    )


def rot2d(xypos: tuple[float, float], ang: float) -> tuple[float, float]:
    """Rotate a 2-d position by the specified angle.

    Parameters
    ----------
    xypos : `tuple` [`float`]
        x, y position.
    ang : `float`
        Angle in radians.

    Returns
    -------
    rotated_xypos : `tuple` [`float`]
        Rotated x,y position.
    """
    x, y = xypos
    sinAng = math.sin(ang)
    cosAng = math.cos(ang)

    return (cosAng * x - sinAng * y, sinAng * x + cosAng * y)


def rot_about_x(xyzpos: numpy.typing.NDArray[np.float64], ang: float) -> numpy.typing.NDArray[np.float64]:
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


def rot_about_y(xyzpos: numpy.typing.NDArray[np.float64], ang: float) -> numpy.typing.NDArray[np.float64]:
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


def rot_about_z(xyzpos: tuple[float, float, float], ang: float) -> numpy.typing.NDArray[np.float64]:
    """Rotate a 3-d position about the z axis.

    Positive rotation is from x to y (the usual right-hand rule).

    Parameters
    ----------
    xyzpos : `tuple` [`float`]
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


def get_next_position(
    position_current: Position,
    position_target: Position,
    step_size_xy: float,
    step_size_z: float,
    step_size_uv: float,
    step_size_w: float,
) -> Position:
    """Get the next position.

    Parameters
    ----------
    position_current : `Position`
        Current position.
    position_target : `Position`
        Target position.
    step_size_xy : `float`
        Absolute maximum step size in the x, y direction in um. Put 0 if
        you want to do the movement in a single step.
    step_size_z : `float`
        Absolute maximum step size in the z direction in um. Put 0 if you
        want to do the movement in a single step.
    step_size_uv : `float`
        Absolute maximum step size in the rx, ry direction in deg. Put 0 if
        you want to do the movement in a single step.
    step_size_w : `float`
        Absolute maximum step size in the rz direction in deg. Put 0 if
        you want to do the movement in a single step.

    Returns
    -------
    position_next : `Position`
        Next position.
    """

    position_next = Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    position_diff = position_target - position_current
    limits = [
        step_size_xy,
        step_size_xy,
        step_size_z,
        step_size_uv,
        step_size_uv,
        step_size_w,
    ]
    for axis, limit in zip(position_current.field_names(), limits):
        diff = getattr(position_diff, axis)
        step = abs(limit) if (diff >= 0) else -abs(limit)

        if (step == 0.0) or (abs(diff) <= abs(step)):
            value = getattr(position_target, axis)
        else:
            value = getattr(position_current, axis) + step

        setattr(position_next, axis, value)

    return position_next
