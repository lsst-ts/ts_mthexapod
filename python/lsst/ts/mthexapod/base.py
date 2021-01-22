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
    "CompensationInputs",
    "Position",
    "PositionLimits",
]

import dataclasses

from lsst.ts import salobj


@dataclasses.dataclass
class CompensationInputs:
    """Input values for the compensation model.

    Parameters
    ----------
    elevation : `float`
        Telescope elevation (deg). Must be in range [0, 90].
    azimuth : `float`
        Telescope azimuth (deg). Wrapped to [0, 360).
    rotation : `float`
        Camera rotation angle (deg). Wrapped to [-180, 180].
    temperature : `float`
        Ambient temperature (C). The range is not constrained,
        but compensation is constant outside a given range;
        see `RangedPolynomial` for details.

    Raises
    ------
    ValueError
        If elevation not in range [0, 90].
    """

    elevation: float
    azimuth: float
    rotation: float
    temperature: float

    def __post_init__(self):
        if self.elevation < 0 or self.elevation > 90:
            raise ValueError(f"elevation={self.elevation} must be in range [0, 90]")
        self.azimuth = salobj.angle_wrap_nonnegative(self.azimuth).deg
        self.rotation = salobj.angle_wrap_center(self.rotation).deg

    @classmethod
    def field_names(cls):
        return tuple(field.name for field in dataclasses.fields(cls))

    @classmethod
    def from_struct(cls, data):
        """Construct an instance from any object with fields:
        maxXY, minZ, maxZ, maxUV, minW, maxW
        """
        return cls(**{name: getattr(data, name) for name in cls.field_names()})


@dataclasses.dataclass
class Position:
    """The position and orientation of the hexapod pivot point.

    This class may also be used for offsets.

    The order of the fields matches the order of parameters in the low-level
    controller command to set position.

    Parameters
    ----------
    x, y, z : `float`
        x, y, z position (um)
    u, v, w : `float`
        Rotation about x, y, z (deg)
    """

    x: float
    y: float
    z: float
    u: float
    v: float
    w: float

    @classmethod
    def field_names(cls):
        return tuple(field.name for field in dataclasses.fields(cls))

    @classmethod
    def from_struct(cls, data):
        """Construct an instance from any object with fields:
        x, y, z, u, v, w.
        """
        return cls(**{name: getattr(data, name) for name in cls.field_names()})

    def __add__(self, other):
        kwargs = {
            name: getattr(self, name) + getattr(other, name)
            for name in self.field_names()
        }
        return Position(**kwargs)

    def __sub__(self, other):
        kwargs = {
            name: getattr(self, name) - getattr(other, name)
            for name in self.field_names()
        }
        return Position(**kwargs)


@dataclasses.dataclass
class PositionLimits:
    """Position limits.

    The field names match those used in the SAL MTHexapod configureLimits
    command and config event. The order of the fields matches the order
    of parameters in the low-level controller command to configure limits.

    Parameters
    ----------
    maxXY : `float`
        Maximum absolute value of x and y, i.e. symmetrical about 0 (um).
    minZ : `float`
        Minimum value of z (um)
    maxZ : `float`
        Maximum value of z (um)
    maxUV : `float`
        Maximum absolute value of u and v, i.e. symmetrical about 0 (deg).
    minW : `float`
        Minimum value of w (deg)
    maxW : `float`
        Maximum value of w (deg)
    ExceptionClass : `Exception`, optional
        Exception class to raise.

    Raises
    ------
    ExceptionClass
        If maxXY <= 0, maxZ <= minZ, maxUV <= 0, or maxW <= minW.
    """

    maxXY: float
    minZ: float
    maxZ: float
    maxUV: float
    minW: float
    maxW: float

    def __post_init__(self):
        if self.maxXY <= 0:
            raise ValueError(f"maxXY={self.maxXY} must be positive.")
        if self.maxZ <= self.minZ:
            raise ValueError(f"maxZ={self.maxZ} must be >= minZ={self.minZ}.")
        if self.maxUV <= 0:
            raise ValueError(f"maxUV={self.maxUV} must be positive.")
        if self.maxW <= self.minW:
            raise ValueError(f"maxW={self.maxW} must be >= minW={self.minW}.")

    @classmethod
    def field_names(cls):
        return tuple(field.name for field in dataclasses.fields(cls))

    @classmethod
    def from_struct(cls, data):
        """Construct an instance from any object with fields:
        maxXY, minZ, maxZ, maxUV, minW, maxW
        """
        return cls(**{name: getattr(data, name) for name in cls.field_names()})
