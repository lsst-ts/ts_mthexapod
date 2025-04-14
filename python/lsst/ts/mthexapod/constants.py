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

__all__ = [
    "MAX_ACCEL_LIMIT",
    "MAX_LINEAR_VEL_LIMIT",
    "MAX_ANGULAR_VEL_LIMIT",
    "NUM_STRUT",
    "MAX_POSITION_LIMITS",
    "IndexControllerConstants",
    "ACTUATOR_BASE_POSITIONS_CAMERA",
    "ACTUATOR_MIRROR_POSITIONS_CAMERA",
    "ACTUATOR_BASE_POSITIONS_M2",
    "ACTUATOR_MIRROR_POSITIONS_M2",
    "PIVOT_CAMERA",
    "PIVOT_M2",
    "ACTUATOR_MAX_LENGTH",
    "ACTUATOR_MIN_LENGTH",
    "ACTUATOR_SPEED",
]

import dataclasses

import numpy as np

from . import base, enums

# Limits for the configureAcceleration command (deg/sec^2).
MAX_ACCEL_LIMIT = 500.0

# Limits for the velocitySet command (deg/sec).
MAX_LINEAR_VEL_LIMIT = 2000.0
MAX_ANGULAR_VEL_LIMIT = 0.1146

# 6 struts in the hexapod
NUM_STRUT = 6

# Position limits for the two hexapods
MAX_POSITION_LIMITS = {
    enums.SalIndex.CAMERA_HEXAPOD: base.PositionLimits(
        maxXY=11400,
        minZ=-13100,
        maxZ=13100,
        maxUV=0.36,
        minW=-0.10,
        maxW=0.10,
    ),
    enums.SalIndex.M2_HEXAPOD: base.PositionLimits(
        maxXY=10500,
        minZ=-8900,
        maxZ=8900,
        maxUV=0.175,
        minW=-0.05,
        maxW=0.05,
    ),
}


@dataclasses.dataclass
class ControllerConstants:
    """Constants needed to communicate with a low-level MTHexapod controller.

    Parameters
    ----------
    subconfig_name : `str`
        Name of device-specific configuration.
    """

    subconfig_name: str


# Dict of SalIndex: ControllerConstants
IndexControllerConstants = {
    enums.SalIndex.CAMERA_HEXAPOD: ControllerConstants(
        subconfig_name="camera_config",
    ),
    enums.SalIndex.M2_HEXAPOD: ControllerConstants(
        subconfig_name="m2_config",
    ),
}

# MTHexapod actuator positions (µm) from
# "CAMERA HEXAPOD STRUT FLEXURE COORDINATES.xlsx"
# (a copy is in the doc directory)
# received from John Andrew 2020-02-13.
# Need to consider the geometry and coordinate system of the hexapod.
ACTUATOR_BASE_POSITIONS_CAMERA = [
    np.array([227600, 653800, -525000]),
    np.array([-227600, 653800, -525000]),
    np.array([-680000, -129700, -525000]),
    np.array([-452300, -524000, -525000]),
    np.array([452300, -524000, -525000]),
    np.array([680000, -129700, -525000]),
]
ACTUATOR_MIRROR_POSITIONS_CAMERA = [
    np.array([472800, 512200, -121400]),
    np.array([-472800, 512200, -121400]),
    np.array([-680000, 153300, -121400]),
    np.array([-207200, -665500, -121400]),
    np.array([207200, -665500, -121400]),
    np.array([680000, 153300, -121400]),
]

ACTUATOR_BASE_POSITIONS_M2 = [
    np.array([0, 1701800, 114300]),
    np.array([-1542350, -719210, 114300]),
    np.array([1542350, -719210, 114300]),
    np.array([790, 1460490, 228600]),
    np.array([-1503370, 7660, 228600]),
    np.array([1503370, 7660, 228600]),
]
ACTUATOR_MIRROR_POSITIONS_M2 = [
    np.array([0, 1701800, 607300]),
    np.array([-1542350, -719210, 607300]),
    np.array([1542350, -719210, 607300]),
    np.array([493790, 1460490, 228600]),
    np.array([-1256870, -419290, 228600]),
    np.array([1256870, -419290, 228600]),
]

# Default pivot position (µm).
PIVOT_CAMERA = (0, 0, -2758400)
PIVOT_M2 = (0, 0, -703000)

# Actuator position limits (µm) and speed (µm/second) from
# https://github.com/lsst-ts/ts_mt_hexRot_middleware/blob/master/config/cam_hex/default.conf  # noqa
ACTUATOR_MAX_LENGTH = 14100
ACTUATOR_MIN_LENGTH = -14100
ACTUATOR_SPEED = 500
