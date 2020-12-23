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
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

__all__ = [
    "CAM_SYNC_PATTERN",
    "M2_SYNC_PATTERN",
    "MAX_ACCEL_LIMIT",
    "MAX_LINEAR_VEL_LIMIT",
    "MAX_ANGULAR_VEL_LIMIT",
    "MAX_POSITION_LIMITS",
]

from . import base
from . import enums

# Required value of command.sync_pattern for MTHexapod commands.
CAM_SYNC_PATTERN = 0x6666
M2_SYNC_PATTERN = 0xB4B4

# Limits for the configureAcceleration command (deg/sec^2).
MAX_ACCEL_LIMIT = 500.0

# Limits for the velocitySet command (deg/sec).
MAX_LINEAR_VEL_LIMIT = 2000.0
MAX_ANGULAR_VEL_LIMIT = 0.1146

# Position limits for the two hexapods
MAX_POSITION_LIMITS = {
    enums.SalIndex.CAMERA_HEXAPOD: base.PositionLimits(
        maxXY=11400, minZ=-13100, maxZ=13100, maxUV=0.36, minW=-0.10, maxW=0.10,
    ),
    enums.SalIndex.M2_HEXAPOD: base.PositionLimits(
        maxXY=10500, minZ=-8900, maxZ=8900, maxUV=0.175, minW=-0.05, maxW=0.05,
    ),
}
