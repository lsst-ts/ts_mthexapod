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

__all__ = ["SalIndex", "CommandCode", "SetEnabledSubstateParam"]

import enum


class SalIndex(enum.IntEnum):
    CAMERA_HEXAPOD = 1
    M2_HEXAPOD = 2


class CommandCode(enum.IntEnum):
    """Values for the ``Command.code`` field.

    In the low-level controller code these are defined in
    enum ``cmdType``. I have reworded them for clarity.
    """

    ENABLE_DRIVES = 0x7000
    SET_STATE = 0x8000
    SET_ENABLED_SUBSTATE = 0x8001
    POSITION_SET = 0x8004
    SET_PIVOTPOINT = 0x8007
    CONFIG_ACCEL = 0x800B
    CONFIG_VEL = 0x800C
    CONFIG_LIMITS = 0x800D
    OFFSET = 0x8010


class SetEnabledSubstateParam(enum.IntEnum):
    """Substates for the ENABLED state."""

    ENABLED_INVALID = 0
    MOVE_POINT_TO_POINT = 1
    STOP = 3
