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
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import ctypes
import enum


ELEVATION_ELEMENTS = 19
AZIMUTH_ELEMENTS = 37
TEMPERATURE_ELEMENTS = 9

CAM_ERROR_CODE = 6201
M2_ERROR_CODE = 6151

# what is this?
HEXAPOD_DDS_TLM_CNT = 392


class CommandType(enum.IntEnum):
    # why trigger and cmd (same value)?
    STATE_TRIGGER = 0x8000
    STATE_CMD = 0x8000
    ENABLED_SUBSTATE_CMD = 0x8001
    OFFLINE_TRIGGER = 0x8002
    POS_CMD = 0x8004
    SET_PIVOTPOINT = 0x8007
    CONFIG_ACCEL = 0x800B
    CONFIG_VEL = 0x800C
    CONFIG_LIMITS = 0x800D
    OFFSET = 0x8010
    # is this relevant to a hexapod?
    TRACK_VEL_CMD = 0x9031


class FrameId(enum.IntEnum):
    CAM_TELEMETRY = 7
    M2_TELEMETRY = 8
    CAM_CONFIG = 27
    M2_CONFIG = 28


class Command(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("sync_pattern", ctypes.c_ushort),
        ("counter", ctypes.c_ushort),
        ("cmd", ctypes.c_uint),
        ("param1", ctypes.double),
        ("param2", ctypes.double),
        ("param3", ctypes.double),
        ("param4", ctypes.double),
        ("param5", ctypes.double),
        ("param6", ctypes.double),
    ]


class Header(ctypes.Structure):
    """Header for output from MT Hexapod or Rotator.
    """
    _pack_ = 1
    _fields_ = [
        ("sync_pattern", ctypes.c_ushort),
        ("frame_id", ctypes.c_ushort),
        ("counter", ctypes.c_ushort),
        ("mjd", ctypes.c_int),
        ("mjd_frac", ctypes.c_double),
        # tv_sec is time_t in C
        ("tv_sec", ctypes.c_int64),
        ("tv_nsec", ctypes.c_long),
    ]


class Config(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("strut_acceleration", ctypes.c_double),
        ("pos_limits", ctypes.c_double * 6),
        ("vel_limits", ctypes.c_double * 6),
        ("initial_pos", ctypes.c_double * 6),
        ("pivot", ctypes.c_double * 3),
        ("el_lut_index", ctypes.c_double * ELEVATION_ELEMENTS),
        ("el_lut_x", ctypes.c_double * ELEVATION_ELEMENTS),
        ("el_lut_y", ctypes.c_double * ELEVATION_ELEMENTS),
        ("el_lut_z", ctypes.c_double * ELEVATION_ELEMENTS),
        ("el_lut_rx", ctypes.c_double * ELEVATION_ELEMENTS),
        ("el_lut_ry", ctypes.c_double * ELEVATION_ELEMENTS),
        ("el_lut_rz", ctypes.c_double * ELEVATION_ELEMENTS),
        ("az_lut_index", ctypes.c_double * AZIMUTH_ELEMENTS),
        ("az_lut_x", ctypes.c_double * AZIMUTH_ELEMENTS),
        ("az_lut_y", ctypes.c_double * AZIMUTH_ELEMENTS),
        ("az_lut_z", ctypes.c_double * AZIMUTH_ELEMENTS),
        ("az_lut_rx", ctypes.c_double * AZIMUTH_ELEMENTS),
        ("az_lut_ry", ctypes.c_double * AZIMUTH_ELEMENTS),
        ("az_lut_rz", ctypes.c_double * AZIMUTH_ELEMENTS),
        ("temp_lut_index", ctypes.c_double * TEMPERATURE_ELEMENTS),
        ("temp_lut_x", ctypes.c_double * TEMPERATURE_ELEMENTS),
        ("temp_lut_y", ctypes.c_double * TEMPERATURE_ELEMENTS),
        ("temp_lut_z", ctypes.c_double * TEMPERATURE_ELEMENTS),
        ("temp_lut_rx", ctypes.c_double * TEMPERATURE_ELEMENTS),
        ("temp_lut_ry", ctypes.c_double * TEMPERATURE_ELEMENTS),
        ("temp_lut_rz", ctypes.c_double * TEMPERATURE_ELEMENTS),
        ("strut_displacement_max", ctypes.c_double),
        ("strut_velocity_max", ctypes.c_double),
    ]


class MainTelemetry(ctypes.Structure):
    """Hexapod telemetry.
    """
    _pack_ = 1
    _fields_ = [
        ("status_word", ctypes.c_uint16 * 6),
        ("latching_fault_status_register", ctypes.c_uint16 * 6),
        ("copley_fault_status_register", ctypes.c_uint32 * 6),
        ("application_status", ctypes.c_uint16 * 6),
        ("input_pin_states", ctypes.c_uint32 * 3),
    ]


class SimulinkTelemetry(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("state", ctypes.c_double),
        ("enabled_substate", ctypes.c_double),
        ("offline_substate", ctypes.c_double),
        ("test_state", ctypes.c_double),
        ("strut_encoder_raw", ctypes.c_double * 6),
        ("strut_encoder_microns", ctypes.c_double * 6),
        # commanded displacement (microns) and rotation (deg)
        ("cmd_pos", ctypes.c_double * 6),
        # measured displacement (microns) and rotation (deg)
        ("meas_pos", ctypes.c_double * 6),
        # commanded strut lengths (microns)
        ("cmd_length", ctypes.c_double * 6),
    ]


class Telemetry(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("main", MainTelemetry),
        ("simulink", SimulinkTelemetry),
    ]
