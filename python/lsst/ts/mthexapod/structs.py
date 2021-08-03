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

import ctypes


ELEVATION_ELEMENTS = 19
AZIMUTH_ELEMENTS = 37
TEMPERATURE_ELEMENTS = 9

# What are these for?
CAM_ERROR_CODE = 6201
M2_ERROR_CODE = 6151

# What is this?
HEXAPOD_DDS_TLM_CNT = 392


class Config(ctypes.Structure):
    """MTHexapod configuration.

    In the low-level controller code these are defined in
    ``ddsConfigTelemetryStreamStructure_t``.
    """

    _pack_ = 1
    _fields_ = [
        ("acceleration_strut", ctypes.c_double),
        # Order: max xy, min z, max z, max uv, min w, max w
        ("pos_limits", ctypes.c_double * 6),
        # Order: xy, uv, z, w
        ("vel_limits", ctypes.c_double * 4),
        ("initial_pos", ctypes.c_double * 6),
        ("pivot", ctypes.c_double * 3),
        ("max_displacement_strut", ctypes.c_double),
        ("max_velocity_strut", ctypes.c_double),
    ]
    # Set the frame ID when constructing the CSC
    FRAME_ID = None


class Telemetry(ctypes.Structure):
    """MTHexapod telemetry.

    In the low-level controller code these are defined in
    ``ddsTelemetryStreamStructure_t``.
    """

    _pack_ = 1
    _fields_ = [
        ("status_word", ctypes.c_uint16 * 6),
        ("latching_fault_status_register", ctypes.c_uint16 * 6),
        ("copley_fault_status_register", ctypes.c_uint32 * 6),
        ("application_status", ctypes.c_uint32),
        ("input_pin_states", ctypes.c_uint32 * 3),
        # TODO DM-31290: uncomment these lines and move them
        # to the correct location when the data is available
        # ("motor_current", ctypes.c_double * 6),
        # ("motor_voltage", ctypes.c_double * 6),
        # simulink telemetry
        ("state", ctypes.c_double),
        ("enabled_substate", ctypes.c_double),
        ("offline_substate", ctypes.c_double),
        ("test_state", ctypes.c_double),
        ("strut_encoder_raw", ctypes.c_double * 6),
        ("strut_encoder_microns", ctypes.c_double * 6),
        # commanded displacement (microns) and rotation (deg)
        ("commanded_pos", ctypes.c_double * 6),
        # measured displacement (microns) and rotation (deg)
        ("measured_pos", ctypes.c_double * 6),
        # commanded strut lengths (microns)
        ("commanded_length", ctypes.c_double * 6),
    ]
    # Set the frame ID when constructing the CSC
    FRAME_ID = None
