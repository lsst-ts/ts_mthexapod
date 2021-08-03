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
        # Order: xy, uv, z, w
        ("vel_limits", ctypes.c_double * 4),
        ("max_displacement_strut", ctypes.c_double),
        ("max_velocity_strut", ctypes.c_double),
        ("pos_limits", ctypes.c_double * 6),
        ("pivot", ctypes.c_double * 3),
    ]
    # Set the frame ID when constructing the CSC
    FRAME_ID = None


class PosVel(ctypes.Structure):
    """Handle annoying order of strut_commanded_posvel telemetry fields."""

    _pack_ = 1
    _fields_ = [
        ("pos", ctypes.c_double),
        ("vel", ctypes.c_double),
    ]


class PosFiltVel(ctypes.Structure):
    """Handle annoying order of strut_measured_posfiltvel telemetry fields."""

    _pack_ = 1
    _fields_ = [
        ("pos", ctypes.c_double),
        ("pos_filt", ctypes.c_double),
        ("vel", ctypes.c_double),
    ]


class Telemetry(ctypes.Structure):
    """MTHexapod telemetry.

    In the low-level controller code these are defined in
    ``ddsTelemetryStreamStructure_t``.
    """

    _pack_ = 1
    _fields_ = [
        # extBissEncoderRaw in telemetryStreamStructure_t
        ("strut_measured_pos_raw", ctypes.c_int32 * 6),
        ("status_word", ctypes.c_uint16 * 6),
        ("latching_fault_status_register", ctypes.c_uint16 * 6),
        ("copley_fault_status_register", ctypes.c_uint32 * 6),
        ("application_status", ctypes.c_uint32),
        ("input_pin_states", ctypes.c_uint32 * 3),
        ("mjd", ctypes.c_double),
        ("mjd_frac", ctypes.c_double),
        ("dt", ctypes.c_double),
        # StrutPosition_1 in telemetryStreamStructure_t;
        # documented as: simulink "calibrated" position (in microns?)
        ("strut_measured_pos_um", ctypes.c_double * 6),
        # CommandPos_n, CommandVel_n in telemetryStreamStructure_t
        ("strut_commanded_posvel", PosVel * 6),
        # CommandAccel_n in telemetryStreamStructure_t
        ("strut_commanded_accel", ctypes.c_double * 6),
        # CommandFinalPos_n in telemetryStreamStructure_t
        ("strut_commanded_final_pos", ctypes.c_double * 6),
        ("command_valid", ctypes.c_double),
        ("track_mode", ctypes.c_double),
        # simulink telemetry
        ("state", ctypes.c_double),
        ("enabled_substate", ctypes.c_double),
        ("offline_substate", ctypes.c_double),
        ("test_state", ctypes.c_double),
        # PosErrorn in telemetryStreamStructure_t with this note:
        # Get the PosError from the drive directly instead of Simulink Model
        ("strut_pos_error", ctypes.c_double * 6),
        # Rn_Est in telemetryStreamStructure_t
        ("measured_uvw", ctypes.c_double * 3),
        # Posn_Est in telemetryStreamStructure_t
        ("measured_xyz", ctypes.c_double * 3),
        ("num_iterations", ctypes.c_double),
        # EstPos_M_n, EstPos_M_Filt_n, EstVel_n in telemetryStreamStructure_t
        ("estimated_posfiltvel", PosFiltVel * 6),
        # LinearEncodern in telemetryStreamStructure_t;
        # I have no idea how this differs from strut_measured_pos_raw
        ("strut_linear_encoder", ctypes.c_double * 6),
        # LinearEncoderVelocityn in telemetryStreamStructure_t
        ("strut_linear_encoder_velocity", ctypes.c_double * 6),
        # Probably hexCmd_mic_deg in telemetryStreamStructure_t
        ("commanded_pos", ctypes.c_double * 6),
        # TODO DM-31290: uncomment these lines and move them
        # to the correct location when the data is available
        # ("motor_current", ctypes.c_double * 6),
        # ("motor_voltage", ctypes.c_double * 6),
    ]
    # Set the frame ID when constructing the CSC
    FRAME_ID = None
