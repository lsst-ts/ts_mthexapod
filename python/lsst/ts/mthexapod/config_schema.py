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
    "CONFIG_SCHEMA",
]

import yaml

# Schema for HexapodCsc configuration
CONFIG_SCHEMA = yaml.safe_load(
    """
$schema: http://json-schema.org/draft-07/schema#
$id: https://github.com/lsst-ts/ts_mthexapod/blob/main/python/lsst/ts/mthexapod/config_schema.py
title: MTHexapod v7
description: Configuration for the MTHexapod CSCs

definitions:
  instance_specific_config:
    description: Configuration specific to the Camera or M2 MTHexapod.
    type: object
    properties:
      elevation_rotation_coeffs:
        description: >-
          Elevation/rotation compensation coefficients.
          Rows are coefficients for x, y, z (um), u, v, w (deg).
          The first index corresponds to powers of elevation and
          the second index corresponds to powers of rotation.
          Values are the coefficients in equation C_0_0
          + C_0_1 * el + C_0_2 * el**2 + ... + C_1_0 * rot
          + C_2_0 * rot**2 + ... + C_1_1 * el * rot
          + C_1_2 * el**2 * rot + ... + C_2_1 * el * rot**2 + ...
          where el is in deg.
          Compensated value = uncompensated (user-specified) value
            + elevation compensation
            + azimuth compensation
            + rotation compensation
            + temperature compensation.
        type: array
        minItems: 6
        maxItems: 6
        items:
          type: array
          minItems: 1
          items:
            type: array
            minItems: 1
            items:
              type: number
      azimuth_coeffs:
        description: >-
          Azimuth compensation coefficients.
          Rows are coefficients for x, y, z (um), u, v, w (deg).
          Values are the coefficients in equation C0 + C1 az + C2 az^2 + ...,
          where az is in deg.
        type: array
        minItems: 6
        maxItems: 6
        items:
          type: array
          minItems: 1
          items:
            type: number
      temperature_coeffs:
        description: >-
          Temperature compensation coefficients.
          Rows are coefficients for x, y, z (um), u, v, w (deg).
          Values are the coefficients in equation C0 + C1 temp + C2 temp^2 + ...,
          where temp is in C.
        type: array
        minItems: 6
        maxItems: 6
        items:
          type: array
          minItems: 1
          items:
            type: number
      min_compensation_adjustment:
        description: >-
          The smallest compensation adjustment the CSC's background compensation
          task will command: x, y, z (um), u, v, w (deg).
          See the User Guide for details.
        type: array
        minItems: 6
        maxitems: 6
        items:
          type: number
          minimum: 0
      min_temperature:
        description: >-
          Minimum temperatures (C) for which the temperature model is valid.
          Below this temperature, terms above the first order are ignored;
          see RangedPolynomial for details.
        type: number
      max_temperature:
        description: >-
          Maximum temperatures (C) for which the temperature model is valid.
          Above this temperature, terms above the first order are ignored;
          see RangedPolynomial for details.
        type: number
      enable_lut_temperature:
        description: >-
          Use temperature LUT to compensate the deformation caused by
          temperature gradients or not.
        type: boolean
      no_movement_idle_time:
        description: >-
          Time limit for no movement in seconds under the Enabled state.
        type: number
        exclusiveMinimum: 0
      step_size_xy:
        description: >-
          Absolute maximum step size in the x, y direction in um. Put 0 if you
          want to do the movement in a single step.
        type: number
      step_size_z:
        description: >-
          Absolute maximum step size in the z direction in um. Put 0 if you
          want to do the movement in a single step.
        type: number
      step_size_uv:
        description: >-
          Absolute maximum step size in the rx, ry rotation in deg. Put 0 if
          you want to do the movement in a single step.
        type: number
      step_size_w:
        description: >-
          Absolute maximum step size in the rz rotation in deg. Put 0 if you
          want to do the movement in a single step.
        type: number
      camera:
        description: >-
          Camera name.
          Used to select the correct configuration.
        type: string
        enum: ["CCCamera", "MTCamera"]
      filter_offsets:
        description: >-
          Filter offsets with specific z_offset for each filter.
        type: object
        patternProperties:
          "^[a-zA-Z0-9_]+$":
            type: object
            properties:
              z_offset:
                description: >-
                  Z-offset value for the filter.
                type: number
            required:
              - z_offset
        additionalProperties: false
      apply_compensation_while_exposing:
        description: >-
            Should compensation be applyed while
            exposing? If False, the CSC will monitor the
            camera shutter and will skip applying compensation
            while the shutter is open.
        type: boolean
      host:
        description: IP address of the TCP/IP interface.
        type: string
        format: hostname
      port:
        description: >-
          Telemetry port number of the TCP/IP interface.
          The command port is one larger.
        type: integer
    required:
      - elevation_rotation_coeffs
      - azimuth_coeffs
      - temperature_coeffs
      - min_compensation_adjustment
      - min_temperature
      - max_temperature
      - enable_lut_temperature
      - no_movement_idle_time
      - step_size_xy
      - step_size_z
      - step_size_uv
      - step_size_w
      - apply_compensation_while_exposing
      - host
      - port
    additionalProperties: false

type: object
properties:
  compensation_interval:
    description: Time between compensation updates (seconds).
    type: number
  connection_timeout:
    description: Time limit for connecting to the TCP/IP interface (sec)
    type: number
    exclusiveMinimum: 0
  camera_config:
    $ref: "#/definitions/instance_specific_config"
  m2_config:
    $ref: "#/definitions/instance_specific_config"
required:
  - compensation_interval
  - connection_timeout
  - camera_config
  - m2_config
additionalProperties: false
"""
)
