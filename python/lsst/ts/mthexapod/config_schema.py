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
$id: https://github.com/lsst-ts/ts_mthexapod/blob/master/python/lsst/ts/mthexapod/config_schema.py
title: MTHexapod v1
description: Configuration for the MTHexapod CSCs

definitions:
  instance_specific_config:
    description: Configuration specific to the Camera or M2 MTHexapod.
    type: object
    properties:
      elevation_coeffs:
        description: >-
          Elevation compensation coefficients.
          Rows are coefficients for x, y, z (um), u, v, w (deg).
          Values are the coefficients in equation C0 + C1 el + C2 el^2 + ...,
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
      rotation_coeffs:
        description: >-
          Camera rotation compensation coefficients.
          Rows are coefficients for x, y, z (um), u, v, w (deg).
          Values are the coefficients in equation C0 + C1 rot + C2 rot^2 + ...,
          where rot is in deg.
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
    required:
      - elevation_coeffs
      - azimuth_coeffs
      - rotation_coeffs
      - temperature_coeffs
      - min_compensation_adjustment
      - min_temperature
      - max_temperature
    additionalProperties: false

type: object
properties:
  compensation_interval:
    description: Time between compensation updates (seconds).
    type: number
    default: 0.2
  camera_config:
    $ref: "#/definitions/instance_specific_config"
    default:
      elevation_coeffs:
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
      azimuth_coeffs:
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
      rotation_coeffs:
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
      temperature_coeffs:
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
      min_compensation_adjustment:
        - 1
        - 1
        - 1
        - 1.0e-4
        - 1.0e-4
        - 1.0e-4
      min_temperature: -20
      max_temperature: 30
  m2_config:
    $ref: "#/definitions/instance_specific_config"
    default:
      elevation_coeffs:
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
      azimuth_coeffs:
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
      rotation_coeffs:
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
      temperature_coeffs:
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
        - [0]
      min_compensation_adjustment:
        - 1
        - 1
        - 1
        - 1.0e-4
        - 1.0e-4
        - 1.0e-4
      min_temperature: -20
      max_temperature: 30
required: [camera_config, m2_config]
additionalProperties: false
"""
)
