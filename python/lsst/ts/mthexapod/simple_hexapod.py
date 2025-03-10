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

__all__ = ["SimpleHexapod"]

import typing

import numpy as np
import numpy.typing
from lsst.ts import simactuators
from scipy.optimize import minimize

from . import utils
from .constants import NUM_STRUT


class SimpleHexapod:
    """Simple model of a hexapod: 6 linear actuators in an arbitrary
    arrangement.

    The intent is to support the mock hexapod controller; as such
    this model is somewhat simplistic. The actuators ends are assumed
    to be perfect point flexures. It is not (yet) possible to compute
    orientation given actuator lengths.

    See `make_zigzag_model` to make a standard symmetrical zigzag hexapod.

    Parameters
    ----------
    base_positions : `list` [`numpy.ndarray`]
        Position of the base end of each linear actuator,
        as a list of z,y,z tuples, one per actuator.
    mirror_positions : `list` [`numpy.ndarray`]
        Position of the mirror end of each actuator at zero
        orientation, as a list of z,y,z tuples, one per actuator.
    pivot : `tuple`
        The point whose orientation is set by the `move` command.
        For a mirror it will typically be the vertex of the mirror.
    min_length : `float`
        Mininum actuator length.
    max_length : `float`
        Maximum actuator length.
    speed : `float`
        Actuator speed.
    """

    def __init__(
        self,
        base_positions: list[numpy.typing.NDArray[np.float64]],
        mirror_positions: list[numpy.typing.NDArray[np.float64]],
        pivot: tuple[float, float, float],
        min_length: float,
        max_length: float,
        speed: float,
    ) -> None:
        if len(base_positions) != NUM_STRUT:
            raise ValueError(f"base_positions={base_positions} must have 6 elements")
        for pos in base_positions:
            if len(pos) != 3:
                raise ValueError(
                    f"base_positions={base_positions}; each item must have 3 elements"
                )
        if len(mirror_positions) != NUM_STRUT:
            raise ValueError(
                f"mirror_positions={mirror_positions} must have 6 elements"
            )
        for pos in mirror_positions:
            if len(pos) != 3:
                raise ValueError(
                    f"mirror_positions={mirror_positions}; each item must have 3 elements"
                )
        if len(pivot) != 3:
            raise ValueError(f"pivot={pivot} must have 3 elements")
        self.base_positions = np.array(base_positions).T * utils.UM_TO_M
        # Positions of mirror actuator ends and pivot at orientation zero.
        self.neutral_mirror_positions = np.array(mirror_positions).T * utils.UM_TO_M
        self.neutral_pivot = np.array(pivot, dtype=float) * utils.UM_TO_M
        # Information commanded by the `move` command.
        self.cmd_pos = np.zeros(3, dtype=float)
        self.cmd_xyzrot = np.zeros(3, dtype=float)
        self.actuators = [
            simactuators.PointToPointActuator(
                min_position=min_length,
                max_position=max_length,
                start_position=0,
                speed=speed,
            )
            for idx in range(NUM_STRUT)
        ]

    @classmethod
    def make_zigzag_model(
        cls,
        base_radius: float,
        mirror_radius: float,
        mirror_z: float,
        base_angle0: float,
        pivot: tuple[float, float, float],
        min_length: float,
        max_length: float,
        speed: float,
    ) -> typing.Self:
        """Make a `SimpleHexapod` of a typical hexapod with 6 actuators in a
        symmetrical zigzag arrangement.

        The base ends of the 6 actuators terminate at 3 points at ``z=0``
        evenly distributed about a circle of radius ``base_radius``:
        actuators 0 and 5 terminate at base_angle0,
        actuators 1 and 2 terminate at base_angle0 + 120,
        and actuators 3 and 4 terminate at base_angle0 + 240.
        The mirror ends of the actuators are similarly arrayed,
        in a plane at ``z=mirror_z`` with attachment points
        rotated 60 degrees from the base attachment points:
        actuators 0 and 1 terminate at base_angle0 + 60, etc.
        This makes a zigzag pattern that is circularly symmetric
        about the z axis.

        base_radius : `float`
            Radius of base positions of actuators.
        mirror_radius : `float`
            Radius of mirror positions of actuators.
        mirror_z : `float`
            z distance between the base ends and the mirror ends
            of the linear actuators.
        base_angle0 : `float`
            Angle of first base actuator point in x,y plane (deg).
        pivot : `tuple`
            The point whose orientation is set by the `move` command.
            For a mirror it will typically be the vertex of the mirror.
        min_length : `float`
            Mininum actuator length.
        max_length : `float`
            Maximum actuator length.
        speed : `float`
            Actuator speed.
        """
        base_positions = []
        mirror_positions = []
        delta_angle = 120
        mirror_angle_offset = 60
        for i in range(3):
            base_angle = base_angle0 + i * delta_angle
            basepos = utils.rot_about_z(
                (base_radius, 0.0, 0.0), base_angle * utils.RAD_PER_DEG
            )
            base_positions += [basepos, basepos]
            mirror_angle = base_angle + mirror_angle_offset
            mirrorpos = utils.rot_about_z(
                (mirror_radius, 0.0, mirror_z), mirror_angle * utils.RAD_PER_DEG
            )
            mirror_positions += [mirrorpos, mirrorpos]
        # base_positions has order [5, 0, 1, 2, 3, 4]; fix to [0, 1, 2...]
        base_positions = base_positions[1:] + base_positions[0:1]
        return cls(
            base_positions=base_positions,
            mirror_positions=mirror_positions,
            pivot=pivot,
            min_length=min_length,
            max_length=max_length,
            speed=speed,
        )

    def moving(self, tai: float | None = None) -> bool:
        """Is any actuator moving?"""
        return any(actuator.moving(tai) for actuator in self.actuators)

    def remaining_time(self, tai: float | None = None) -> float:
        """Remaining time for this move (sec)."""
        return max(actuator.remaining_time(tai) for actuator in self.actuators)

    def move(
        self, pos: tuple[float, float, float], xyzrot: tuple[float, float, float]
    ) -> float:
        """Move the actuators so the pivot point is at the specified
        orientation.

        Parameters
        ----------
        pos : `tuple`
            x, y, z position of pivot point.
        xyzrot : `tuple`
            Orientation of translated pivot point,
            as a rotation about x, then y, then z (deg).

        Returns
        -------
        duration : `float`
            Duration of the move (second).
        """

        positions_xyz = np.array(pos) * utils.UM_TO_M
        positions_rxryrz = np.deg2rad(np.array(xyzrot))

        actuator_lengths = (
            SimpleHexapod.inverse_kinematics(
                np.append(positions_xyz, positions_rxryrz),
                self.neutral_pivot,
                self.neutral_mirror_positions,
                self.base_positions,
            )
            / utils.UM_TO_M
        )

        self.assert_in_range(actuator_lengths)
        max_duration = 0
        for actuator, actuator_length in zip(self.actuators, actuator_lengths):
            duration = actuator.set_position(actuator_length)
            max_duration = max(duration, max_duration)
        self.cmd_pos = np.array(pos)
        self.cmd_xyzrot = np.array(xyzrot)
        return max_duration

    def stop(self) -> None:
        """Stop all actuators."""
        for actuator in self.actuators:
            actuator.stop()

    def assert_in_range(
        self, actuator_lengths: numpy.typing.NDArray[np.float64]
    ) -> None:
        """Assert that all actuators would be in range if set to the
        specified length.
        """
        in_range = [
            actuator.min_position <= actuator_length <= actuator.max_position
            for actuator_length, actuator in zip(actuator_lengths, self.actuators)
        ]
        if not all(in_range):
            raise ValueError(f"One or more actuators would be out of range: {in_range}")

    @staticmethod
    def inverse_kinematics(
        positions: numpy.typing.NDArray[np.float64],
        pivot: numpy.typing.NDArray[np.float64],
        mirror_positions: numpy.typing.NDArray[np.float64],
        base_positions: numpy.typing.NDArray[np.float64],
    ) -> numpy.typing.NDArray[np.float64]:
        """Inverse kinematics for a hexapod movement. This calculates the delta
        strut lengths based on the hexapod positions.

        Notes
        -----
        The calculation is translated from ts_mt_hexRot_simulink repository:
        hexapod_controller_source_final/hexapod_kin_calc.slx

        Parameters
        ----------
        positions : `numpy.ndarray`
            Hexapod positions: (x, y, z, rx, ry, rz). The units are the meter
            and radian.
        pivot : `numpy.ndarray`
            Pivot (x, y, z) as the rotation center. The unit is meter.
        mirror_positions : `numpy.ndarray`
            Strut positions on the mirror. This is a 3x6 matrix. The row is the
            (x, y, z) position in meter. The column is the strut index.
        base_positions : `numpy.ndarray`
            Strut positions on the base. This is a 3x6 matrix. The row is the
            (x, y, z) position in meter. The column is the strut index.

        Returns
        -------
        `numpy.ndarray`
            6 delta strut lengths in meter.
        """

        x, y, z, rx, ry, rz = positions

        # Consider the translation part
        translation = np.kron(
            (np.array([x, y, z]) + pivot).reshape(len(pivot), -1),
            np.ones(NUM_STRUT),
        )

        # Consider the rotation part

        # Rotation matrix
        rot_x = np.array(
            [[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]]
        )
        rot_y = np.array(
            [[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]]
        )
        rot_z = np.array(
            [[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]]
        )
        rot_xyz = rot_x.dot(rot_y).dot(rot_z)

        # Rotate the base positions
        rotation_projection = rot_xyz.dot(
            base_positions
            - np.kron(
                pivot.reshape(len(pivot), -1),
                np.ones(NUM_STRUT),
            )
        )

        # Delta change
        delta_location = translation + rotation_projection - mirror_positions

        # Calculate the delta strut lengths
        strut_length_total = np.sqrt(np.sum(delta_location**2, axis=0))

        strut_length_norm = np.sqrt(
            np.sum((base_positions - mirror_positions) ** 2, axis=0)
        )
        strut_length_delta = strut_length_total - strut_length_norm

        return strut_length_delta

    @staticmethod
    def forward_kinematics(
        initial_guess: numpy.typing.NDArray[np.float64],
        strut_length_delta: numpy.typing.NDArray[np.float64],
        pivot: numpy.typing.NDArray[np.float64],
        mirror_positions: numpy.typing.NDArray[np.float64],
        base_positions: numpy.typing.NDArray[np.float64],
        tol: float = 1e-6,
    ) -> numpy.typing.NDArray[np.float64]:
        """Forward kinematics for a hexapod movement. This calculates the
        hexapod positions based on the delta strut lengths.

        Notes
        -----
        The calculation is translated from ts_mt_hexRot_simulink repository:
        hexapod_controller_source_final/hexapod_kin_calc.slx

        Parameters
        ----------
        initial_guess : `numpy.ndarray`
            Initial guess of the hexapod positions: (x, y, z, rx, ry, rz). The
            units are the meter and radian.
        strut_length_delta : `numpy.ndarray`
            6 delta strut lengths in meter.
        pivot : `numpy.ndarray`
            Pivot (x, y, z) as the rotation center. The unit is meter.
        mirror_positions : `numpy.ndarray`
            Strut positions on the mirror. This is a 3x6 matrix. The row is the
            (x, y, z) position in meter. The column is the strut index.
        base_positions : `numpy.ndarray`
            Strut positions on the base. This is a 3x6 matrix. The row is the
            (x, y, z) position in meter. The column is the strut index.
        tol : `float`, optional
            Tolerance for the optimization. (the default is 1e-6)

        Returns
        -------
        `numpy.ndarray`
            Estimated hexapod positions: (x, y, z, rx, ry, rz). The units are
            the meter and radian.
        """

        res = minimize(
            lambda x: np.sum(
                (
                    SimpleHexapod.inverse_kinematics(
                        x,
                        pivot,
                        mirror_positions,
                        base_positions,
                    )
                    - strut_length_delta
                )
                ** 2
            ),
            initial_guess,
            method="Nelder-Mead",
            tol=tol,
        )
        return res.x
