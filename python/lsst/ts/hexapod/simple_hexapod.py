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

__all__ = ["SimpleHexapod"]

import numpy as np

from . import utils
from lsst.ts import simactuators


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
    base_positions : `List` [`List` [`float`]]
        Position of the base end of each linear actuator,
        as a list of z,y,z tuples, one per actuator.
    mirror_positions : `List` [`List` [`float`]]
        Position of the mirror end of each actuator at zero
        orientation, as a list of z,y,z tuples, one per actuator.
    pivot : `numpy.ndarray`
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
        self, base_positions, mirror_positions, pivot, min_length, max_length, speed
    ):
        if len(base_positions) != 6:
            raise ValueError(f"base_positions={base_positions} must have 6 elements")
        for pos in base_positions:
            if len(pos) != 3:
                raise ValueError(
                    f"base_positions={base_positions}; each item must have 3 elements"
                )
        if len(mirror_positions) != 6:
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
        self.base_positions = [np.array(pos) for pos in base_positions]
        # Positions of mirror actuator ends and pivot at orientation zero.
        self.neutral_mirror_positions = [np.array(pos) for pos in mirror_positions]
        self.neutral_pivot = np.array(pivot, dtype=float)
        # Information commanded by the `move` command.
        self.cmd_pos = np.zeros(3, dtype=float)
        self.cmd_xyzrot = np.zeros(3, dtype=float)
        self.cmd_mirror_positions = mirror_positions[:]
        self.neutral_actuator_lengths = self.compute_actuator_lengths(
            mirror_positions=mirror_positions, absolute=True
        )
        self.actuators = [
            simactuators.PointToPointActuator(
                min_position=min_length,
                max_position=max_length,
                start_position=0,
                speed=speed,
            )
            for actuator_length in self.neutral_actuator_lengths
        ]

    @classmethod
    def make_zigzag_model(
        cls,
        base_radius,
        mirror_radius,
        mirror_z,
        base_angle0,
        pivot,
        min_length,
        max_length,
        speed,
    ):
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
        pivot : `numpy.ndarray`
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
                (base_radius, 0, 0), base_angle * utils.RAD_PER_DEG
            )
            base_positions += [basepos, basepos]
            mirror_angle = base_angle + mirror_angle_offset
            mirrorpos = utils.rot_about_z(
                (mirror_radius, 0, mirror_z), mirror_angle * utils.RAD_PER_DEG
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

    def moving(self, tai=None):
        """Is any actuator moving?"""
        return any(actuator.moving(tai) for actuator in self.actuators)

    def remaining_time(self, tai=None):
        """Remaining time for this move (sec)."""
        return max(actuator.remaining_time(tai) for actuator in self.actuators)

    def move(self, pos, xyzrot):
        """Move the actuators so the pivot point is at the specified
        orientation.

        Parameters
        ----------
        pos : `numpy.ndarray`
            x, y, z position of pivot point.
        xyzrot : `numpy.ndarray`
            Orientation of translated pivot point,
            as a rotation about x, then y, then z (deg).

        Returns
        -------
        duration : `float`
            Duration of the move (second).
        """
        mirror_positions = self.compute_mirror_positions(pos=pos, xyzrot=xyzrot)
        # print(f"mirror_positions={mirror_positions}")
        # print(f"neutral_mirror_positions={self.neutral_mirror_positions}")
        actuator_lengths = self.compute_actuator_lengths(
            mirror_positions=mirror_positions, absolute=False
        )
        self.assert_in_range(actuator_lengths)
        max_duration = 0
        for actuator, actuator_length in zip(self.actuators, actuator_lengths):
            duration = actuator.set_position(actuator_length)
            max_duration = max(duration, max_duration)
        self.cmd_pos = pos
        self.cmd_xyzrot = xyzrot
        self.cmd_mirror_positions = mirror_positions
        return max_duration

    def stop(self):
        """Stop all actuators.
        """
        for actuator in self.actuators:
            actuator.stop()

    def assert_in_range(self, actuator_lengths):
        """Assert that all actuators would be in range if set to the
        specified length.
        """
        in_range = [
            actuator.min_position <= actuator_length <= actuator.max_position
            for actuator_length, actuator in zip(actuator_lengths, self.actuators)
        ]
        if not all(in_range):
            raise ValueError(f"One or more actuators would be out of range: {in_range}")

    def compute_actuator_lengths(self, mirror_positions, absolute):
        """Compute actuator lengths, given mirror positions.

        Parameters
        ----------
        mirror_positions : `List` [`numpy.ndarray`]
            Position of the mirror end of each actuator.
        absolute : `bool`
            If True then return end to end actuator lengths.
            If False then return lengths relative to neutral lengths;
            this requires ``self.neutral_actuator_lengths``.

        Returns
        -------
        actuator_end_to_end_lengths : `numpy.ndarray`
            End to end length of each actuator.
        """
        lengths = np.array(
            [
                np.linalg.norm(mirror_pos - base_pos)
                for mirror_pos, base_pos in zip(mirror_positions, self.base_positions)
            ],
            dtype=float,
        )
        if not absolute:
            lengths -= self.neutral_actuator_lengths
        return lengths

    def compute_mirror_positions(self, pos, xyzrot):
        """Compute the actuator mirror positions needed to move the pivot point
        to a specified orientation.

        Parameters
        ----------
        pos : `numpy.ndarray`
            Desired x, y, z position of pivot point,
            relative to the neutral pivot point.
        xyzrot : `numpy.ndarray`
            Orientation of translated pivot point,
            as a rotation about x, then y, then z (deg).

        Returns
        -------
        mirror_positions : `List` [`numpy.ndarray`]
            Resulting position of the mirror end of each actuator.
        """
        pos = np.array(pos, dtype=float)
        xyzrot = np.array(xyzrot, dtype=float)
        xyzrot_rad = xyzrot * utils.RAD_PER_DEG
        # Compute the mirror position and resulting length of each actuator,
        # as well as determining if that length in range.
        mirror_positions = []
        for neutral_mirror_pos, base_pos in zip(
            self.neutral_mirror_positions, self.base_positions
        ):
            # Rotate actuator mirror position
            # (we could do this after translation, but before is a bit easier).
            mirror_pos_in_pivot_frame = neutral_mirror_pos - self.neutral_pivot
            mirror_pos_in_pivot_frame = utils.rot_about_x(
                mirror_pos_in_pivot_frame, xyzrot_rad[0]
            )
            mirror_pos_in_pivot_frame = utils.rot_about_y(
                mirror_pos_in_pivot_frame, xyzrot_rad[1]
            )
            mirror_pos_in_pivot_frame = utils.rot_about_z(
                mirror_pos_in_pivot_frame, xyzrot_rad[2]
            )
            mirror_pos = mirror_pos_in_pivot_frame + self.neutral_pivot

            # Translate actuator mirror position
            mirror_pos = mirror_pos + pos
            mirror_positions.append(mirror_pos)

        return mirror_positions
