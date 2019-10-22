# This file is part of ts_rotator.
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

import time

import numpy as np

from . import utils


class Actuator:
    """Model an actuator that moves between given limits at constant velocity.

    Information is computed on request. This works because the system being
    modeled can only be polled.
    """
    def __init__(self, min_pos, max_pos, pos, speed):
        if speed <= 0:
            raise ValueError(f"speed={speed} must be positive")
        if min_pos >= max_pos:
            raise ValueError(f"min_pos={min_pos} >= max_pos={max_pos}")
        self.min_pos = min_pos
        self.max_pos = max_pos
        self._start_pos = pos
        self._start_time = time.monotonic()
        self._end_pos = pos
        self._end_time = time.monotonic()
        self.speed = speed
        if not min_pos <= pos <= max_pos:
            raise ValueError(f"pos={pos} not in range [{min_pos}, {max_pos}]")

    @property
    def start_pos(self):
        """Start position of move."""
        return self._start_pos

    @property
    def end_pos(self):
        """End position of move."""
        return self._end_pos

    def set_pos(self, pos):
        """Set a new desired position."""
        if pos < self.min_pos or pos > self.max_pos:
            raise ValueError(f"pos={pos} not in range [{self.min_pos}, {self.max_pos}]")
        self._start_pos = self.curr_pos
        self._start_time = time.monotonic()
        self._end_pos = pos
        dtime = self._move_duration()
        self._end_time = self._start_time + dtime

    def _move_duration(self):
        return abs(self.end_pos - self.start_pos) / self.speed

    @property
    def curr_pos(self):
        """Current position."""
        curr_time = time.monotonic()
        if curr_time > self._end_time:
            return self.end_pos
        else:
            dtime = curr_time - self._start_time
            return self.start_pos + self.direction*self.speed*dtime

    @property
    def direction(self):
        """1 if moving or moved to greater position, -1 otherwise."""
        return 1 if self.end_pos >= self.start_pos else -1

    @property
    def moving(self):
        """Is the axis moving?"""
        return time.monotonic() < self._end_time

    def stop(self):
        """Stop motion instantly.

        Set start_pos and end_pos to the current position
        and start_time and end_time to the current time.
        """
        curr_pos = self.curr_pos
        curr_time = time.monotonic()
        self._start_pos = curr_pos
        self._start_time = curr_time
        self._end_pos = curr_pos
        self._end_time = curr_time

    @property
    def remaining_time(self):
        """Remaining time for this move (sec)."""
        duration = self._end_time - time.monotonic()
        return max(duration, 0)


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
    base_positions : `List` [`numpy.ndarray`]
        Position of the base end of each linear actuator,
        as a list of z,y,z tuples, one per actuator.
    mirror_positions : `List` [`numpy.ndarray`]
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
    def __init__(self, base_positions, mirror_positions, pivot,
                 min_length, max_length, speed):
        if len(base_positions) != 6:
            raise ValueError(f"base_positions={base_positions} must have 6 elements")
        for pos in base_positions:
            if len(pos) != 3:
                raise ValueError(f"base_positions={base_positions}; each item must have 3 elements")
        if len(mirror_positions) != 6:
            raise ValueError(f"mirror_positions={mirror_positions} must have 6 elements")
        for pos in mirror_positions:
            if len(pos) != 3:
                raise ValueError(f"mirror_positions={mirror_positions}; each item must have 3 elements")
        if len(pivot) != 3:
            raise ValueError(f"pivot={pivot} must have 3 elements")
        self.base_positions = base_positions
        # Positions of mirror actuator ends and pivot at orientation zero.
        self.neutral_mirror_positions = mirror_positions
        self.neutral_pivot = np.array(pivot, dtype=float)
        # Information commanded by the `move` command.
        self.cmd_pos = np.zeros(3, dtype=float)
        self.cmd_xyzrot = np.zeros(3, dtype=float)
        self.cmd_mirror_positions = mirror_positions[:]
        self.neutral_actuator_lengths = self.compute_actuator_lengths(mirror_positions=mirror_positions)
        self.actuators = [Actuator(min_pos=min_length, max_pos=max_length, pos=actuator_length, speed=speed)
                          for actuator_length in self.neutral_actuator_lengths]

    @classmethod
    def make_zigzag_model(cls, base_radius, mirror_radius, mirror_z, base_angle0, pivot,
                          min_length, max_length, speed):
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
            base_angle = base_angle0 + i*delta_angle
            basepos = utils.rot_about_z((base_radius, 0, 0), base_angle*utils.RAD_PER_DEG)
            base_positions += [basepos, basepos]
            mirror_angle = base_angle + mirror_angle_offset
            mirrorpos = utils.rot_about_z((mirror_radius, 0, mirror_z), mirror_angle*utils.RAD_PER_DEG)
            mirror_positions += [mirrorpos, mirrorpos]
        # base_positions has order [5, 0, 1, 2, 3, 4]; fix to [0, 1, 2...]
        base_positions = base_positions[1:] + base_positions[0:1]
        return cls(base_positions=base_positions, mirror_positions=mirror_positions,
                   pivot=pivot, min_length=min_length, max_length=max_length, speed=speed)

    @property
    def moving(self):
        """Is any actuator moving?"""
        return any(actuator.moving for actuator in self.actuators)

    @property
    def remaining_time(self):
        """Remaining time for this move (sec)."""
        return max(actuator.remaining_time for actuator in self.actuators)

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
        """
        mirror_positions = self.compute_mirror_positions(pos=pos, xyzrot=xyzrot)
        # print(f"mirror_positions={mirror_positions}")
        # print(f"neutral_mirror_positions={self.neutral_mirror_positions}")
        actuator_lengths = self.compute_actuator_lengths(mirror_positions=mirror_positions)
        self.assert_in_range(actuator_lengths)
        for actuator, actuator_length in zip(self.actuators, actuator_lengths):
            actuator.set_pos(actuator_length)
        self.cmd_pos = pos
        self.cmd_xyzrot = xyzrot
        self.cmd_mirror_positions = mirror_positions

    def stop(self):
        """Stop all actuators.
        """
        for actuator in self.actuators:
            actuator.stop()

    def assert_in_range(self, actuator_lengths):
        """Assert that all actuators would be in range if set to the
        specified length.
        """
        in_range = [actuator.min_pos <= actuator_length <= actuator.max_pos
                    for actuator_length, actuator in zip(actuator_lengths, self.actuators)]
        if not all(in_range):
            raise ValueError(f"One or more actuators would be out of range: {in_range}")

    def compute_actuator_lengths(self, mirror_positions):
        """Compute actuator lengths given mirror positions
        and whether that length is in range.

        Parameters
        ----------
        mirror_positions : `List` [`numpy.ndarray`]
            Position of the mirror end of each actuator.

        Returns
        -------
        actuator_lengths : `List` [`float`]
            Required length of each actuator.
        in_range : `List` [`bool`]
            Is the length in range for each actuator?
        """
        return [np.linalg.norm(mirror_pos - base_pos)
                for mirror_pos, base_pos in zip(mirror_positions, self.base_positions)]

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
        xyzrot_rad = xyzrot*utils.RAD_PER_DEG
        # Compute the mirror position and resulting length of each actuator,
        # as well as determining if that length in range.
        mirror_positions = []
        for neutral_mirror_pos, base_pos in zip(self.neutral_mirror_positions, self.base_positions):
            # Rotate actuator mirror position
            # (we could do this after translation, but before is a bit easier).
            mirror_pos_in_pivot_frame = neutral_mirror_pos - self.neutral_pivot
            mirror_pos_in_pivot_frame = utils.rot_about_x(mirror_pos_in_pivot_frame, xyzrot_rad[0])
            mirror_pos_in_pivot_frame = utils.rot_about_y(mirror_pos_in_pivot_frame, xyzrot_rad[1])
            mirror_pos_in_pivot_frame = utils.rot_about_z(mirror_pos_in_pivot_frame, xyzrot_rad[2])
            mirror_pos = mirror_pos_in_pivot_frame + self.neutral_pivot

            # Translate actuator mirror position
            mirror_pos = mirror_pos + pos
            mirror_positions.append(mirror_pos)

        return mirror_positions