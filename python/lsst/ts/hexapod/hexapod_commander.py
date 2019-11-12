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

__all__ = ["HexapodCommander"]

from lsst.ts import hexrotcomm

from . import enums

STD_TIMEOUT = 5  # timeout for command ack


def as_bool(value):
    """Return a string cast to a bool.

    The following are false: 0, f, false
    The following are true: 1, t, true
    No other values are valid.
    """
    return {
        "0": False,
        "f": False,
        "false": False,
        "1": True,
        "t": True,
        "true": True,
    }[value]


class HexapodCommander(hexrotcomm.CscCommander):
    def __init__(self, index):
        index = enums.SalIndex(index)
        super().__init__(
            name="Hexapod",
            index=index,
            help_text="""Special commands:
* exit  # Quit the interpreter.
* help  # Print this help.

State transitions commands (none take arguments):
* enterControl
* start
* enable
* disable
* standby
* exitControl
* clearError

Other commands and arguments:
* configureAcceleration accmax                      # Set acceleration: µm/s2
* configureLimits xymax zmin zmax uvmax wmin wmax   # Set position limits: µm µm µm deg deg deg
* configureVelocity xymax rxrymax zmax rzmax        # Set velocity: µm/s deg/s µm/s deg/s
* move                          # Move to position set by positionSet or offset
* moveLUT az elev temp          # Same as "move" but with lookup table compensation: deg deg C
* offset x y z u v w synch      # Specify an offset for move or moveLUT: µm µm µm deg deg deg 0/1
* pivot x y z                   # Set pivot point: µm µm µm
* positionSet x y z u v w synch # Specify a position for move or moveLUT: µm µm µm deg deg deg 0/1
* stop
* test ivalue1 ivalue2

For example:
  positionSet 5 5 5 0.001 0 0 0
  move
  stop  # in case you want to stop a move early
  exit""")

    async def do_configureAcceleration(self, args):
        kwargs = self.check_arguments(args, "accmax")
        await self.remote.cmd_configureAcceleration.set_start(**kwargs, timeout=STD_TIMEOUT)

    async def do_configureLimits(self, args):
        kwargs = self.check_arguments(args, "xymax", "zmin", "zmax", "uvmax", "wmin", "wmax")
        await self.remote.cmd_configureLimits.set_start(**kwargs, timeout=STD_TIMEOUT)

    async def do_configureVelocity(self, args):
        kwargs = self.check_arguments(args, "xymax", "rxrymax", "zmax", "rzmax")
        await self.remote.cmd_configureVelocity.set_start(**kwargs, timeout=STD_TIMEOUT)

    async def do_move(self, args):
        self.check_arguments(args)
        await self.remote.cmd_move.start(timeout=STD_TIMEOUT)

    async def do_moveLUT(self, args):
        kwargs = self.check_arguments(args, "az", "elev", "temp")
        await self.remote.cmd_moveLUT.set_start(**kwargs, timeout=STD_TIMEOUT)

    async def do_offset(self, args):
        kwargs = self.check_arguments(args, "x", "y", "z", "u", "v", "w", ("sync", as_bool))
        await self.remote.cmd_offset.set_start(**kwargs, timeout=STD_TIMEOUT)

    async def do_pivot(self, args):
        kwargs = self.check_arguments(args, "x", "y", "z")
        await self.remote.cmd_pivot.set_start(**kwargs, timeout=STD_TIMEOUT)

    async def do_positionSet(self, args):
        kwargs = self.check_arguments(args, "x", "y", "z", "u", "v", "w", ("sync", as_bool))
        await self.remote.cmd_positionSet.set_start(**kwargs, timeout=STD_TIMEOUT)

    async def do_stop(self, args):
        # Don't check arguments, just STOP.
        await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)

    async def do_test(self, args):
        kwargs = self.check_arguments(args, ("ivalue1", int), ("ivalue2", int))
        await self.remote.cmd_test.set_start(**kwargs, timeout=STD_TIMEOUT)
