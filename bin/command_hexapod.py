#!/usr/bin/env python
# This file is part of ts_hexapod.
#
# Developed for the LSST Telescope and Site Systems.
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
"""Monitor and command an MT Hexapod.

To use:

command_hexapod.py 1  # For the Camera Hexapod

or

command_hexapod.py 2  # For the M2 Hexapod

Then wait for it to connect. Once it has connected it will print
initial hexapod status and help.

Commands are entered by typing the command and arguments (if any),
separated by spaces, then <return>. "help" is a command.
"""
import asyncio
import argparse

from lsst.ts import hexapod

parser = argparse.ArgumentParser(f"Command the hexapod from the command line")
parser.add_argument("index", type=int,
                    help="Hexapod index; 1=Camera 2=M2")
args = parser.parse_args()

commander = hexapod.HexapodCommander(index=args.index)
asyncio.get_event_loop().run_until_complete(commander.amain())
