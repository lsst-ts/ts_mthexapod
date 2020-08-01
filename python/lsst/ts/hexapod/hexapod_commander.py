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

import numpy as np

from lsst.ts import salobj

from . import enums

STD_TIMEOUT = 5  # timeout for command ack


def as_bool(value):
    """Return a string cast to a bool.

    The following are false: 0, f, false
    The following are true: 1, t, true
    No other values are valid.
    """
    return {"0": False, "f": False, "false": False, "1": True, "t": True, "true": True}[
        value
    ]


class HexapodCommander(salobj.CscCommander):
    """Command the Hexapod CSC from the command line.

    Parameters
    ----------
    index : `int`
        SAL index of Hexapod CSC.

    Read commands from stdin and write updated events and telemetry to stdout.
    The telemetry is filtered so that tiny changes due to encoder jitter
    are ignored.

    See bin/command_hexapod.py for an example of how to use this class.
    """

    def __init__(self, index, enable):
        index = enums.SalIndex(index)
        super().__init__(
            name="Hexapod", index=index, enable=enable,
        )
        for command_to_ignore in ("abort", "setValue"):
            del self.command_dict[command_to_ignore]

    def positions_close(self, position1, position2):
        """Return True if two positions are nearly equal.

        Parameters
        ----------
        position1 : `List` [`float`]
            Position 1: x, y, z (µm) and rotx, roty, rotz (deg)
        position2 : `List` [`float`]
            Position 2: x, y, z (µm) and rotx, roty, rotz (deg)
        """
        return np.allclose(position1[:3], position2[:3], atol=1) and np.allclose(
            position1[3:], position2[3:], atol=1e-5
        )

    async def tel_actuators_callback(self, data):
        """Callback for actuators telemetry.

        Output actuators telemetry data if the values have changed enough
        to be interesting.

        Parameters
        ----------
        data : self.controller.tel_actuators.DataType.
            Actuators data.
        """
        if self.previous_tel_actuators is not None and np.allclose(
            self.previous_tel_actuators.calibrated, data.calibrated, atol=1
        ):
            return
        self.previous_tel_actuators = data
        print(f"actuators: {self.format_data(data)}")

    async def tel_application_callback(self, data):
        """Callback for the application telemetry.

        Output application telemetry if the values have changed enough
        to be interesting.

        Parameters
        ----------
        data : self.controller.tel_application.DataType.
            Actuators data.
        """
        if (
            self.previous_tel_application is not None
            and self.positions_close(
                self.previous_tel_application.position, data.position
            )
            and np.array_equal(data.demand, self.previous_tel_application.demand)
        ):
            return
        self.previous_tel_application = data
        print(f"application: {self.format_data(data)}")
