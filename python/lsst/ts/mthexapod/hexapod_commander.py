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

__all__ = ["HexapodCommander", "command_hexapod"]

import asyncio

import numpy as np
from lsst.ts import salobj

from . import enums

STD_TIMEOUT = 5  # timeout for command ack


class HexapodCommander(salobj.CscCommander):
    """Command the MTHexapod CSC from the command line.

    Parameters
    ----------
    index : `int`
        SAL index of MTHexapod CSC.

    Read commands from stdin and write updated events and telemetry to stdout.
    The telemetry is filtered so that tiny changes due to encoder jitter
    are ignored.

    Used by `command_mthexapod`.
    """

    def __init__(self, index: int, enable: bool) -> None:
        index = enums.SalIndex(index)
        super().__init__(
            name="MTHexapod",
            index=index,
            enable=enable,
        )
        for command_to_ignore in ("enterControl", "abort", "setValue"):
            self.command_dict.pop(command_to_ignore, None)

    def positions_close(self, position1: list[float], position2: list[float]) -> bool:
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

    async def tel_actuators_callback(self, data: salobj.BaseMsgType) -> None:
        """Callback for actuators telemetry.

        Output actuators telemetry data if the values have changed enough
        to be interesting.

        Parameters
        ----------
        data : self.controller.tel_actuators.DataType.
            Actuators data.
        """
        if (
            self.previous_tel_actuators is not None  # type: ignore[has-type]
            and np.allclose(
                self.previous_tel_actuators.calibrated, data.calibrated, atol=1  # type: ignore[has-type]
            )
        ):
            return
        self.previous_tel_actuators = data
        print(f"actuators: {self.format_data(data)}")

    async def tel_application_callback(self, data: salobj.BaseMsgType) -> None:
        """Callback for the application telemetry.

        Output application telemetry if the values have changed enough
        to be interesting.

        Parameters
        ----------
        data : self.controller.tel_application.DataType.
            Actuators data.
        """
        if (
            self.previous_tel_application is not None  # type: ignore[has-type]
            and self.positions_close(
                self.previous_tel_application.position, data.position  # type: ignore[has-type]
            )
            and np.array_equal(data.demand, self.previous_tel_application.demand)  # type: ignore[has-type]
        ):
            return
        self.previous_tel_application = data
        print(f"application: {self.format_data(data)}")


def command_hexapod() -> None:
    """Run the hexapod commander."""
    asyncio.run(HexapodCommander.amain(index=True))
