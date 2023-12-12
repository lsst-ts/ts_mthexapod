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

import types
import unittest

import pytest
from lsst.ts import mthexapod


class BaseTestCase(unittest.TestCase):
    def assert_dataclass_data_equal(
        self, instance: types.SimpleNamespace, data_dict: dict
    ) -> None:
        """Assert that the fields of a dataclass instance are equal to
        the specified values.
        """
        instance_data = vars(instance)
        assert instance_data == data_dict

    def assert_dataclass_data_almost_equal(
        self, instance: types.SimpleNamespace, data_dict: dict
    ) -> None:
        """Assert that the fields of a dataclass instance are almost equal to
        the specified values.
        """
        instance_data = vars(instance)
        assert set(instance_data) == set(data_dict)
        for name in instance_data:
            assert instance_data[name] == pytest.approx(data_dict[name])

    def test_compensation_inputs(self) -> None:
        base_data = dict(
            elevation=11.0,
            azimuth=22.0,
            rotation=-33.0,
            temperature=-44.0,
        )

        # Test elevations in range.
        for good_elevation in (0.0, 45.0, 90.0):
            data = base_data.copy()
            data["elevation"] = good_elevation
            for comp_input in (
                mthexapod.CompensationInputs(**data),
                mthexapod.CompensationInputs(*data.values()),
                mthexapod.CompensationInputs.from_struct(types.SimpleNamespace(**data)),
            ):
                self.assert_dataclass_data_almost_equal(comp_input, data)

        # Test elevations out of range.
        for bad_elevation in (-0.001, 90.001):
            data = base_data.copy()
            data["elevation"] = bad_elevation
            with pytest.raises(ValueError):
                mthexapod.CompensationInputs(**data)

        # Test azimuth wrap.
        for good_azimuth, wrapped_azimuth in (
            (-500, 220),
            (-1, 359),
            (0, 0),
            (270, 270),
            (359.999, 359.999),
            (360, 0),
            (900, 180),
        ):
            data = base_data.copy()
            data["azimuth"] = good_azimuth
            wrapped_data = data.copy()
            wrapped_data["azimuth"] = wrapped_azimuth
            comp_input = mthexapod.CompensationInputs(**data)
            self.assert_dataclass_data_almost_equal(comp_input, wrapped_data)

        # Test rotation wrap.
        for good_rotation, wrapped_rotation in (
            (-500, -140),
            (-181, 179),
            (-179.999, -179.999),
            (0, 0),
            (179.999, 179.999),
            (270, -90),
            (899.999, 179.999),
        ):
            data = base_data.copy()
            data["rotation"] = good_rotation
            wrapped_data = data.copy()
            wrapped_data["rotation"] = wrapped_rotation
            comp_input = mthexapod.CompensationInputs(**data)
            self.assert_dataclass_data_almost_equal(comp_input, wrapped_data)

    def test_position(self) -> None:
        assert mthexapod.Position.field_names() == ("x", "y", "z", "u", "v", "w")
        data = dict(x=1, y=2, z=3, u=4, v=5, w=6)
        pos1 = mthexapod.Position(**data)
        pos2 = mthexapod.Position(*data.values())
        pos3 = mthexapod.Position.from_struct(types.SimpleNamespace(**data))
        self.assert_dataclass_data_equal(pos1, data)
        self.assert_dataclass_data_equal(pos2, data)
        self.assert_dataclass_data_equal(pos3, data)

        # Test addition and subtraction
        pos12 = pos1 + pos2
        pred_data = {name: value * 2 for name, value in data.items()}
        self.assert_dataclass_data_almost_equal(pos12, pred_data)

        zero_pos = pos1 - pos2
        pred_data = {name: 0 for name, value in data.items()}
        self.assert_dataclass_data_almost_equal(zero_pos, pred_data)

    def test_position_limits(self) -> None:
        assert mthexapod.PositionLimits.field_names() == (
            "maxXY",
            "minZ",
            "maxZ",
            "maxUV",
            "minW",
            "maxW",
        )
        data = dict(maxXY=1, minZ=-2, maxZ=3, maxUV=4, minW=-5, maxW=6)
        poslim1 = mthexapod.PositionLimits(**data)
        poslim2 = mthexapod.PositionLimits(*data.values())
        poslim3 = mthexapod.PositionLimits.from_struct(types.SimpleNamespace(**data))
        self.assert_dataclass_data_equal(poslim1, data)
        self.assert_dataclass_data_equal(poslim2, data)
        self.assert_dataclass_data_equal(poslim3, data)

        for bad_items in (
            dict(maxXY=0),
            dict(maxXY=-0.001),
            dict(minZ=data["maxZ"]),
            dict(maxUV=0),
            dict(maxUV=-0.001),
            dict(minW=data["maxW"]),
        ):
            bad_data = data.copy()
            bad_data.update(bad_items)  # type: ignore[call-overload]
            with pytest.raises(ValueError):
                mthexapod.PositionLimits(**bad_data)
            with pytest.raises(ValueError):
                mthexapod.PositionLimits(*bad_data.values())
            with pytest.raises(ValueError):
                mthexapod.PositionLimits.from_struct(types.SimpleNamespace(**bad_data))
