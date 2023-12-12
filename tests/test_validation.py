# This file is part of ts_ATDomeTrajectory.
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

import copy
import itertools
import pathlib
import unittest

import jsonschema
import pytest
import yaml
from lsst.ts import mthexapod, salobj

TEST_CONFIG_DIR = pathlib.Path(__file__).parent / "data" / "config"


class ValidationTestCase(unittest.TestCase):
    """Test validation of the config schema."""

    def setUp(self) -> None:
        self.schema = mthexapod.CONFIG_SCHEMA
        self.validator = salobj.StandardValidator(schema=self.schema)
        self.instance_names = ("camera_config", "m2_config")

    def load_config(self) -> dict:
        with open(TEST_CONFIG_DIR / "_init.yaml", "r") as f:
            raw_config = f.read()
        return yaml.safe_load(raw_config)

    def test_basics(self) -> None:
        config = self.load_config()
        self.validator.validate(config)

    def test_missing_data(self) -> None:
        config = self.load_config()
        for key in config:
            with self.subTest(key=key):
                bad_config = config.copy()
                del bad_config[key]
                with pytest.raises(jsonschema.ValidationError):
                    self.validator.validate(bad_config)

        for instance_name in self.instance_names:
            for key in config[instance_name]:
                with self.subTest(instance_name=instance_name, key=key):
                    bad_config = config.copy()
                    bad_config[instance_name] = config[instance_name].copy()
                    del bad_config[instance_name][key]
                    with pytest.raises(jsonschema.ValidationError):
                        self.validator.validate(bad_config)

    def test_bad_coeffs(self) -> None:
        defaults = self.load_config()
        for instance_name in self.instance_names:
            for short_name, bad_coeffs in itertools.product(
                ("elevation", "temperature"),
                (
                    [[0]] * 5,
                    [[0]] * 7,
                    [[], [0], [0], [0], [0], [0]],
                    [[0], [], [0], [0], [0], [0]],
                    [[0], [0], [], [0], [0], [0]],
                    [[0], [0], [0], [], [0], [0]],
                    [[0], [0], [0], [0], [], [0]],
                    [[0], [0], [0], [0], [0], []],
                ),
            ):
                data = copy.deepcopy(defaults)
                name = f"{short_name}_coeffs"
                data[instance_name][name] = bad_coeffs
                with pytest.raises(jsonschema.exceptions.ValidationError):
                    self.validator.validate(data)
