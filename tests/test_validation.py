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
import unittest
import pathlib

import numpy as np
import jsonschema
import yaml

from lsst.ts import salobj


class ValidationTestCase(unittest.TestCase):
    """Test validation of the config schema."""

    def setUp(self):
        schemaname = "MTHexapod.yaml"
        schemapath = pathlib.Path(__file__).parents[1].joinpath("schema", schemaname)
        with open(schemapath, "r") as f:
            rawschema = f.read()
        self.schema = yaml.safe_load(rawschema)
        self.validator = salobj.DefaultingValidator(schema=self.schema)
        self.instance_names = ("camera_config", "m2_config")

    def test_default(self):
        result = self.validator.validate(None)
        self.assertEqual(result["compensation_interval"], 0.2)
        for instance in self.instance_names:
            self.assertEqual(len(result[instance]["elevation_coeffs"]), 6)
            self.assertEqual(len(result[instance]["azimuth_coeffs"]), 6)
            self.assertEqual(len(result[instance]["rotation_coeffs"]), 6)
            self.assertEqual(len(result[instance]["temperature_coeffs"]), 6)
            self.assertLessEqual(result[instance]["min_temperature"], 0)
            self.assertGreaterEqual(result[instance]["max_temperature"], 20)

    def test_minmax_temperature_specified(self):
        defaults = self.validator.validate(None)
        for instance in self.instance_names:
            for name, delta in (("min_temperature", -1.5), ("max_temperature", 3.1)):
                data = copy.deepcopy(defaults)
                value = data[instance][name] + delta
                data[instance][name] = value
                result = self.validator.validate(data)
                self.assertAlmostEqual(result[instance][name], value)

    def test_coeffs_specified(self):
        defaults = self.validator.validate(None)
        # Test validating a dict that only has data for one instance,
        # and with a dict that has data for all instances.
        for just_one_instance in (False, True):
            for instance in self.instance_names:
                for short_name, coeffs in itertools.product(
                    ("elevation", "azimuth", "rotation", "temperature"),
                    (
                        [[0]] * 6,
                        [
                            [1.1, 1.2],
                            [2.1, 2.2, 2.3],
                            [3.1, 3.2, 3.3, 3.4],
                            [4.1, 4.2, 4.3, 4.4, 4.5],
                            [5.1, 5.2, 5.3, 5.4, 5.5, 5.6],
                            [6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7],
                        ],
                    ),
                ):
                    if just_one_instance:
                        data = {instance: copy.deepcopy(defaults[instance])}
                    else:
                        data = copy.deepcopy(defaults)
                    name = f"{short_name}_coeffs"
                    data[instance][name] = coeffs
                    result = self.validator.validate(data)
                    for i in range(6):
                        np.testing.assert_allclose(result[instance][name][i], coeffs[i])

    def test_bad_coeffs(self):
        defaults = self.validator.validate(None)
        for instance in self.instance_names:
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
                data[instance][name] = bad_coeffs
                with self.assertRaises(jsonschema.exceptions.ValidationError):
                    self.validator.validate(data)


if __name__ == "__main__":
    unittest.main()
