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

import jsonschema
import numpy as np
from numpy.testing import assert_allclose
import pytest

from lsst.ts import salobj
from lsst.ts import mthexapod


class ValidationTestCase(unittest.TestCase):
    """Test validation of the config schema."""

    def setUp(self):
        self.schema = mthexapod.CONFIG_SCHEMA
        self.validator = salobj.DefaultingValidator(schema=self.schema)
        self.instance_names = ("camera_config", "m2_config")

    def test_default(self):
        result = self.validator.validate(None)
        assert result["compensation_interval"] == 0.2
        for instance_name in self.instance_names:
            instance = result[instance_name]
            for coeffs_prefix in ("elevation", "azimuth", "rotation", "temperature"):
                coeffs_name = f"{coeffs_prefix}_coeffs"
                assert len(instance[coeffs_name]) == 6
                assert instance[coeffs_name] == [[0], [0], [0], [0], [0], [0]]
            assert_allclose(
                instance["min_compensation_adjustment"], [1, 1, 1, 1e-4, 1e-4, 1e-4]
            )
            assert instance["min_temperature"] <= 0
            assert instance["max_temperature"] >= 20

    def test_minmax_temperature_specified(self):
        defaults = self.validator.validate(None)
        for instance_name in self.instance_names:
            for name, delta in (("min_temperature", -1.5), ("max_temperature", 3.1)):
                data = copy.deepcopy(defaults)
                new_value = data[instance_name][name] + delta
                data[instance_name][name] = new_value
                result = self.validator.validate(data)
                assert result[instance_name][name] == pytest.approx(new_value)

    def test_coeffs_specified(self):
        defaults = self.validator.validate(None)
        # Test validating a dict that only has data for one instance,
        # and with a dict that has data for all instances.
        for just_one_instance in (False, True):
            for instance_name in self.instance_names:
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
                        data = {instance_name: copy.deepcopy(defaults[instance_name])}
                    else:
                        data = copy.deepcopy(defaults)
                    name = f"{short_name}_coeffs"
                    data[instance_name][name] = coeffs
                    result = self.validator.validate(data)
                    for i in range(6):
                        np.testing.assert_allclose(
                            result[instance_name][name][i], coeffs[i]
                        )

    def test_bad_coeffs(self):
        defaults = self.validator.validate(None)
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

    def test_missing_data(self):
        """The defaults for a given instance only work if no values
        are specified for that instance.
        """
        defaults = self.validator.validate(None)
        for instance_name in self.instance_names:
            for name in defaults[instance_name]:
                data = copy.deepcopy(defaults)
                del data[instance_name][name]
                with pytest.raises(jsonschema.exceptions.ValidationError):
                    self.validator.validate(data)
