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

__all__ = ["MockMTHexapodController"]

import dataclasses

import numpy as np

from lsst.ts import hexrotcomm
from lsst.ts.idl.enums.MTHexapod import (
    ControllerState,
    EnabledSubstate,
    ApplicationStatus,
)
from . import base
from . import constants
from . import enums
from . import structs
from . import utils
from . import simple_hexapod

# Maximum time between track commands (seconds)
# The real controller may use 0.15
TRACK_TIMEOUT = 1

# Model motor current as proportional to fractional velocity
# and bus voltage as constant
AMPS_PER_FRAC_SPEED = 1
BUS_VOLTAGE = 100


class MockMTHexapodController(hexrotcomm.BaseMockController):
    """Mock MTHexapod controller that talks over TCP/IP.

    Parameters
    ----------
    index : `SalIndex` or `int`
        SAL index; see `SalIndex` for the allowed values.
    log : `logging.Logger`
        Logger.
    port : `int`, optional
        Command socket port.
    initial_state : `lsst.ts.idl.enums.MTHexapod.ControllerState`, optional
        Initial state of mock controller.

    Notes
    -----
    To start the mock controller:

        ctrl = MockHexapodController(...)
        await ctrl.start_task

    To stop the mock controller:

        await ctrl.close()

    *Known Limitations*

    * The synchronized move parameter is ignored.
      Supporting this would be fairly easy.
    * No lookup table support. Thus MOVE_LUT is the same as
      MOVE_POINT_TO_POINT.
    * Acceleration is treated as instantanous.
    """

    # MTHexapod actuator positions (µm) from
    # "CAMERA HEXAPOD STRUT FLEXURE COORDINATES.xlsx"
    # (a copy is in the doc directory)
    # received from John Andrew 2020-02-13.
    actuator_base_positions = [
        (-227647, 653753, 0),
        (227647, 653753, 0),
        (679990, -129728, 0),
        (452343, -524025, 0),
        (-452343, -524025, 0),
        (-679990, -129728, 0),
    ]
    actuator_mirror_positions = [
        (-472917, 512146, 403918),
        (472917, 512146, 403918),
        (679990, 153485, 403918),
        (207073, -665631, 403918),
        (-207073, -665631, 403918),
        (-679990, 153485, 403918),
    ]
    # Actuator position limits (µm) and speed (µm/second) from
    # https://github.com/lsst-ts/ts_mt_hexRot_middleware/blob/master/config/cam_hex/default.conf  # noqa
    actuator_max_length = 14100
    actuator_min_length = -14100
    actuator_speed = 500

    # Default pivot position (µm). This is merely a guess.
    pivot = (0, 0, 500_000)

    # Encoder resolution (counts/µm). This is merely a guess.
    actuator_encoder_resolution = 10

    def __init__(
        self,
        index,
        log,
        port=0,
        initial_state=ControllerState.OFFLINE,
    ):
        index = enums.SalIndex(index)
        self.max_pos_limits = constants.MAX_POSITION_LIMITS[index]

        # Amplitude of jitter in various measured values,
        # to simulate encoder jitter. This add realism
        # and exercises jitter rejection in HexapodCommander.
        self.xyz_jitter = 0.1  # um
        self.uvw_jitter = 1.0e-6  # deg
        self.strut_jitter = 0.1  # encoder counts

        config = structs.Config()
        config.acceleration_strut = 500
        # Order: xy (um), minZ, max, uv (deg), minW, maxW
        config.pos_limits = dataclasses.astuple(self.max_pos_limits)
        # Order: xy (deg/sec), xy rotation (um/sec), z, z rotation
        config.vel_limits = (
            constants.MAX_LINEAR_VEL_LIMIT,
            constants.MAX_ANGULAR_VEL_LIMIT,
            constants.MAX_LINEAR_VEL_LIMIT,
            constants.MAX_ANGULAR_VEL_LIMIT,
        )
        # Order: x, y, z, u, w, v
        config.pivot = self.pivot
        config.max_displacement_strut = self.actuator_max_length
        config.max_velocity_strut = self.actuator_speed

        self.hexapod = simple_hexapod.SimpleHexapod(
            base_positions=self.actuator_base_positions,
            mirror_positions=self.actuator_mirror_positions,
            pivot=self.pivot,
            min_length=self.actuator_min_length,
            max_length=self.actuator_max_length,
            speed=self.actuator_speed,
        )
        self.move_commanded = False

        telemetry = structs.Telemetry()
        telemetry.commanded_pos = (0,) * 6
        # The position specified by the POSITION_SET command (a `Position`);
        # reset to None after any other command.
        self.set_position = None

        # Dict of command key: command
        extra_commands = {
            (
                enums.CommandCode.SET_ENABLED_SUBSTATE,
                enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
            ): self.do_move_point_to_point,
            # Note: the mock controller ignores the lookup table,
            # so MOVE_LUT is identical to MOVE_POINT_TO_POINT
            (
                enums.CommandCode.SET_ENABLED_SUBSTATE,
                enums.SetEnabledSubstateParam.MOVE_LUT,
            ): self.do_move_point_to_point,
            (
                enums.CommandCode.SET_ENABLED_SUBSTATE,
                enums.SetEnabledSubstateParam.STOP,
            ): self.do_stop,
            enums.CommandCode.POSITION_SET: self.do_position_set,
            enums.CommandCode.SET_PIVOTPOINT: self.do_set_pivotpoint,
            enums.CommandCode.CONFIG_ACCEL: self.do_config_accel,
            enums.CommandCode.CONFIG_LIMITS: self.do_config_limits,
            enums.CommandCode.CONFIG_VEL: self.do_config_vel,
        }

        super().__init__(
            log=log,
            CommandCode=enums.CommandCode,
            extra_commands=extra_commands,
            config=config,
            telemetry=telemetry,
            port=port,
            initial_state=initial_state,
        )

    async def close(self):
        """Kill command and telemetry tasks and close the connections.

        Always safe to call.
        """
        self.hexapod.stop()
        await super().close()

    async def do_config_accel(self, command):
        self.assert_stationary()
        if not 0 < command.param1 <= constants.MAX_ACCEL_LIMIT:
            raise ValueError(
                f"Requested accel limit {command.param1} "
                f"not in range (0, {constants.MAX_ACCEL_LIMIT}]"
            )
        self.config.acceleration_strut = command.param1
        await self.write_config()

    async def do_config_limits(self, command):
        self.assert_stationary()
        limits_values = tuple(getattr(command, f"param{i + 1}") for i in range(6))
        limits = base.PositionLimits(*limits_values)
        utils.check_new_position_limits(limits=limits, max_limits=self.max_pos_limits)
        self.config.pos_limits = limits_values
        await self.write_config()

    async def do_config_vel(self, command):
        self.assert_stationary()
        utils.check_positive_value(command.param1, "xy", constants.MAX_LINEAR_VEL_LIMIT)
        utils.check_positive_value(
            command.param2, "uv", constants.MAX_ANGULAR_VEL_LIMIT
        )
        utils.check_positive_value(command.param3, "z", constants.MAX_LINEAR_VEL_LIMIT)
        utils.check_positive_value(command.param4, "w", constants.MAX_ANGULAR_VEL_LIMIT)
        self.config.vel_limits = (
            command.param1,
            command.param2,
            command.param3,
            command.param4,
        )
        await self.write_config()

    async def do_offset(self, command):
        self.assert_stationary()

    async def do_position_set(self, command):
        self.assert_stationary()
        position_values = tuple(getattr(command, f"param{i + 1}") for i in range(6))
        position = base.Position(*position_values)
        limits = base.PositionLimits(*self.config.pos_limits)
        utils.check_position(position=position, limits=limits)
        self.set_position = position

    async def do_set_pivotpoint(self, command):
        self.assert_stationary()
        self.config.pivot = (command.param1, command.param2, command.param3)
        await self.write_config()

    async def do_stop(self, command):
        self.assert_state(ControllerState.ENABLED)
        self.hexapod.stop()
        self.telemetry.enabled_substate = EnabledSubstate.STATIONARY
        self.move_commanded = False

    async def do_move_point_to_point(self, command):
        self.assert_stationary()
        if self.set_position is None:
            raise RuntimeError(
                "Must call POSITION_SET before calling MOVE_POINT_TO_POINT"
            )
        self.telemetry.commanded_pos = dataclasses.astuple(self.set_position)
        duration = self.hexapod.move(
            pos=self.telemetry.commanded_pos[0:3],
            xyzrot=self.telemetry.commanded_pos[3:6],
        )
        self.telemetry.commanded_length = tuple(
            actuator.end_position for actuator in self.hexapod.actuators
        )
        self.telemetry.enabled_substate = EnabledSubstate.MOVING_POINT_TO_POINT
        self.move_commanded = True
        self.log.debug(
            "Move to %s; move duration %0.1f",
            self.telemetry.commanded_pos[:],
            duration,
        )

    async def end_run_command(self, command, cmd_method):
        if cmd_method != self.do_position_set:
            self.set_position = None

    async def update_telemetry(self, curr_tai):
        try:
            self.telemetry.status_word = (0,) * 6
            self.telemetry.latching_fault_status_register = (0,) * 6
            self.telemetry.copley_fault_status_register = (0,) * 6
            if self.telemetry.state != ControllerState.ENABLED:
                self.move_commanded = False
            axes_in_position = [
                self.move_commanded and not actuator.moving(curr_tai)
                for actuator in self.hexapod.actuators
            ]
            in_position = all(axes_in_position)
            self.telemetry.application_status = (
                int(in_position) * ApplicationStatus.MOVE_COMPLETE
                | ApplicationStatus.DDS_COMMAND_SOURCE
            )
            self.telemetry.input_pin_states = (0,) * 3

            # TODO DM-31290: uncomment these lines when the data is available
            # Model current and volage as proportional to fractional velocity
            # axes_frac_velocity = [
            #     actuator.velocity(tai=curr_tai) / actuator.speed
            #     for actuator in self.hexapod.actuators
            # ]
            # self.telemetry.motor_current[:] = np.multiply(
            #     axes_frac_velocity, AMPS_PER_FRAC_SPEED
            # )
            # self.telemetry.bus_voltage[:] = BUS_VOLTAGE

            # state, enabled_substate and offline_substate
            # are all set by set_state
            self.telemetry.test_state = 0
            current_lengths = [
                actuator.position(curr_tai) for actuator in self.hexapod.actuators
            ]
            if self.telemetry.state == ControllerState.ENABLED:
                # Add some fake encoder jitter,
                current_lengths += self.strut_jitter * (np.random.random(6) - 0.5)
            self.telemetry.strut_encoder_raw = tuple(
                pos * self.actuator_encoder_resolution for pos in current_lengths
            )
            self.telemetry.strut_measured_pos_um = tuple(current_lengths)

            # self.telemetry.commanded_pos and commanded_length are both set
            # by MOVE and MOVE_LUT.
            # self.telemetry.measured_xyz and uvw should be based on
            # current position, but SimpleHexapod does not yet support
            # determining orientation from actuator length.
            measured_xyz = np.copy(self.telemetry.commanded_pos[:3])
            measured_uvw = np.copy(self.telemetry.commanded_pos[3:])
            if self.telemetry.state == ControllerState.ENABLED:
                # Add ~0.1 micron jitter to the current positions and
                # ~0.003 arcsec jitter to the current rotations for realism.
                measured_xyz += self.xyz_jitter * (np.random.random(3) - 0.5)
                measured_uvw += self.uvw_jitter * (np.random.random(3) - 0.5)
            self.telemetry.measured_xyz = tuple(measured_xyz)
            self.telemetry.measured_uvw = tuple(measured_uvw)
            if (
                self.telemetry.state == ControllerState.ENABLED
                and self.telemetry.enabled_substate
                == EnabledSubstate.MOVING_POINT_TO_POINT
                and all(axes_in_position)
            ):
                self.telemetry.enabled_substate = EnabledSubstate.STATIONARY
        except Exception:
            self.log.exception("update_telemetry failed; output incomplete telemetry")
