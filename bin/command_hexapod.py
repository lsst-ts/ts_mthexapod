#!/usr/bin/env python
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
import functools
import sys

from lsst.ts import salobj
from lsst.ts.idl.enums import Hexapod

STD_TIMEOUT = 5  # timeout for command ack


async def stdin_generator():
    """Thanks to http://blog.mathieu-leplatre.info
    """
    loop = asyncio.get_event_loop()
    reader = asyncio.StreamReader(loop=loop)
    reader_protocol = asyncio.StreamReaderProtocol(reader)
    await loop.connect_read_pipe(lambda: reader_protocol, sys.stdin)
    while True:
        line = await reader.readline()
        if not line:  # EOF.
            break
        yield line.decode("utf-8").strip()


class RemoteWatcher:
    """Monitor a Hexapod remote.
    """
    def __init__(self, remote):
        self.remote = remote

        for name in remote.salinfo.event_names:
            topic = getattr(self.remote, f"evt_{name}")
            func = getattr(self, f"{name}_callback", None)
            if func is None:
                func = functools.partial(self.event_callback, name=name)
            setattr(topic, "callback", functools.partial(self.event_callback, name=name))

        for name in remote.salinfo.telemetry_names:
            setattr(self, f"prev_{name}", None)
            topic = getattr(self.remote, f"tel_{name}")
            setattr(topic, "callback", functools.partial(self.telemetry_callback, name=name))

    def format_item(self, key, value):
        if isinstance(value, float):
            return f"{key}={value:0.2f}"
        return f"{key}={value}"

    def format_data(self, data):
        return ", ".join(self.format_item(key, value) for key, value in self.get_public_fields(data).items())

    def get_public_fields(self, data):
        return dict((key, value) for key, value in data.get_vars().items()
                    if not key.startswith("private_") and key != "timestamp")

    def event_callback(self, data, name):
        """Generic callback for events."""
        print(f"{name}: {self.format_data(data)}")

    def telemetry_callback(self, data, name):
        """Generic callback for telemetry."""
        public_fields = self.get_public_fields(data)
        if public_fields != getattr(self, f"prev_{name}"):
            setattr(self, f"prev_{name}", public_fields)
            print(f"{name}: {self.format_data(data)}")

    def controllerState_callback(self, data):
        print(f"controllerState: state={Hexapod.ControllerState(data.controllerState)!r}; "
              f"offline_substate={Hexapod.OfflineSubstate(data.offlineSubstate)!r}; "
              f"enabled_substate={Hexapod.EnabledSubstate(data.enabledSubstate)!r}; "
              f"applicationStatus={data.applicationStatus}")

    def summaryState_callback(self, data):
        print(f"summaryState={salobj.State(data.summaryState)!r}")


help_text = """Special commands:
* help  # Print this help.
* exit  # Quit the interpreter, after stopping existing ramp, sine or motion.

State transitions commands (none take arguments):
* enterControl
* start
* enable
* disable
* standby
* exitControl
* clearError

Other commands and arguments:
* configureAcceleration acc                         # Set acceleration: µm/s2
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
  exit"""


def check_arguments(args, num_required):
    if len(args) != num_required:
        raise ValueError(f"requires {num_required} arguments; {len(args)} provided")


async def command_loop(index):
    print("Creating domain and remote.")
    async with salobj.Domain() as domain:
        remote = salobj.Remote(domain=domain, name="Hexapod", index=index)
        try:
            RemoteWatcher(remote)
            print("Waiting for the remote to connect.")
            await remote.start_task

            no_arguments_commands = ("enterControl", "start", "enable", "disable",
                                     "move", "standby", "exitControl", "clearError",
                                     "stop")

            print(f"\n{help_text}")
            async for line in stdin_generator():
                # Strip trailing comment, if any.
                if "#" in line:
                    line = line.split("#", maxsplit=1)[0].strip()
                if not line:
                    continue
                tokens = line.split()
                command = tokens[0]
                args = [float(token) for token in tokens[1:]]
                try:
                    if command == "exit":
                        break
                    elif command == "help":
                        print(help_text)
                    elif command in no_arguments_commands:
                        remote_command = getattr(remote, f"cmd_{command}")
                        await remote_command.start(timeout=STD_TIMEOUT)
                    elif command == "configureAcceleration":
                        check_arguments(args, 1)
                        await remote.cmd_configureAcceleration.set_start(accmax=args[0], timeout=STD_TIMEOUT)
                    elif command == "configureLimits":
                        check_arguments(args, 6)
                        await remote.cmd_positionSet.set_start(xymax=args[0],
                                                               zmin=args[1],
                                                               zmax=args[2],
                                                               uvmax=args[3],
                                                               wmin=args[4],
                                                               wmax=args[5],
                                                               timeout=STD_TIMEOUT)
                    elif command == "configureVelocity":
                        check_arguments(args, 4)
                        await remote.cmd_configureVelocity.set_start(xymax=args[0],
                                                                     rxrymax=args[1],
                                                                     zmax=args[2],
                                                                     rzmax=args[3],
                                                                     timeout=STD_TIMEOUT)
                    elif command == "moveLUT":
                        check_arguments(args, 3)
                        await remote.cmd_moveLUT.set_start(az=args[0],
                                                           elev=args[1],
                                                           temp=args[2],
                                                           timeout=STD_TIMEOUT)
                    elif command == "offset":
                        check_arguments(args, 7)
                        await remote.cmd_offset.set_start(x=args[0],
                                                          y=args[1],
                                                          z=args[2],
                                                          u=args[3],
                                                          v=args[4],
                                                          w=args[5],
                                                          sync=args[6] != 0,
                                                          timeout=STD_TIMEOUT)
                    elif command == "pivot":
                        check_arguments(args, 3)
                        await remote.cmd_pivot.set_start(x=args[0],
                                                         y=args[1],
                                                         z=args[2],
                                                         timeout=STD_TIMEOUT)
                    elif command == "positionSet":
                        check_arguments(args, 7)
                        await remote.cmd_positionSet.set_start(x=args[0],
                                                               y=args[1],
                                                               z=args[2],
                                                               u=args[3],
                                                               v=args[4],
                                                               w=args[5],
                                                               sync=args[6] != 0,
                                                               timeout=STD_TIMEOUT)
                    elif command == "test":
                        check_arguments(args, 2)
                        await remote.cmd_positionSet.set_start(ivalue1=int(args[0]),
                                                               ivalue2=int(args[1]),
                                                               timeout=STD_TIMEOUT)
                    else:
                        print(f"Unrecognized command {command}")
                        continue
                except Exception as e:
                    print(f"Command {command} failed: {e}")
                    continue
                print(f"Finished command {command}")
        finally:
            print("Exiting; please wait.")
            try:
                await remote.cmd_stop.start(timeout=STD_TIMEOUT)
            except Exception:
                # Best effort attempt to stop. The command may not
                # even be valid in the current state.
                pass
            await remote.close()

nargs = len(sys.argv)
if nargs != 2:
    print("Must specify the hexapod index: 1 for Camera, 2 for M2")
    sys.exit(1)
index = int(sys.argv[1])
if index not in (1, 2):
    print("Must specify the hexapod index: 1 for Camera, 2 for M2")
    sys.exit(1)
asyncio.run(command_loop(index))
