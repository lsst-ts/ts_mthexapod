.. py:currentmodule:: lsst.ts.mthexapod

.. _lsst.ts.mthexapod.version_history:

###############
Version History
###############

v1.4.3
------

* Improve the way to trigger the new movement in the compensation mode.

v1.4.2
------

* Support the ESS temperature sensor for M2 hexapod.

v1.4.1
------

* Publish the error code event of interlock-open when the CSC is in fault.

v1.4.0
------

* Update the CSC to support the movements in steps.

v1.3.8
------

* Check the delta strut length before the movement.

v1.3.7
------

* Improve the ``setup.py`` to support the version of Python 3.11 and 3.12.

v1.3.6
------

* Add scipy to conda recipe.

v1.3.5
------

* Translate the Simulink calculation between the hexapod position and strut position.
* Add the **kinematics.rst** and **trajectory.rst**.

v1.3.4
------

* Remove the **ts_idl**.
* Improve the tests.

v1.3.3
------

* Idle (disable) the controller automatically.

v1.3.2
------

* In hexapod_csc:

  * Moved code that was wrongly put in monitor_camera_filter back into configure, where it belongs.
  
  * Added some logging and error handling in monitor_camera_filter.
  
  * Fixed monitor_camera_filter use of self.Remote, instead of salobj.Remote.
  
  * Fixed how filter offset is applied.
  
    Instead of adding it to the compensated position, add it to the compensation offset.
  
  * Fixed issue in get_filter_offset with variable not being defined.
  
  * Updated compensation_loop to send the CSC to Fault if compensation fails.
  
    This current behavior is actually not correct, CSCs should go to Fault if something unnexpected happens.
    This was causing issues at night as we were operating for long periods with compensation off and having it get enabled by a slew operation, causing the hexapod to jump position.
    See OBS-648 for more information.

  * Updated wait_stopped, to wait for n_telemetry consecutive telemetry values whith stationary hexapod.

  * Added a new context manager to handle the CSC going to fault due to an operational level error, instead of a hardware problem.
    
    The context manager will ensure that the hexapod controller is stopped and in standby before sending the CSC to Fault.

  * Updated get_filter_offset to raise an exception if the requested filter is not in the list of filter offsets.
    
    This will cause the CSC to go to Fault, instead of falling back to having a 0.0 offset for an undefined filter.

v1.3.1
------

* Fix the race condition in ``HexapodCsc._move()``.

v1.3.0
------

* Add filter_offsets to account for filter thickness defocus.

v1.2.3
------

* Add the ``enable_lut_temperature`` to the **config_schema.py**.
* Read the temperature sensor data for the camera hexapod.

v1.2.2
------

* Remove the workaround of ts_xml backward compatibility.

v1.2.1
------

* Subscribe the telemetry of rotator and mount in compensation mode.
If the target event is not available, the CSC will use the telemetry instead.

v1.2.0
------

* Adapt the simplified state machine and update interface with the controller.

Requires:

* ts_hexapod_controller 1.6.0
* ts_hexrotcomm 1.3.0

v1.1.2
------

* Reformat code with black.
* Update the version of ts-conda-build to 0.4 in the conda recipe.
* Remove the workaround code of backward compatibility.

v1.1.1
------

* Support the **mypy**.

v1.1.0
------

* Fix the telemetry.
* Update the ``.ts_pre_commit_config.yaml``.
* Import the enums from **ts_xml** instead of **ts_idl**.

v1.0.4
------

* Fix the test case: ``test_offset_with_compensation()`` when using Python 3.11.
* Improve the test case: ``test_move_translate()``.

v1.0.3
------

* Add {{python}} to conda recipe.

v1.0.2
------

* Temporarily disable a unit test case that hangs on Python 3.11.

Requires:

* ts_hexapod_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 14.

v1.0.1
------

* `HexapodCsc`: assume that ``positionError`` is present in the ``actuators`` telemetry topic (DM-36424).

Requires:

* ts_hexapod_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 14.

v1.0.0
------

* Use ts_pre_commit_conf.
* ``Jenkinsfile``: use new shared library.
* Remove scons support.

Requires:

* ts_hexapod_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 14.

v0.28.1
-------

* pre-commit: update black to 23.1.0, isort to 5.12.0, mypy to 1.0.0, and pre-commit-hooks to v4.4.0.
* ``Jenkinsfile``: do not run as root.

Requires:

* ts_hexapod_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 11.

v0.28.0
-------

* Write the ``positionError`` field of the actuators telemetry topic, if present.
  This field will be added to ts_xml 12.1.
* Make unit test test_move_interrupt_move_immediately in test_csc.py more robust.
* Fix Jenkins CI file by changing HOME to WHOME everywhere except the cleanup section.
* Add setupRequired(ts_config_mttcs) to the ups table file.

Requires:

* ts_hexapod_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 11.

v0.27.0
-------

* Rename command-line scripts to remove ".py" suffix.
* `HexapodCsc`: call ``super().start()`` at the beginning of the start method.
  This requires ts_salobj 7.1.
* Build with pyproject.toml.
* Modernize the continuous integration ``Jenkinsfile``.

Requires:

* ts_hexapod_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 11.

v0.26.0
-------

* Update for ts_hexapod_controller 1.4.0, which is required.
  This version reports motor currents and bus voltages.
* Report motor currents and bus voltages in the electrical telemetry topic.
* Make a unit test more robust.

Requires:

* ts_hexapod_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7
* ts_idl 3.4
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 11.

v0.25.0
-------

* Update for ts_salobj v7, ts_xml 11, and ts_hexrotcomm 0.29, all of which are required.

Requires:

* ts_hexapod_controller 1.3.0
* ts_hexrotcomm 0.29
* ts_salobj 7
* ts_idl 3.4
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 11.

v0.24.0
-------

* Update for ts_hexrotcomm 0.28 and ts_hexapod_controller 1.3.0:

    * Remove support for the sync_pattern field in low-level commands.
    * Remove ``FRAME_ID`` class constants from the `Config` and `Telemetry` structs, because frame IDs are now standardized.
    * Remove support for older (pre-ts_xml 9.2) actuators telemetry.

* Use index_generator from ts_utils instead of the deprecated version in ts_salobj.

Requires:

* ts_hexapod_controller 1.3.0
* ts_hexrotcomm 0.28
* ts_salobj 6.8
* ts_idl 3.4
* ts_xml 10.2
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.23.0
-------

* `HexapodCsc`: changed the interlock event's field from ``detail`` (a string) to ``enabled`` (a boolean).
  Also change `applicationStatus` to a scalar (instead of an array with only the first element nonzero).
  These changes requires ts_xml 10.2.
  These changes also requires ts_hexrotcomm 0.27 (only because it has other changes that require ts_xml 10.2),
  which in turn requires ts_salobj 6.8.
* `CONFIG_SCHEMA`: change ``master`` to ``main`` in the ``$id`` field.

Requires:

* ts_hexapod_controller 1.2.4
* ts_hexrotcomm 0.27
* ts_salobj 6.8
* ts_idl 3.4
* ts_xml 10.2
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.22.1
-------

* Update the command sync patterns for ts_hexapod_controller 1.2.4, which is required.
  This change also requires ts_hexrotcomm 0.25, because ts_hexapod_controller 1.2.4 acknowledges commands.

Requires:

* ts_hexapod_controller 1.2.4
* ts_hexrotcomm 0.25
* ts_salobj 6.3
* ts_idl 3.4
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.22.0
-------

* Update the default host addresses in the CSC config schema to match the new public addresses.

v0.21.2
-------

* `HexapodCommander`: make compatible with ts_xml 10.1 (while retaining backwards compatibility).

Requires:

* ts_hexapod_controller 1.2.0
* ts_hexrotcomm 0.23
* ts_salobj 6.3
* ts_idl 3.4
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.21.1
-------

* Improve handling of NaNs in compensation inputs.
  Treat them as missing data: report them once and keep running the compensation loop.
* Modernized unit tests to use bare asserts.

Requires:

* ts_hexapod_controller 1.2.0
* ts_hexrotcomm 0.23
* ts_salobj 6.3
* ts_idl 3.4
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.21.0
-------

* Update for ts_hexrotcomm v0.23.0, which is required.

    * Run the TCP/IP clients in the CSC and the servers in the mock controller.
    * Disassociated controller state from CSC state.
      As part of the ``standby`` command the CSC connects to the low-level controller.
      As part of the ``enable`` command the CSC attempts to enable the low-level controller
      (including resetting errors if the low-level controller is in fault state).
    * The CSC is no longer alive in the OFFLINE state, and no longer supports the enterControl command.
    * Added ``host``, ``port``, and ``connection_timeout`` fields to the CSC configuration.

* Update the schema to v2 because it has new host and port fields
  (which must be specified if you specify other hexapod-specific settings, such as compensation coefficients).
* Update to use `lsst.ts.idl.enums.MTHexapod.ErrorCode`, which requires ts_idl 3.4.
* setup.cfg: add an [options] section.

Requires:

* ts_hexapod_controller 1.2.0
* ts_hexrotcomm 0.23
* ts_salobj 6.3
* ts_idl 3.4
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.20.0
-------

* Handle updated configuration and telemetry messages from low-level controller 1.1.8, which is required.
* Set the ``timestamp`` field in ``encoders`` telemetry topic, if the field is present.
  This field will be added in ts_xml 10.0.
  This change requires ts_hexrotcomm 0.20 (the time in low-level message headers is TAI) for correct values.

Requires:

* ts_hexapod_controller 1.1.8
* ts_hexrotcomm 0.20
* ts_salobj 6.3
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.19.0
-------

Changes:

* Add ``min_compensation_adjustment`` CSC configuration parameter.
  See the config schema and User Guide for details.

Requires:

* ts_hexrotcomm 0.19
* ts_salobj 6.3
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.18.0
-------

Changes:

* This version requires ts_hexrotcomm 0.19, because it handles another change
  to the low-level controller TCP/IP interface that was made at the same time
  (removing the two MJD fields from message headers).
* Update for changes to the low-level controller TCP/IP interface:

    * `Config`: remove LUT entries
    * `Telemetry`: remove 5 unused fields.

Requires:

* ts_hexrotcomm 0.19
* ts_salobj 6.3
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.17.1
-------

Changes:

* This version requires ts_hexrotcomm 0.18.
* `CscHexapod` bug fix: the ``move`` and ``offset`` commands were rejected if actuators were moving.
* `CscHexapod` bug fix: ``stop``, ``move``, and ``offset`` still did not reliably interrupt a move.

Requires:

* ts_hexrotcomm 0.18
* ts_salobj 6.3
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.17.0
-------

Deprecations:

* The actuatorInPosition event is deprecated (because the CSC does not receive the necessary information)
  and is no longer published.

Changes:

* Fix the inPosition event.
  The code now expects a single value for application_status from the low-level controller
  and ignores the unused 5 extra values. These unused values will go away in a later update
  (which requires a corresponding update to the low-level controller).

Requires:

* ts_hexrotcomm 0.18
* ts_salobj 6.3
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.16.0
-------

Changes:

* Make moves and offsets more reliable: if the hexapod is moving, stop it before issuing the new move command.
  This change requires ts_hexrotcomm v0.18.

Requires:

* ts_hexrotcomm 0.18
* ts_salobj 6.3
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.15.1
-------

Changes:

* Use `unittest.IsolatedAsyncioTestCase` instead of the abandoned asynctest package.
* Format the code with black 20.8b1.

Requires:

* ts_hexrotcomm 0.16
* ts_salobj 6.3
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.15.0
-------

Changes:

* `RotatorCsc`: save the configuration schema in code instead of a separate .yaml file.
  This requires ts_salobj 6.3 and ts_hexrotcomm 0.16.
* Delete obsolete file ``schema/MTRotator.yaml``.
* Users's Guide: improve the information for switching from GUI to DDS mode.

Requires:

* ts_hexrotcomm 0.16
* ts_salobj 6.3
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.14.0
-------

Changes:

* `MTHexapodCsc`: removed the ``moveToReference`` command.
  The associated values should be part of the compensation model coefficients.
* `MTHexapodCsc`: set class variable ``version``, which sets the ``cscVersion`` field of the ``softwareVersions`` event.
* Configuration schema: update to require azimuth and rotation coefficients.
* `SimpleHexapod`: use a safer way to copy the ``mirror_positions`` argument.
  This avoids issues in case the user changes the argument after creating the object (an unlikely scenario).
* Modernize ``doc/conf.py`` for documenteer 0.6.

Requires:

* ts_hexrotcomm 0.13
* ts_salobj 6.1
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.13.0
-------

Changes:

* Updated to use device-specific TCP/IP ports.
  This requires ts_hexrotcomm v0.14.

Requires:

* ts_hexrotcomm 0.14
* ts_salobj 6.1
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.12.1
-------

Changes:

* A fix for tx_xml 7.1 (required): use MTMount XML instead of NewMTMount.
* Fix the Jenkins build: build MTMount and MTRotator IDL files in addition to MTHexapod.

Requires:

* ts_hexrotcomm 0.13
* ts_salobj 6.1
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod, MTMount, and MTRotator IDL files, e.g. made using ``make_idl_files.py MTHexapod MTMount MTRotator``

v0.12.0
-------

Changes:

* Update for ts_xml 7.1 and ts_hexrotcomm 0.13 (both are required).
* Overhaul the way compensation is handled.
* Add the ``setCompensationMode`` command and remove the ``moveWithCompensation`` command.
* Rename the ``pivot`` command to ``setPivot``, for consistency.
* Replace the ``target`` event with ``uncompensatedPosition`` and ``compensatedPosition`` events, because ``target`` was ambiguous.
* Add the ``moveToReference`` command to move to the configured reference position.
* Add ``compensation_interval`` and ``reference_position`` entries to the configuration.
  The former is common to both hexapods, the latter is specific each hexapod.

Requires:

* ts_hexrotcomm 0.13
* ts_salobj 6.1
* ts_idl 2.2
* ts_xml 7.1
* MTHexapod IDL files, e.g. made using ``make_idl_files.py MTHexapod``

v0.11.1
-------

Changes:

* Update Jenkinsfile.conda to use the shared library.
* Pin the versions of ts_idl and ts_salobj in conda/meta.yaml.

Requires:

* ts_hexrotcomm 0.12
* ts_salobj 6.1
* ts_idl 2.2
* ts_xml 7
* MTHexapod IDL files, e.g. made using ``make_idl_files.py MTHexapod``

v0.11.0
-------

Changes:

* Update to use and require ts_hexrotcomm 0.12:

    * Add argument ``curr_tai`` to `MockMTHexapodController.update_telemetry` and use it.

Requires:

* ts_hexrotcomm 0.12
* ts_salobj 6.1
* ts_idl 2.2
* ts_xml 7
* MTHexapod IDL files, e.g. made using ``make_idl_files.py MTHexapod``

v0.10.0
-------

Changes:

* Updated to use and require ts_xml 7, ts_idl 2.2, and ts_hexrotcomm 0.11:

    * Rename SAL component and ts_idl enum module ``Hexapod`` to ``MTHexapod``.

* Renamed the package to ``ts_mthexapod``.

Requires:

* ts_hexrotcomm 0.11
* ts_salobj 6.1
* ts_idl 2.2
* ts_xml 7
* MTHexapod IDL files, e.g. made using ``make_idl_files.py MTHexapod``

v0.9.0
------

Changes:

* Updated to use and require ts_salobj 6.1 and ts_hexrotcomm 0.10.
* Update the handling of initial_state in `HexapodCsc`:

    * If initial_state != OFFLINE then report all transitional summary states and controller states at startup.
    * Require initial_state = OFFLINE unless simulating.

Requires:

* ts_hexrotcomm 0.10
* ts_salobj 6.1
* ts_idl 2
* ts_xml 6.2
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators 2

v0.8.0
------

Major Changes:

* Add support for compensated moves, where compensation is done in the CSC instead of in the low-level controller.
* Add a data fitter for compensation data. See the README in new directory ``fitter``.
* Overhaul the SAL API.
* Modernize the documentation.

Minor Changes:

* Add missing ``config_dir`` constructor argument to `HexapodCsc`.
* Use `lsst.ts.salobj.BaseCscTestCase` and `lsst.ts.salobj.CscCommander` instead of the versions in ts_hexrotcomm.
* Add several ``<x>_jitter`` attributes to `MockMTHexapodController` to clarify the mount of jitter added to measured values.
* Use corrected spelling of ``Hexapod.ApplicationStatus.SAFETY_INTERLOCK``.
  This requires ts_idl 1.4 or later.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.15 or 6
* ts_idl 1.4 (for salobj 5) or 2 (for salobj 6)
* ts_xml 6.2
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators 2

v0.7.0
------

Changes:

* Make `HexapodCsc` configurable.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.15
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators 2

v0.6.0
------

Changes:

* Update for ts_simactuators 2.

Requires:

* ts_hexrotcomm 0.5
* ts_salobj 5.15
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators 2

v0.5.4
------

Changes:

* Add black to conda test dependencies.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5.11
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators

v0.5.3
------

Changes:

* Add ``tests/test_black.py`` to verify that files are formatted with black.
  This requires ts_salobj 5.11 or later.
* Update ``.travis.yml`` to remove ``sudo: false`` to github travis checks pass once again.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5.11
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators

v0.5.2
------

* Fix flake8 violations.
* Add Jenkinsfile for CI job.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators

v0.5.1
------

* Include conda package build configuration.
* Added a Jenkinsfile to support continuous integration and to build conda packages.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators

v0.5.0
------

Use `lsst.ts.simactuators.PointToPointActuator` instead of an internal copy.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators

v0.4.0
------

Major changes:

* Use correct hexapod geometry in the simulation mode.
  The motion limits and reported actuator lengths and limits should now be much more realistic.
* Code formatted by ``black``, with a pre-commit hook to enforce this.
  See the README file for configuration instructions.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``

v0.3.1
------

Version 0.3.1

Add a link to the docs in the README file.
Add a unit test.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``

v0.3.0
------

Update for changes to the XML.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``

v0.2.2
------

The first version we tested against the real hexapod controller!

Requires:
* ts_hexrotcomm v0.1.0
* ts_salobj 5
* ts_idl 1
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
