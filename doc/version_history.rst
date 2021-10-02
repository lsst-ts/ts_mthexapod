.. py:currentmodule:: lsst.ts.mthexapod

.. _lsst.ts.mthexapod.version_history:

###############
Version History
###############

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
