.. py:currentmodule:: lsst.ts.mthexapod

.. _lsst.ts.mthexapod.version_history:

###############
Version History
###############

v0.14.0
=======

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
=======

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
=======

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
=======

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
=======

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
=======

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
=======

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
======

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
======

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
======

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
======

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
======

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
======

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
======

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
======

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
======

Use `lsst.ts.simactuators.PointToPointActuator` instead of an internal copy.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
* ts_simactuators

v0.4.0
======

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
======

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
======

Update for changes to the XML.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``

v0.2.2
======

The first version we tested against the real hexapod controller!

Requires:
* ts_hexrotcomm v0.1.0
* ts_salobj 5
* ts_idl 1
* Hexapod IDL files, e.g. made using ``make_idl_files.py Hexapod``
