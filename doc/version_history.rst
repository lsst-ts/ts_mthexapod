.. py:currentmodule:: lsst.ts.mthexapod

.. _lsst.ts.mthexapod.version_history:

###############
Version History
###############

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
