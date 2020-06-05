.. py:currentmodule:: lsst.ts.hexapod

.. _lsst.ts.hexapod.version_history:

###############
Version History
###############

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
