##########
ts_hexapod
##########

Python Commandable SAL Component (CSC) for the camera and M2 hexapods on the Simonyi Survey Telescope.

`Documentation <https://ts-hexapod.lsst.io>`_

The package is compatible with setuptools, as well as Vera Rubin LSST DM's ``eups`` package management system and ``scons`` build system.
Assuming you have the basic DM stack installed you can do the following, from within the package directory:

* ``setup -r .`` to setup the package and dependencies, at which point the unit tests can be run and the package can be used "in place".
* ``pytest`` to run the unit tests.
* ``python setup.py install`` to install the software.
* ``package-docs build`` to build the documentation.
  This requires ``documenteer``; see `building single package docs`_ for installation instructions.

.. _building single package docs: https://developer.lsst.io/stack/building-single-package-docs.html
