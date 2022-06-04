.. py:currentmodule:: lsst.ts.mthexapod

.. _lsst.ts.mthexapod:

#################
lsst.ts.mthexapod
#################

.. image:: https://img.shields.io/badge/Project Metadata-gray.svg
    :target: https://ts-xml.lsst.io/index.html#index-master-csc-table-mthexapod
.. image:: https://img.shields.io/badge/SAL\ Interface-gray.svg
    :target: https://ts-xml.lsst.io/sal_interfaces/MTHexapod.html
.. image:: https://img.shields.io/badge/GitHub-gray.svg
    :target: https://github.com/lsst-ts/ts_mthexapod
.. image:: https://img.shields.io/badge/Jira-gray.svg
    :target: https://jira.lsstcorp.org/issues/?jql=labels+%3D+ts_mthexapod

Overview
========

The MTHexapod CSC controls the camera and M2 hexapods on the Simonyi Survey Telescope.
Run one instance of the CSC for each hexapod, using index=1 for the camera hexapod and index=2 for the M2 hexapod.

User Guide
==========

Start an MTHexapod CSC
----------------------

.. prompt:: bash

    run_mthexapod <index>

where ``<index>`` is 1 for the camera hexapod, 2 for the M2 hexapod

Use a Hexapod for Observing
---------------------------

* Enable the CSC.
  If this fails see :ref:`troubleshooting <lsst.ts.mthexapod.troubleshooting>`.
* Enable :ref:`compensation mode<lsst.ts.mthexapod.compensation_mode>` with the ``setCompensationMode`` command.
* Use the ``move`` and/or ``offset`` commands to improve and maintain collimation.

.. _lsst.ts.mthexapod.troubleshooting:

Troubleshooting
---------------

The hexapod will refuse to go into ENABLED state if:

* The low-level controller is in state=Offline, offline_substate=PublishOnly,
  which is the state in which the low-level controller wakes up.
  To fix this :ref:`use the EUI to enable DDS mode <lsst.ts.mthexapod.enable_with_eui>`.
* The EUI has control.
  To fix this :ref:`use the EUI to enable DDS mode <lsst.ts.mthexapod.enable_with_eui>`.

To recover from a low-level controller fault:

* Figure out why the controller faulted and fix the problem.
* Send the CSC to STANDBY state, then to ENABLED state.

.. _lsst.ts.mthexapod.compensation_mode:

Compensation Mode
-----------------

The CSC is capable of applying compensation (correction) for mechanical changes induced by changes in gravity and temperature, in order to preserve collimation.
Each effect is modeled independently; the corrections are added together to get the total compensation offset.
The inputs to the model are telescope target elevation and azimuth, camera rotator target angle, and temperature.
The CSC uses target positions in order to give the hexapods more time to get in position during a slew.
The coefficients for the compensation model are specified in the CSC :ref:`configuration <lsst.ts.mthexapod.configuration>`.

The CSC starts with compensation disabled.
To enable compensation: issue the ``setCompensationMode`` command with ``enable=True``.
Compensation is disabled when:

* A user commands ``setCompensationMode`` with ``enable=False``.
  This will remove the current compensation offset, which will move the hexapod by a small amount.
* The CSC leaves the enabled state.
  This will not remove the current compensation offset.
* An error occurs in the compensation loop.
  This will not remove the current compensation offset.

The ``move`` command specifies the uncompensated (aka nominal) position.
Thus the specified position should produce (approximately) the same effect on collimation, regardless of the current telescope elevation, temperature, etc.
If you wish to move the hexapod to a give mechanical position (with reproducible hexapod strut lengths), disable compensation before you command the move.

The compensated (aka corrected) position is the position sent to the low-level controller.
If compensation is enabled but the CSC does not yet have all the inputs it needs for the compensation model (e.g. telescope target position or rotator target position), it will issue one warning ``logMessage`` event and keep trying.
When all compensation inputs are available, compensation corrections will begin.

The compensation will continue to be updated in the background for changes in elevation, temperature, etc.
In order to reduce heat generation in the hexapod, the configuration parameter ``min_compensation_adjustment`` specifies the smallest compensation offset the background compensation task will command.
Compensation is only updated if ``abs(new_compensation_offset - current_compensation_offset) >= min_compensation_adjustment`` in any axis.
Note that ``min_compensation_adjustment`` does not affect the ``move``, ``offset``, and ``setCompensationMode`` commands;
these commands always apply compensation if compensation mode is enabled.
Thus you can force a compensation update by issuing an ``offset`` command with x, y, z, u, v, and w all zero.

Relevant events:

* ``uncompensatedPosition``: the position commanded by the user.
  This event is output once in response to a ``move`` or ``offset`` command.
* ``compensatedPosition``: the compensated (corrected) position; the position sent to the low-level controller.
  This event is output once in response to a ``move`` or ``offset`` command and (if compensation is enabled) once each time the compensation loop applies a correction.
  This will match the uncompensated position if compensation is disabled or cannot be computed.
  In order to compute compensation a ``move`` must have been commanded since the MTHexapod CSC was started,
  and all inputs to the compensation model must have been published by the appropriate CSCs.
  Thus failure to compute a compensation correction is only likely when MTHexapod, or any of the CSCs that generate compensation inputs, have first been brought up.
* ``compensation``: inputs to the compensation model and the resulting compensation offset.
  Output when compensation is applied.

.. _lsst.ts.mthexapod.configuration:

Configuration
-------------

Configuration is specified in `ts_config_mttcs <https://github.com/lsst-ts/ts_config_mttcs>`_ following `this schema <https://github.com/lsst-ts/ts_mthexapod/blob/develop/python/lsst/ts/mthexapod/config_schema.py>`_.
The most important settings are:

* Coefficients for the :ref:`compensation<lsst.ts.mthexapod.compensation_mode>` model.

Simulator
---------

The CSC includes a simulation mode. To run using CSC's internal simulator:

.. prompt:: bash

    run_mthexapod <index> --simulate

.. _lsst.ts.mthexapod.enable_with_eui:

Enable With the EUI
-------------------

The control mode must be ``DDS`` in order for the CSC to control the low-level controller.
If the control mode is ``GUI`` then you can use the EUI (aka GUI) to change it to ``DDS`` as follows:

* In the main panel: change the state to ``state=Offline, offline_substate=Available``.
* Go to the ``Parameters`` panel to change the control mode to ``DDS``.

Notes:

* The EUI *shows* the control mode on the main panel, but that display is read-only.
  You must use the ``Parameters`` panel to change the control mode.
* If you issue any hexapod command in the EUI, control mode will switch back to ``GUI``.
  So if you want the CSC to retain control, please be careful what you touch when using the GUI.

Developer Guide
===============

.. toctree::
    developer_guide
    :maxdepth: 1

Version History
===============

.. toctree::
    version_history
    :maxdepth: 1
