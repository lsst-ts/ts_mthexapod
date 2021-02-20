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

Start an MTHexapod CSC as follows:

.. prompt:: bash

    run_mthexapod.py <index>

where ``<index>`` is 1 for the camera hexapod, 2 for the M2 hexapod

Then check that the CSC has control of the low-level controller, as follows:

* Wait for the ``connected`` event to report ``command=True`` and ``telemetry=True``.
  This should happen quickly; if it does not then check that the low-level controller is fully booted up and configured to use the correct IP address for the CSC.
* Check the ``controllerState`` event.
  If it is ``state=Offline, offline_substate=PublishOnly``, which is the state the low-level controller wakes up in,
  then you must :ref:`use the EUI <lsst.ts.mthexapod.enable_with_eui>` to change the state.
* Check the ``commandableByDDS`` event.
  If ``state=False`` then you must :ref:`use the EUI <lsst.ts.mthexapod.enable_with_eui>` to change the control mode.

To use a hexapod for observing:

* Check that the CSC has control of the low-level controller, as just described.
* Enable the CSC.
* Enable :ref:`compensation mode<lsst.ts.mthexapod.compensation_mode>` with the ``setCompensationMode`` command.
* Move to x=0, y=0, z=0, u=0, v=0, w=0 with the ``move`` command.
* Apply offsets, as required, to improve collimation with the ``offset`` command (or specify absolute position with the ``move`` command).

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

Configuration is specified in `ts_config_mttcs <https://github.com/lsst-ts/ts_config_mttcs>`_ following `this schema <https://github.com/lsst-ts/ts_mthexapod/blob/develop/schema/MTHexapod.yaml>`_.
The most important settings are:

* Coefficients for the :ref:`compensation<lsst.ts.mthexapod.compensation_mode>` model.

Notes
-----

* To recover from the ``FAULT`` state (after fixing whatever is wrong) issue the ``clearError`` command.
  This will transition to the ``STANDBY`` state.

* The low-level controller maintains the CSC summary state,
  so the CSC reports a summary state of ``OFFLINE`` until it receives telemetry from the low-level controller.
  Thus the CSC may transition from ``OFFLINE`` to almost any other state as it starts up.

* Communication between the low-level controller and CSC is quite unusual:

  * The low-level controller connects to a TCP/IP *server* in the CSC.
    Thus the low-level controller must be configured with the TCP/IP address of the CSC.
  * The low-level controller does not acknowledge commands in any way.
    Thus the CSC must try to predict whether the low-level controller can execute a command and reject the command if not.
    Unfortunately this prediction cannot be completely accurate.
  * The connection uses two separate sockets, one for commands and the other for telemetry and configuration.
    Both are one-directional: the low-level controller reads commands on the command socket and writes configuration and telemetry to the telemetry socket.

Simulator
---------

The CSC includes a simulation mode. To run using CSC's internal simulator:

.. prompt:: bash

    run_mthexapod.py <index> --simulate

.. _lsst.ts.mthexapod.enable_with_eui:

Enable With the EUI
-------------------

Use the Engineering User Interface (EUI) as follows to enable CSC control of the low-level controller:

    * State must be ``state=Offline, offline_substate=Available`` or any more enabled state.
      The low-level controller wakes up in ``state=Offline, offline_substate=PublishOnly``,
      and you must change this before the CSC can control the low-level controller.
      Change the state on the main panel of the EUI.
    * Control mode must be ``DDS``.
      The low-level controller wakes up in control mode ``GUI``,
      and you must change this before the CSC can control the low-level controller.
      To change the control mode use the ``Parameters`` panel;
      note that the EUI *shows* the control mode on the main panel, but that display is read-only.

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
