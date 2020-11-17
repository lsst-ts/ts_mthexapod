.. py:currentmodule:: lsst.ts.mthexapod

.. _lsst.ts.mthexapod.developer_guide:

###############
Developer Guide
###############

The MTHexapod CSC is implemented using `ts_salobj <https://github.com/lsst-ts/ts_salobj>`_ and `ts_hexrotcom <https://ts-hexrotcomm.lsst.io>`_.

.. _lsst.ts.mthexapod-api:

API
===

The primary class is:

* `HexapodCsc`: the CSC.

.. automodapi:: lsst.ts.mthexapod
   :no-main-docstr:

Build and Test
==============

This is a pure python package. There is nothing to build except the documentation.

.. code-block:: bash

    make_idl_files.py MTHexapod
    setup -r .
    pytest -v  # to run tests
    package-docs clean; package-docs build  # to build the documentation

Contributing
============

``ts_mthexapod`` is developed at https://github.com/lsst-ts/ts_mthexapod.
You can find Jira issues for this package using `labels=ts_mthexapod <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20labels%20%20%3D%20ts_mthexapod>`_..
