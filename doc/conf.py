"""Sphinx configuration file for an LSST stack package.

This configuration only affects single-package Sphinx documentation builds.
"""

import lsst.ts.mthexapod  # noqa
from documenteer.conf.pipelinespkg import *  # type: ignore # noqa

project = "ts_mthexapod"
html_theme_options["logotext"] = project  # type: ignore # noqa
html_title = project
html_short_title = project
doxylink = {}  # type: ignore # noqa

intersphinx_mapping["ts_xml"] = ("https://ts-xml.lsst.io", None)  # type: ignore # noqa
intersphinx_mapping["ts_salobj"] = ("https://ts-salobj.lsst.io", None)  # type: ignore # noqa
intersphinx_mapping["ts_hexrotcomm"] = ("https://ts-hexrotcomm.lsst.io", None)  # type: ignore # noqa
