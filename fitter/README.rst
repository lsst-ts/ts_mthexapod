Code to fit coefficients for compensated moves.

Contents
--------

* ``fit_data.py``: a command-line script that fits a data file
* ``zd_camera.txt`` zenith distance data for the Camera hexapod, from finite element analysis
* ``zd_m2.txt`` zenith distance data for the M2 hexapod, from finite element analysis

Data Format
-----------

The data format is as follows. See the data files for examples:

* Data is text, with space separated columns.
* Comment lines must begin with #
* There must be exactly 7 columns of data in the following order:
  
  * cause: cause of motion; one of zenith distance, elevation, azimuth (deg), or temperature (C).
  * x offset: (um)
  * y offset: (um)
  * z offset: (um)
  * u (rot about x) offset: (deg)
  * v (rot about y) offset: (deg)
  * w (rot about z) offset: (deg)
* The sign of the offsets is: compensated value = offset + uncompensated (user-specified) value.

The files were obtained by selecting and copying the appropriate region of these spreadsheets:
https://github.com/bxin/hexrot/blob/master/LUT/Camera%20Hexapod%20Motions%20in%20Elevation%20Axis%202020%2007%2023.xlsx
https://github.com/bxin/hexrot/blob/master/LUT/M2%20Hexapod%20Motions%20in%20Elevation%20Axis%202020%2007%2023.xlsx

Notes
-----

``fit_data.py`` is completely self-contained because I wanted to be able to run it on macOS,
without worrying about OpenSplice.
