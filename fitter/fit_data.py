#!/usr/bin/env python

"""Fit coefficients for a Hexpod compensation model.

Example of use::

    fit_data.py hexapod_motions_camera.txt poly -n 5 6 7

You can change matplotlib's backend by editing the code.

The data format is explained in the README file.
"""
import argparse
import itertools
import pdb
import math

import numpy as np
import matplotlib
import scipy.optimize

# Edit the following to change matplotlib's backend
matplotlib.use("Qt5Agg")

import matplotlib.pyplot as plt  # noqa

RAD_PER_DEG = math.pi / 180.0

# Allowed names for the cause being compensated for
CAUSE_NAMES = ("elevation", "zd", "azimuth", "temperature")

# Names of hexapod axes in the data files.
COLUMN_NAMES = ("cause", "x", "y", "z", "u", "v", "w")

UNITS_DICT = dict(
    elevation="deg",
    azimuth="deg",
    temperature="C",
    x="um",
    y="um",
    z="um",
    u="deg",
    v="deg",
    w="deg",
)

# Names of hexapod axes to fit.
# We only expect compensation for y, z, and u.
# However, if you measure reproducible effects in other axes
# then edit the following.
AXES_TO_FIT = ["x", "y", "z", "u"]


def read_data(path):
    """Read and parse a data file.

    Parameters
    ----------
    path : `str` or `pathlib.Path`
        Path to data file

    Returns
    -------
    data : `numpy.ndarray`
        Parsed data as a list of arrays named:
        cause, x, y, z (um), u, v, w (deg),
        where cause is the literal string "cause".
    """
    data = np.loadtxt(
        fname=path,
        dtype=dict(names=COLUMN_NAMES, formats=(float,) * 7),
        usecols=[0, 1, 2, 3, 4, 5, 6],
    )
    return data


def cospoly(ang, *coeffs):
    """Polynomial with x = cos(ang) and ang in degrees.

    f(ang) = C0 + C1 cos(ang) + C2 cos(ang)^2 + ...
    """
    x = np.cos(np.radians(ang))
    poly = np.polynomial.Polynomial(coeffs)
    return poly(x)


def poly(x, *coeffs):
    """Standard polynomial.

    f(x) = C0 + C1 x + C2 x^2 + ...
    """
    poly = np.polynomial.Polynomial(coeffs)
    return poly(x)


def return_one(data):
    return 1


def fourier(ang, *coeffs):
    """Real-valued Fourier series with ang in degrees.

    f(ang) = C0 + C1 sin(ang) + C2 cos(ang) + C3 sin(2 ang) + C4 cos(2 ang) ...
    """
    function_iter = itertools.cycle([np.sin, np.cos])
    functions = [return_one] + [next(function_iter) for i in range(len(coeffs) - 1)]
    # Angle multipliers for C0, C1, C2, C3
    angle_multipliers = [((i + 1) // 2) * RAD_PER_DEG for i in range(len(coeffs))]

    result = 0
    for coeff, function, angle_multiplier in zip(coeffs, functions, angle_multipliers):
        result += coeff * function(ang * angle_multiplier)
    return result


def fit_one(data, cause, axis, model, ncoeffs):
    """Fit one axis of MTHexapod motion vs cause to a model.

    Parameters
    ----------
    data : `numpy.ndarray`
        Data as a structured array with fields: elevation (deg)
        plus all fields in COLUMN_NAMES
    cause : `str`
        Name of cause of motion, e.g. "elevation".
    axis : `str`
        Name of hexapod axis to fit, e.g. "u".
    model : callable
        Model to fit. It must take the following arguments:

        * x (positional): a numpy.ndarray of values;
          most models require an angle in degrees.
        * *coeffs: a sequence of coefficients
    ncoeffs : `int`
        Number of coefficients.

    Returns
    -------
    coeffs : `list` [`float`]
        The fit coefficients.
    covariance : `numpy.ndarray`
        Estimated covariance of coeffs. The diagonals provide the variance
        of the parameter estimate. See the documentation for
        ``scipy.optimize.curve_fit`` for more information.
    """
    if ncoeffs < 1:
        raise ValueError("Must be at least 1 coeff")
    cause_data = data["cause"]
    effect_data = data[axis]
    coeffs, covariance = scipy.optimize.curve_fit(
        model, cause_data, effect_data, p0=[0] * ncoeffs
    )
    return coeffs, covariance


def fit_all_axes(
    data, cause, model, ncoeffs_arr, print_covariance=False, graph_path=None
):
    """Fit and plot all AXES_TO_FIT using varying numbers of coeffs.

    Parameters
    ----------
    data : `numpy.ndarray`
        Data as a structured array with fields: elevation (deg)
        plus all fields in COLUMN_NAMES
    cause : `str`
        Name of cause being compensated for, e.g. "elevation".
        Note that `read_data` converts zd to elevation,
        so you should do the same for this argument.
    model : callable
        Model to fit. It must take the following arguments:

        * ang: a numpy.ndarray of angles, in deg
        * *coeffs: a sequence of coefficients
    ncoeffs_arr : `list` [`int`]
        Sequence of numbers of coefficients to try.
    print_covariance : `bool`, optional
        Print the covariance matrix from the fit?
    graph_path : `str` or `None`, optional
        Path to which to save the graph.
        If `None` do not save the graph and pause to show it.
    """
    cause_data = data["cause"]
    cause_units = UNITS_DICT[cause]
    dense_cause_data = np.linspace(start=cause_data[0], stop=cause_data[-1])

    nfits = len(AXES_TO_FIT)
    fig, plots = plt.subplots(2, nfits, sharex=True, figsize=(25, 10))
    plot_dict = {axis: (plots[0, i], plots[1, i]) for i, axis in enumerate(AXES_TO_FIT)}

    for axis in AXES_TO_FIT:
        print()
        effect_units = UNITS_DICT[axis]
        effect_data = data[axis]

        data_plot, resid_plot = plot_dict[axis]
        data_plot.set(ylabel=f"{axis} ({effect_units})")
        resid_plot.set(xlabel=f"{cause} ({cause_units})", ylabel=f"{axis} residuals)")
        data_plot.set_title(f"{axis} vs. {cause}")

        for ncoeffs in ncoeffs_arr:
            coeffs, covariance = fit_one(
                data=data, cause=cause, axis=axis, model=model, ncoeffs=ncoeffs
            )
            dense_modeled_data = model(dense_cause_data, *coeffs)
            data_plot.plot(dense_cause_data, dense_modeled_data)
            residuals = model(cause_data, *coeffs) - effect_data
            resid_plot.plot(cause_data, residuals, ".")
            print(
                f"{axis} coeffs: {np.array2string(coeffs, separator=', ', max_line_width=1000)}"
            )
            abs_residuals = np.abs(residuals)
            print(
                f"{axis} residuals: mean(abs) = {abs_residuals.mean():0.3g}; "
                f"max(abs) = {abs_residuals.max():0.3g}; "
                f"std dev = {residuals.std():0.3g}"
            )
            if print_covariance:
                print(f"{axis} coeffs covariance = {covariance}")

        # Plot data last so colors match between data and residual plots.
        data_plot.plot(cause_data, effect_data, ".")

        legend_labels = [f"{n} coeffs" for n in ncoeffs_arr]
        resid_plot.legend(legend_labels)

    fig.show()
    if graph_path:
        fig.savefig(graph_path)
    else:
        pdb.set_trace()


model_dict = dict(poly=poly, fourier=fourier, cospoly=cospoly)
model_names = sorted(model_dict.keys())


def main():
    parser = argparse.ArgumentParser("fit MTHexapod compensation coefficients")
    parser.add_argument(
        "datafile", help="Path to data file.",
    )
    parser.add_argument(
        "cause", help="Cause being compensated for.", choices=CAUSE_NAMES
    )
    parser.add_argument("model", help="Model to fit.", choices=model_names)
    parser.add_argument(
        "-n",
        "--ncoeffs",
        type=int,
        help="Number of coefficients",
        nargs="*",
        default=(3, 5, 7, 9),
    )
    parser.add_argument(
        "--covar",
        help="Print the covariance matrix returned by the fitter?",
        action="store_true",
    )
    parser.add_argument(
        "--graphpath",
        help="Path to which to save the graph. The graph is not saved, if omitted.",
    )
    args = parser.parse_args()

    model = model_dict[args.model]
    if args.cause == "temperature" and args.model != "poly":
        parser.error("Only the poly model can be used to fit temperature")

    data = read_data(path=args.datafile)
    if args.cause == "zd":
        print("Converting zenith distance to elevation")
        data["cause"][:] = 90 - data["cause"]
        args.cause = "elevation"
    fit_all_axes(
        data=data,
        cause=args.cause,
        model=model,
        ncoeffs_arr=args.ncoeffs,
        print_covariance=args.covar,
        graph_path=args.graphpath,
    )


main()
