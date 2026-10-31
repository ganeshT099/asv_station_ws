#!/usr/bin/env python3
"""Fit hull drag and thruster curves from real (or simulated) test data.

Replace the simulation's guessed coefficients with measured ones:

1. Free-decay test.  Push the boat along one axis, release it, and log
   ``time,v`` (body velocity on that axis, m/s) from your localization.
   Fit linear and quadratic drag per unit mass::

       python3 identify_hydrodynamics.py decay decay_surge.csv --mass 43

   Copy the printed values into ``<xU>/<xUabsU>`` (or the sway/yaw
   equivalents) in ``research_catamaran.urdf``.

2. Bollard pull.  Clamp the boat to a load cell, step each thruster through
   commands and log ``command,thrust_N`` rows.  Fit the curve, deadband and
   reverse/forward asymmetry::

       python3 identify_hydrodynamics.py thrust bollard.csv

   Use the results for ``thruster_deadband`` and ``thruster_reverse_scale``.
"""

import argparse
import csv
import sys

import numpy as np


def read_columns(path, *names):
    with open(path, newline='') as handle:
        rows = list(csv.DictReader(handle))
    return [np.array([float(r[n]) for r in rows]) for n in names]


def fit_decay(t, v, mass, smooth=7):
    """Regress dv/dt = -(b1/m) v - (b2/m)|v|v on a free-decay record."""
    kernel = np.ones(smooth) / smooth
    vs = np.convolve(v, kernel, mode='same')
    dv = np.gradient(vs, t)
    keep = slice(smooth, -smooth)
    x = np.column_stack([vs[keep], np.abs(vs[keep]) * vs[keep]])
    coeff, *_ = np.linalg.lstsq(x, -dv[keep], rcond=None)
    b1, b2 = coeff * mass
    residual = -dv[keep] - x @ coeff
    r2 = 1.0 - residual.var() / np.var(dv[keep])
    return b1, b2, r2


def fit_thrust(command, thrust):
    """Return deadband, forward/reverse gain and a quality figure."""
    command, thrust = np.asarray(command), np.asarray(thrust)
    live = np.abs(thrust) > 0.05 * np.max(np.abs(thrust))
    deadband = float(np.min(np.abs(command[live]))) if live.any() else 0.0
    gains = {}
    for name, mask in (('forward', command > 0), ('reverse', command < 0)):
        sel = mask & live
        if sel.sum() >= 2:
            gains[name] = float(
                np.sum(command[sel] * thrust[sel]) / np.sum(command[sel] ** 2))
    scale = abs(gains['reverse'] / gains['forward']) \
        if 'forward' in gains and 'reverse' in gains else float('nan')
    return deadband, gains, scale


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    sub = parser.add_subparsers(dest='mode', required=True)
    decay = sub.add_parser('decay')
    decay.add_argument('csv')
    decay.add_argument('--mass', type=float, required=True,
                       help='vehicle mass incl. payload (kg)')
    thrust = sub.add_parser('thrust')
    thrust.add_argument('csv')
    args = parser.parse_args()
    if args.mode == 'decay':
        t, v = read_columns(args.csv, 'time', 'v')
        b1, b2, r2 = fit_decay(t, v, args.mass)
        print(f'linear drag    : {b1:.3f} N/(m/s)   -> <xU> = {-b1:.3f}')
        print(f'quadratic drag : {b2:.3f} N/(m/s)^2 -> <xUabsU> = {-b2:.3f}')
        print(f'fit R^2        : {r2:.3f}')
    else:
        c, f = read_columns(args.csv, 'command', 'thrust_N')
        deadband, gains, scale = fit_thrust(c, f)
        print(f'deadband (command units) : {deadband:.3f}')
        print(f'gain forward / reverse   : {gains}')
        print(f'thruster_reverse_scale   : {scale:.2f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
