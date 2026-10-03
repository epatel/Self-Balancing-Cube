#!/usr/bin/env python3
"""Measure the MPU6050's accelerometer zero offsets from three face readings.

Lay the cube still on three faces that meet at a corner (the three wheel
faces work well) and note the dashboard's raw X Y Z line for each.  Lying
still, each reading is gravity plus the same constant offset; gravity on
three perpendicular faces is three equal, mutually perpendicular vectors.
Fitting that model gives the offset, which is converted back to the chip's
own axes and added to the ACC_OFFSET_* values already in ESP32.h.

    python tools/accel_offsets.py  X,Y,Z  X,Y,Z  X,Y,Z

Needs numpy.  Reads IMU_MOUNT and ACC_OFFSET_* from esp32_cube_enc/ESP32.h.
"""
import pathlib
import re
import sys

import numpy as np

HDR = pathlib.Path(__file__).resolve().parent.parent / "esp32_cube_enc" / "ESP32.h"


def define(name, text):
    m = re.search(r"#define\s+%s\s+(-?\d+)" % name, text)
    if not m:
        raise SystemExit("%s not found in %s" % (name, HDR))
    return int(m.group(1))


def to_chip(v, mount):
    """Dashboard frame -> chip axes (inverse of the IMU_MOUNT rotation)."""
    if mount == 1:                     # firmware X = -chip Z, Y = Y, Z = chip X
        return np.array([v[2], v[1], -v[0]])
    return np.array(v, dtype=float)


def fit(readings):
    """Least-squares offset o and gravity G: |r_i - o| = G, (r_i - o) perpendicular."""
    def residuals(p):
        o, G = p[:3], p[3]
        g = [r - o for r in readings]
        return np.array([np.linalg.norm(x) - G for x in g] +
                        [g[0] @ g[1] / G, g[0] @ g[2] / G, g[1] @ g[2] / G])
    p = np.array([0.0, 0.0, 0.0, 16384.0])
    for _ in range(50):                # Gauss-Newton with a numeric Jacobian
        r = residuals(p)
        J = np.array([(residuals(p + d) - r) / 1e-3 for d in np.eye(4) * 1e-3]).T
        step = np.linalg.lstsq(J, -r, rcond=None)[0]
        p += step
        if np.abs(step).max() < 1e-3:
            break
    return p[:3], p[3], residuals(p)


def angles(v):
    a = lambda u, w: np.degrees(np.arccos(u @ w / np.linalg.norm(u) / np.linalg.norm(w)))
    return "%.1f / %.1f / %.1f" % (a(v[0], v[1]), a(v[0], v[2]), a(v[1], v[2]))


def main(args):
    if len(args) != 3:
        raise SystemExit(__doc__)
    readings = [np.array([float(x) for x in a.split(",")]) for a in args]
    text = HDR.read_text(encoding="utf-8")
    mount = define("IMU_MOUNT", text)
    current = np.array([define("ACC_OFFSET_" + k, text) for k in "XYZ"])

    o, G, res = fit(readings)
    g = [r - o for r in readings]
    print("as read:   |g| %s   angles %s" %
          (" / ".join("%.0f" % np.linalg.norm(r) for r in readings), angles(readings)))
    print("corrected: |g| %s   angles %s   (fit residual max %.0f)" %
          (" / ".join("%.0f" % np.linalg.norm(x) for x in g), angles(g), np.abs(res).max()))
    print("1 g = %.0f counts" % G)
    up = sum(g) / np.linalg.norm(sum(g))
    print("board tilt from the corner's diagonal: %.1f deg "
          "(meaningful if the three faces meet at the balancing corner)"
          % np.degrees(np.arccos(abs(up[2]))))
    new = current + to_chip(o, mount)
    print("\nresidual offset, dashboard frame: %s" % np.round(o).astype(int))
    print("new values for ESP32.h (chip axes):")
    for k, v in zip("XYZ", new):
        print("  #define ACC_OFFSET_%s  %d" % (k, round(v)))


if __name__ == "__main__":
    main(sys.argv[1:])
