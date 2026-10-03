# -*- coding: utf-8 -*-
"""WigglePath — the wiggle path families (2026-10-02).

Why: the LIMO never turns tighter than ~0.85-1.2 m whatever is commanded
(SPEC.md, 71-bag sweep), so the planned R 1.0..0.4 cells mostly saturate the
steering. The wiggle families stay ABOVE that ceiling and instead change
curvature often — the advisor's "more wiggly paths". Each is defined directly
by its curvature kappa(s) along arc length with peak |kappa| = 1/R, so the
matrix's radius axis keeps one meaning across every family.

Every kind starts at the origin heading +x (a freshly zeroed odom aligns it to
the robot, like step/slalom), with a straight lead-in L1 and lead-out L_end.

  wiggle_sine    kappa = (1/R) cos(2 pi s / wavelength): a smooth meander,
                 heading swinging +- wavelength / (2 pi R) about +x.
  wiggle_chirp   the same peak curvature, but the wavelength shrinks linearly
                 from `wavelength` to `wavelength_end`: one run sweeps how fast
                 curvature changes (controller bandwidth) at fixed curvature.
  wiggle_square  back-to-back arcs of +-1/R with no straight between them
                 (curvature jumps by 2/R): half arc, full arcs alternating,
                 half arc, so the mean direction stays +x. A full arc is half
                 a wavelength long (angle wavelength / 2R).

Every kind is n_periods wavelengths long, so the length does NOT grow with R:
a curve that fits the venue at R 1.2 still fits at R 2.0 (field 2026-10-02:
the R-scaled square arc reached 12.6 m and could not be seated as an A/B pair
in a 14.6 m venue).

The path is integrated once on a fine arc-length grid (ds = 5 mm) and then
interpolated, the same approach as the vendor's SinusoidalPath.
"""
import math

import numpy as np

try:
    from vfg_pathfollowing.paths.path_base import PathBase
except Exception:  # pragma: no cover - lets geometry tools run without vfg
    PathBase = object

KINDS = ("sine", "chirp", "square")


class WigglePath(PathBase):
    """Curvature-defined wiggle path. See the module docstring for the kinds.

    Parameters
    ----------
    kind : 'sine' | 'chirp' | 'square'
    R : float              tightest radius [m]; peak |curvature| = 1/R
    wavelength : float     (start) wavelength [m]; square: two full arcs
    wavelength_end : float chirp: wavelength at the end of the wiggle [m]
    n_periods : int        number of full left-right swings
    L1, L_end : float      straight lead-in / lead-out [m]
    """

    DS = 0.005

    def __init__(self, kind="sine", R=1.2, wavelength=3.0, wavelength_end=1.5,
                 n_periods=2, L1=0.5, L_end=0.5):
        if kind not in KINDS:
            raise ValueError(f"unknown wiggle kind {kind!r}; expected one of {KINDS}")
        if R <= 0 or n_periods < 1:
            raise ValueError("wiggle needs R > 0 and n_periods >= 1")
        k = 1.0 / float(R)
        ds = self.DS
        if kind == "square":
            # Half arc, (2n-1) full arcs alternating sign, half arc; a full arc
            # is half a wavelength long.
            half = float(wavelength) / 2.0
            if half / float(R) >= math.pi:
                raise ValueError("wiggle wavelength too long for this R "
                                 "(square heading swing would exceed 90 deg)")
            lens = [half / 2.0] + [half] * (2 * n_periods - 1) + [half / 2.0]
            segs = [(L, k * (1 if i % 2 == 0 else -1))
                    for i, L in enumerate(lens)]
            kap_w = np.concatenate([np.full(max(1, int(round(L / ds))), kk)
                                    for L, kk in segs])
        else:
            lam0 = float(wavelength)
            lam1 = float(wavelength_end) if kind == "chirp" else lam0
            if lam0 <= 0 or lam1 <= 0:
                raise ValueError("wiggle wavelength must be > 0")
            # Wiggle length S so the phase runs exactly n_periods cycles:
            # phase(s) = 2 pi * integral ds / lam(s), lam linear lam0 -> lam1.
            if abs(lam1 - lam0) < 1e-9:
                S = n_periods * lam0
            else:
                S = n_periods * (lam1 - lam0) / math.log(lam1 / lam0)
            n_w = max(2, int(round(S / ds)))
            s_w = np.arange(n_w) * ds
            if abs(lam1 - lam0) < 1e-9:
                phase = 2.0 * math.pi * s_w / lam0
            else:
                lam = lam0 + (lam1 - lam0) * s_w / S
                phase = 2.0 * math.pi * S / (lam1 - lam0) * np.log(lam / lam0)
            kap_w = k * np.cos(phase)
            # The sine heading swing must stay below 90 deg or it loops back.
            if max(lam0, lam1) / (2.0 * math.pi * R) >= math.pi / 2:
                raise ValueError("wiggle wavelength too long for this R "
                                 "(heading swing would exceed 90 deg)")
        n1 = max(1, int(round(float(L1) / ds)))
        n2 = max(1, int(round(float(L_end) / ds)))
        kappa = np.concatenate([np.zeros(n1), kap_w, np.zeros(n2)])
        # Grid: sample i covers [i*ds, (i+1)*ds); heading integrates kappa
        # exactly for piecewise-constant curvature (rectangle rule).
        psi = np.concatenate([[0.0], np.cumsum(kappa) * ds])
        x = np.concatenate([[0.0], np.cumsum(np.cos(psi[:-1] + kappa * ds / 2.0)) * ds])
        y = np.concatenate([[0.0], np.cumsum(np.sin(psi[:-1] + kappa * ds / 2.0)) * ds])
        self.kind = kind
        self.R = float(R)
        self._s = np.arange(len(psi)) * ds
        self._x, self._y, self._psi = x, y, psi
        self._kappa = np.concatenate([kappa, kappa[-1:]])
        self._L = float(self._s[-1])

    @property
    def total_length(self):
        return self._L

    def _clip(self, s):
        return np.clip(s, 0.0, self._L)

    def position(self, s):
        s = self._clip(s)
        return np.array([np.interp(s, self._s, self._x),
                         np.interp(s, self._s, self._y)])

    def heading(self, s):
        return float(np.interp(self._clip(s), self._s, self._psi))

    def tangent(self, s):
        h = self.heading(s)
        return np.array([math.cos(h), math.sin(h)])

    def normal(self, s):
        h = self.heading(s)
        return np.array([-math.sin(h), math.cos(h)])

    def curvature(self, s):
        # Piecewise constant on the grid (no blending across a square jump).
        i = int(min(len(self._kappa) - 1,
                    max(0, math.floor(self._clip(s) / self.DS))))
        return float(self._kappa[i])


def wiggle_from_recipe(ptype, params):
    """Build a WigglePath from a recipe of type 'wiggle_<kind>'. The single
    place every consumer (follower, venue_geom, run_eval) maps the recipe, so
    the checked geometry is exactly the driven one."""
    kind = str(ptype).lower().split("_", 1)[1]

    def _f(key, default):
        return float(params.get(key, default))
    return WigglePath(kind=kind, R=_f("R", 1.2),
                      wavelength=_f("wavelength", 3.0),
                      wavelength_end=_f("wavelength_end", 1.5),
                      n_periods=int(params.get("n_periods", 2)),
                      L1=_f("L1", 0.5), L_end=_f("L_end", 0.5))


WIGGLE_TYPES = tuple(f"wiggle_{k}" for k in KINDS)
