"""Analytic gas model for the confined-space (tank interior) Spot demo.

Pure stdlib + numpy — NO omni/isaacsim/rclpy imports — so it unit-tests on a
Mac (``python3 -m unittest tests.test_gas_field`` from the repo root) and is
also importable unchanged inside the Isaac Sim 6.1 container via
``/isaac-sim/python.sh``.

Model
-----
The atmosphere is a clean baseline (O2 20.9 %, LEL 0 %, H2S 0 ppm, CO 0 ppm)
everywhere. Inside the circular tank footprint (default center (0, 0),
radius 6 m) a deterministic, seed-derived layout of Gaussian "pockets" is
superimposed:

* one H2S pocket pooled LOW — concentration decays exponentially with height
  above the floor (H2S is heavier than air),
* one hydrocarbon (LEL) pocket,
* mild O2 depletion co-located with the LEL pocket (vapor displaces air).

Built-in pockets contribute nothing outside the tank footprint, so readings
outside the tank are exactly baseline (plus optional sensor noise). Pockets
added at runtime with :meth:`GasField.inject_spike` are isotropic 3-D
Gaussians and apply wherever they are placed (inside or outside the tank).

Opt-in modes (defaults reproduce the model above exactly)
--------------------------------------------------------
* ``headspace=``: a horizontally well-mixed, floor-referenced layer over the
  whole footprint, i.e. the atmosphere of a tank that has not been
  ventilated at all (the discrete pockets are its local peaks). The facade
  uses :data:`UNVENTILATED_HEADSPACE`: H2S stratified low (about 51 ppm at
  0.2 m, 13.5 ppm at 0.6 m, 3.6 ppm at 1.0 m), LEL about 7.5 %, O2 about
  19.9 %.
  Illustrative demo values, not derived from any real tank.
* Forced ventilation, time-aware: :meth:`GasField.set_ventilation` makes every
  BUILT-IN contribution (pockets + headspace) decay as ``exp(-(t - t_on) /
  tau)`` toward ``residual_fraction`` (default 0.04, below every alarm) once
  ventilation is on. Pass the sim time to :meth:`GasField.sample` /
  :meth:`GasField.probe` as ``t``; ``t=None`` evaluates the field as of the
  last ventilation change (no decay applied). Turning ventilation off holds
  the current fraction (no re-accumulation: the product is isolated);
  :meth:`GasField.reset_ventilation` restores the unventilated state.
  Injected spikes model an active release and do NOT decay; remove them with
  :meth:`GasField.clear_injected`.

Alarm thresholds follow OSHA 1910.146 Appendix E:
O2 < 19.5 % or > 23.5 %, LEL >= 10 %, H2S >= 10 ppm, CO >= 35 ppm.

Units: distances in meters, z = height above tank floor. O2 and LEL are
percentages; H2S and CO are ppm.

:class:`ProbeGeometry` holds the facade's gas-probe rules (an extendable gas
probe, simulated; arm-mounted on the real robot): body pose required,
1.1 m horizontal reach from the body's front, and line of sight into the
tank only through the doorway opening ("occluded by tank wall" otherwise).
"""

import math

import numpy as np

# Baseline clean atmosphere.
BASELINE = {
    "o2_pct": 20.9,
    "lel_pct": 0.0,
    "h2s_ppm": 0.0,
    "co_ppm": 0.0,
}

# OSHA 1910.146 Appendix E thresholds.
O2_LOW_PCT = 19.5
O2_HIGH_PCT = 23.5
LEL_ALARM_PCT = 10.0
H2S_ALARM_PPM = 10.0
CO_ALARM_PPM = 35.0

# 1-sigma gaussian sensor noise per gas (kept small relative to thresholds).
NOISE_SIGMA = {
    "o2_pct": 0.05,
    "lel_pct": 0.1,
    "h2s_ppm": 0.2,
    "co_ppm": 0.3,
}

# Map the short gas names accepted by inject_spike to sample-dict keys.
GAS_KEYS = {
    "o2": "o2_pct",
    "lel": "lel_pct",
    "h2s": "h2s_ppm",
    "co": "co_ppm",
}

# Forced-ventilation model defaults (demo-tunable per call).
DEFAULT_VENT_TAU_S = 15.0
DEFAULT_RESIDUAL_FRACTION = 0.04

# At most this many injected (runtime) pockets at once: a flood of
# payload/gas/inject cannot grow the pocket list, and with it the cost of
# every sample, without bound (threat model T23). clear_injected() frees them.
MAX_INJECTED_POCKETS = 16

# Unventilated headspace used by the demo facade (sim/isaac_app.py), as
# (sample key, floor amplitude, z_scale m): value(z) = amplitude *
# exp(-z / z_scale) everywhere inside the footprint. H2S (1.19x air density)
# stratifies low, so a probe at 1.0 m alone (3.6 ppm) would pass a tank
# that reads 51 ppm at 0.2 m. Sized so that a door-mouth probe only passes
# once the interior route is also below every alarm (tests assert this).
UNVENTILATED_HEADSPACE = (
    ("h2s_ppm", 100.0, 0.3),
    ("lel_pct", 8.0, 3.0),
    ("o2_pct", -1.0, 3.0),
)


# Gas-probe reach / occlusion rules (the facade's payload/gas/probe; see
# ProbeGeometry). The demo robot has no arm model: the facade stands in an
# extendable probe. The real Spot Arm reaches ~0.98 m, so points more than
# PROBE_MAX_REACH_M (horizontal) from the body's front are refused.
PROBE_LABEL = "extendable gas probe (simulated; arm-mounted on the real robot)"
PROBE_MAX_REACH_M = 1.1
BODY_FRONT_OFFSET_M = 0.55      # Spot body box 1.10 x 0.50 m: nose 0.55 m ahead of the center
PROBE_Z_RANGE_M = (0.0, 3.0)    # height above the tank floor
DOOR_WIDTH_M = 1.30             # sim/build_scene.py DOOR_WIDTH (clear width between jambs)
DOOR_HEIGHT_M = 1.30            # sim/build_scene.py DOOR_HEIGHT (clear height, floor to lintel)
DOOR_FRAME_HALF_DEPTH_M = 0.125  # build_scene jambs are 0.25 m deep (radially)


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("%s must be finite, got %r" % (name, value))
    return value


class GasField:
    """Deterministic analytic gas field for the tank interior.

    Parameters
    ----------
    seed : int
        Seed for the pocket layout and for the sensor-noise stream.
        Two instances built with the same seed produce identical layouts
        and identical noisy sample sequences.
    center : tuple of float
        (x, y) of the tank footprint center. Default (0.0, 0.0).
    radius : float
        Tank footprint radius in meters. Default 6.0.
    headspace : iterable of (key, amplitude, z_scale) or None
        Optional unventilated headspace layer (see module docstring and
        :data:`UNVENTILATED_HEADSPACE`). Default None = pockets only.
    residual_fraction : float
        Fraction of the built-in contributions left after long forced
        ventilation. Default 0.04.
    max_injected : int
        Most injected pockets held at once (:meth:`inject_spike` refuses
        more). Default :data:`MAX_INJECTED_POCKETS`.
    """

    def __init__(self, seed=7, center=(0.0, 0.0), radius=6.0, headspace=None,
                 residual_fraction=DEFAULT_RESIDUAL_FRACTION,
                 max_injected=MAX_INJECTED_POCKETS):
        if radius <= 0.0:
            raise ValueError("radius must be > 0, got %r" % (radius,))
        self.max_injected = int(max_injected)
        if self.max_injected < 0:
            raise ValueError("max_injected must be >= 0, got %r" % (max_injected,))
        residual_fraction = _finite(residual_fraction, "residual_fraction")
        if not 0.0 <= residual_fraction < 1.0:
            raise ValueError("residual_fraction must be in [0, 1), got %r"
                             % (residual_fraction,))
        self.residual_fraction = residual_fraction
        self.seed = int(seed)
        self.center = (float(center[0]), float(center[1]))
        self.radius = float(radius)

        # Separate rng streams: one consumed once for the layout, one that
        # advances per sample() call for sensor noise.
        layout_rng = np.random.default_rng([self.seed, 0])
        self._noise_rng = np.random.default_rng([self.seed, 1])

        # Each pocket: dict(key, center=(x,y), amplitude, sigma_xy,
        #                   z_scale or None, z_center or None, masked_to_tank).
        # z_scale: exponential decay with height (floor-pooled, heavy gas).
        # z_center + sigma_z: isotropic vertical gaussian (injected spikes).
        self._pockets = []

        cx, cy = self.center
        r = self.radius

        def _pos():
            # Keep built-in pockets well inside the footprint so their tails
            # are negligible at the wall even before masking.
            ang = layout_rng.uniform(0.0, 2.0 * math.pi)
            dist = layout_rng.uniform(0.15, 0.55) * r
            return (cx + dist * math.cos(ang), cy + dist * math.sin(ang))

        # 1) H2S pocket, pooled low (heavier than air).
        h2s_pos = _pos()
        self._pockets.append({
            "key": "h2s_ppm",
            "center": h2s_pos,
            "amplitude": float(layout_rng.uniform(30.0, 60.0)),
            "sigma_xy": float(layout_rng.uniform(0.8, 1.4)),
            "z_scale": float(layout_rng.uniform(0.35, 0.6)),
            "z_center": None,
            "sigma_z": None,
            "masked_to_tank": True,
            "builtin": True,
        })

        # 2) Hydrocarbon / LEL pocket (mild vertical falloff).
        lel_pos = _pos()
        lel_sigma = float(layout_rng.uniform(1.0, 1.6))
        self._pockets.append({
            "key": "lel_pct",
            "center": lel_pos,
            "amplitude": float(layout_rng.uniform(20.0, 40.0)),
            "sigma_xy": lel_sigma,
            "z_scale": float(layout_rng.uniform(1.5, 2.5)),
            "z_center": None,
            "sigma_z": None,
            "masked_to_tank": True,
            "builtin": True,
        })

        # 3) Mild O2 depletion co-located with the LEL pocket
        #    (negative amplitude: subtracts from o2_pct).
        self._pockets.append({
            "key": "o2_pct",
            "center": lel_pos,
            "amplitude": -float(layout_rng.uniform(1.5, 3.0)),
            "sigma_xy": lel_sigma,
            "z_scale": float(layout_rng.uniform(1.5, 2.5)),
            "z_center": None,
            "sigma_z": None,
            "masked_to_tank": True,
            "builtin": True,
        })

        # 4) Optional unventilated headspace: horizontally uniform
        #    (sigma_xy None) floor-referenced layers. Appended after the
        #    seeded pockets and drawn from no rng, so the seed layout and
        #    pocket_center() are unchanged.
        for key, amplitude, z_scale in (headspace or ()):
            if key not in BASELINE:
                raise ValueError("headspace key must be one of %s, got %r"
                                 % (sorted(BASELINE), key))
            z_scale = _finite(z_scale, "headspace z_scale")
            if z_scale <= 0.0:
                raise ValueError("headspace z_scale must be > 0, got %r" % (z_scale,))
            self._pockets.append({
                "key": key,
                "center": self.center,
                "amplitude": _finite(amplitude, "headspace amplitude"),
                "sigma_xy": None,
                "z_scale": z_scale,
                "z_center": None,
                "sigma_z": None,
                "masked_to_tank": True,
                "builtin": True,
            })

        # Ventilation state. Built-in contributions are scaled by
        # fraction_remaining(t): _frac_anchor while off, else decaying from
        # _frac_anchor (the fraction when ventilation last turned on).
        self._vent_on = False
        self._vent_t_on = None
        self._vent_tau = DEFAULT_VENT_TAU_S
        self._frac_anchor = 1.0

    # ------------------------------------------------------------------ #

    @property
    def pockets(self):
        """Read-only view of the pocket layout (list of dict copies)."""
        return [dict(p) for p in self._pockets]

    def pocket_center(self, gas):
        """(x, y) center of the first built-in/injected pocket for ``gas``."""
        key = self._gas_key(gas)
        for p in self._pockets:
            if p["key"] == key:
                return p["center"]
        return None

    @staticmethod
    def _gas_key(gas):
        try:
            return GAS_KEYS[str(gas).lower()]
        except KeyError:
            raise ValueError(
                "gas must be one of %s, got %r"
                % (sorted(GAS_KEYS), gas)) from None

    def _inside_tank(self, x, y):
        dx = x - self.center[0]
        dy = y - self.center[1]
        return (dx * dx + dy * dy) <= self.radius * self.radius

    def _pocket_contribution(self, pocket, x, y, z):
        if pocket["masked_to_tank"] and not self._inside_tank(x, y):
            return 0.0
        dx = x - pocket["center"][0]
        dy = y - pocket["center"][1]
        s = pocket["sigma_xy"]
        if s is None:
            horiz = 1.0  # headspace layer: uniform over the footprint
        else:
            horiz = math.exp(-(dx * dx + dy * dy) / (2.0 * s * s))
        if pocket["z_scale"] is not None:
            # Floor-referenced exponential decay with height.
            vert = math.exp(-max(z, 0.0) / pocket["z_scale"])
        else:
            dz = z - pocket["z_center"]
            sz = pocket["sigma_z"]
            vert = math.exp(-(dz * dz) / (2.0 * sz * sz))
        return pocket["amplitude"] * horiz * vert

    # ------------------------------------------------------------------ #

    def sample(self, x, y, z, t=None, noise=True):
        """Gas readings at world point (x, y, z) at sim time ``t`` (s).

        Returns a dict of plain floats:
        ``{"o2_pct", "lel_pct", "h2s_ppm", "co_ppm"}``. ``t`` only matters
        once ventilation has been switched on (see :meth:`set_ventilation`);
        ``t=None`` evaluates as of the last ventilation change.
        Set ``noise=False`` for the exact analytic value (used by tests and
        by :meth:`worst_case_along`). Noise draws come from a dedicated
        seeded rng stream, so a fixed seed plus a fixed call sequence is
        fully deterministic.
        """
        if isinstance(t, bool):
            # sample(x, y, z, False) used to mean noise=False.
            raise TypeError("t is a sim time in seconds; pass noise= by keyword")
        x, y, z = float(x), float(y), float(z)
        frac = self.fraction_remaining(t)
        reading = dict(BASELINE)
        for pocket in self._pockets:
            c = self._pocket_contribution(pocket, x, y, z)
            if pocket["builtin"] and frac != 1.0:
                c *= frac
            reading[pocket["key"]] += c
        if noise:
            for key, sigma in NOISE_SIGMA.items():
                reading[key] += float(self._noise_rng.normal(0.0, sigma))
        # Clamp to physical ranges.
        reading["o2_pct"] = float(min(max(reading["o2_pct"], 0.0), 100.0))
        reading["lel_pct"] = float(min(max(reading["lel_pct"], 0.0), 100.0))
        reading["h2s_ppm"] = float(max(reading["h2s_ppm"], 0.0))
        reading["co_ppm"] = float(max(reading["co_ppm"], 0.0))
        return reading

    def inject_spike(self, gas, ppm_or_pct, x, y, z, radius):
        """Add a runtime pocket: an isotropic 3-D Gaussian centered at
        (x, y, z) with 1-sigma extent ``radius`` and peak amplitude
        ``ppm_or_pct`` (ppm for h2s/co, percentage points for o2/lel).

        For ``gas="o2"`` the amplitude is a signed delta on o2_pct: pass a
        negative value to model displacement/depletion, positive for
        enrichment. Injected spikes are NOT masked to the tank footprint,
        do NOT decay under ventilation (they model an active release), and
        are removed only by :meth:`clear_injected`. At most
        ``max_injected`` are held at once; one more raises ValueError.
        """
        key = self._gas_key(gas)
        # Non-finite input (JSON NaN/Infinity reach the facade's
        # payload/gas/inject) would turn every reading NaN: NaN compares
        # False against every threshold, i.e. a silent "no alarm".
        radius = _finite(radius, "radius")
        if radius <= 0.0:
            raise ValueError("radius must be > 0, got %r" % (radius,))
        ppm_or_pct = _finite(ppm_or_pct, "ppm_or_pct")
        x, y, z = (_finite(v, "spike position") for v in (x, y, z))
        if key != "o2_pct" and ppm_or_pct < 0.0:
            raise ValueError(
                "negative amplitude is only meaningful for gas='o2' "
                "(signed delta); got %r for %r" % (ppm_or_pct, gas))
        if self.injected_count >= self.max_injected:
            raise ValueError("at most %d injected pockets at once; clear them first"
                             % self.max_injected)
        self._pockets.append({
            "key": key,
            "center": (float(x), float(y)),
            "amplitude": float(ppm_or_pct),
            "sigma_xy": radius,
            "z_scale": None,
            "z_center": float(z),
            "sigma_z": radius,
            "masked_to_tank": False,
            "builtin": False,
        })

    @property
    def injected_count(self):
        """Number of runtime (injected) pockets currently active."""
        return sum(1 for p in self._pockets if not p["builtin"])

    def clear_injected(self):
        """Remove every injected spike; built-in pockets are untouched.

        Returns the number removed. Rebinds the pocket list (never mutates
        it in place), so a concurrent sample() iterating the old list is safe.
        """
        kept = [p for p in self._pockets if p["builtin"]]
        removed = len(self._pockets) - len(kept)
        self._pockets = kept
        return removed

    # -- forced ventilation -------------------------------------------- #

    def fraction_remaining(self, t=None):
        """Scale on the built-in contributions at sim time ``t``, in
        [residual_fraction, 1]. 1.0 = unventilated."""
        if not self._vent_on or t is None:
            return self._frac_anchor
        elapsed = max(0.0, _finite(t, "t") - self._vent_t_on)
        r = self.residual_fraction
        return r + (self._frac_anchor - r) * math.exp(-elapsed / self._vent_tau)

    def set_ventilation(self, on, t, tau_s=DEFAULT_VENT_TAU_S):
        """Switch forced ventilation on/off at sim time ``t`` (s).

        On: built-in contributions decay from their current fraction toward
        ``residual_fraction`` with time constant ``tau_s``. Already on with
        the same tau: no-op; with a new tau: re-anchored at ``t`` from the
        current fraction. Off: the current fraction is held (no
        re-accumulation). Returns :meth:`ventilation_state` at ``t``.
        """
        t = _finite(t, "t")
        tau_s = _finite(tau_s, "tau_s")
        if tau_s <= 0.0:
            raise ValueError("tau_s must be > 0, got %r" % (tau_s,))
        current = self.fraction_remaining(t)
        if on:
            if not (self._vent_on and tau_s == self._vent_tau):
                self._vent_on, self._vent_t_on = True, t
                self._vent_tau, self._frac_anchor = tau_s, current
        else:
            self._vent_on, self._vent_t_on, self._frac_anchor = False, None, current
        return self.ventilation_state(t)

    def reset_ventilation(self, ventilated=False, t=0.0):
        """Restore a scenario start: unventilated (fraction 1.0, off) or,
        with ``ventilated=True``, already vented (on since ``t``, fraction
        at the residual)."""
        if ventilated:
            self._vent_on, self._vent_t_on = True, _finite(t, "t")
            self._frac_anchor = self.residual_fraction
        else:
            self._vent_on, self._vent_t_on, self._frac_anchor = False, None, 1.0
        self._vent_tau = DEFAULT_VENT_TAU_S

    def ventilation_state(self, t=None):
        """``{"on", "t_on", "tau_s", "fraction_remaining",
        "residual_fraction"}`` at sim time ``t`` (t_on is None while off)."""
        return {
            "on": self._vent_on,
            "t_on": self._vent_t_on,
            "tau_s": self._vent_tau,
            "fraction_remaining": self.fraction_remaining(t),
            "residual_fraction": self.residual_fraction,
        }

    def probe(self, points, t=None, noise=True):
        """Samples at each (x, y, z) in ``points`` (a gas probe / sample
        line reaching into the space). All points are validated before any
        is sampled. Returns a list of :meth:`sample` dicts, in order."""
        pts = []
        for p in points:
            p = tuple(p)
            if len(p) != 3:
                raise ValueError("probe points are [x, y, z], got %r" % (p,))
            pts.append(tuple(_finite(v, "probe coordinate") for v in p))
        return [self.sample(x, y, z, t=t, noise=noise) for (x, y, z) in pts]

    @staticmethod
    def alarms(sample_dict):
        """Per-gas alarm booleans for a sample dict, per OSHA 1910.146 App E.

        o2: alarm if < 19.5 % or > 23.5 % (exact boundary values are OK).
        lel >= 10 %, h2s >= 10 ppm, co >= 35 ppm (boundary value alarms).
        A non-finite (NaN/inf) reading alarms (fail-safe, as agent/mission.py
        treats it). ``"any"`` is the OR of the four.
        """
        def bad(key):
            return not math.isfinite(sample_dict[key])

        result = {
            "o2": bool(bad("o2_pct") or sample_dict["o2_pct"] < O2_LOW_PCT
                       or sample_dict["o2_pct"] > O2_HIGH_PCT),
            "lel": bool(bad("lel_pct") or sample_dict["lel_pct"] >= LEL_ALARM_PCT),
            "h2s": bool(bad("h2s_ppm") or sample_dict["h2s_ppm"] >= H2S_ALARM_PPM),
            "co": bool(bad("co_ppm") or sample_dict["co_ppm"] >= CO_ALARM_PPM),
        }
        result["any"] = result["o2"] or result["lel"] or result["h2s"] or result["co"]
        return result

    def worst_case_along(self, points, z, t=None):
        """Worst-case noiseless readings along a path at height ``z``.

        ``points`` is an iterable of (x, y). Returns a sample-style dict
        holding the per-gas worst case over the path: the MAX reading for
        lel/h2s/co; for o2 the MIN (depletion is the primary modeled
        hazard), unless the path's max o2 exceeds the enrichment limit
        (23.5 %) by more than the min falls below the depletion limit
        (19.5 %) — then the MAX is returned so an o2-enrichment hazard
        (e.g. an injected positive o2 spike) is not silently dropped.
        Sampled without sensor noise so path planning is deterministic;
        ``t`` as in :meth:`sample`.
        """
        points = list(points)
        if not points:
            return dict(BASELINE)
        samples = [self.sample(px, py, z, t=t, noise=False) for (px, py) in points]
        worst = {
            "lel_pct": max(s["lel_pct"] for s in samples),
            "h2s_ppm": max(s["h2s_ppm"] for s in samples),
            "co_ppm": max(s["co_ppm"] for s in samples),
        }
        o2_min = min(s["o2_pct"] for s in samples)
        o2_max = max(s["o2_pct"] for s in samples)
        if (o2_max - O2_HIGH_PCT) > max(O2_LOW_PCT - o2_min, 0.0):
            worst["o2_pct"] = o2_max
        else:
            worst["o2_pct"] = o2_min
        return worst


class ProbeGeometry:
    """Reach and line-of-sight rules for the facade's gas probe.

    The probe is an ``extendable gas probe (simulated; arm-mounted on the
    real robot)`` held from Spot's body. :meth:`check` refuses a point when

    * the body pose is unknown (the probe cannot be aimed),
    * z is outside ``z_range`` (height above the tank floor),
    * it is more than ``max_reach`` m, measured horizontally, from the body's
      front (the nose, ``front_offset`` m ahead of the body center along its
      heading; the real Spot Arm reaches ~0.98 m), or
    * the straight segment from the body center (at body height) to the
      point passes through the tank shell anywhere but the doorway opening
      (``door_width`` wide, from the floor up to ``door_height``):
      "occluded by tank wall".

    Tank shell: a vertical cylinder of radius ``tank_radius`` around
    ``tank_center``; the opening is centered on ``door_xy`` (on the shell
    line) with outward normal ``door_normal_yaw``. The segment is tested
    against three concentric circles -- the shell line and +/- ``half_depth``
    (the door frame is 0.25 m deep) -- and every crossing must lie inside
    the opening. A chord between two interior points never crosses the
    shell (the footprint is convex), so probing from inside the tank is
    only limited by reach. Pure geometry, no sampling.
    """

    def __init__(self, tank_center=(0.0, 0.0), tank_radius=6.0, door_xy=(6.0, 0.0),
                 door_normal_yaw=0.0, door_width=DOOR_WIDTH_M, door_height=DOOR_HEIGHT_M,
                 half_depth=DOOR_FRAME_HALF_DEPTH_M, max_reach=PROBE_MAX_REACH_M,
                 front_offset=BODY_FRONT_OFFSET_M, z_range=PROBE_Z_RANGE_M):
        self.cx, self.cy = (_finite(v, "tank_center") for v in tank_center)
        self.radius = _finite(tank_radius, "tank_radius")
        self.door_x, self.door_y = (_finite(v, "door_xy") for v in door_xy)
        yaw = _finite(door_normal_yaw, "door_normal_yaw")
        self.normal = (math.cos(yaw), math.sin(yaw))
        self.tangent = (-self.normal[1], self.normal[0])
        self.half_width = _finite(door_width, "door_width") / 2.0
        self.door_height = _finite(door_height, "door_height")
        self.half_depth = _finite(half_depth, "half_depth")
        self.max_reach = _finite(max_reach, "max_reach")
        self.front_offset = _finite(front_offset, "front_offset")
        self.z_range = (float(z_range[0]), float(z_range[1]))
        if self.radius <= self.half_depth or self.half_width <= 0.0 or self.max_reach <= 0.0:
            raise ValueError("ProbeGeometry: bad tank/door/reach dimensions")

    def limits(self):
        """JSON-ready description of the rules (published on payload/site)."""
        return {
            "label": PROBE_LABEL,
            "max_reach_m": self.max_reach,
            "reach_from": "body_front",
            "reach_metric": "horizontal",
            "body_front_offset_m": self.front_offset,
            "z_range_m": list(self.z_range),
            "requires_pose": True,
            "line_of_sight": "straight body-center -> point segment may cross the tank "
                             "shell only inside the doorway opening",
            "door_width_m": round(2.0 * self.half_width, 4),
            "door_height_m": self.door_height,
        }

    def inside_tank(self, x, y):
        return math.hypot(x - self.cx, y - self.cy) < self.radius

    def front_xy(self, body_x, body_y, body_yaw):
        return (body_x + self.front_offset * math.cos(body_yaw),
                body_y + self.front_offset * math.sin(body_yaw))

    def _crossings(self, p0, p1, r):
        """Parameters t in [0, 1] where the horizontal projection of the
        segment p0 -> p1 crosses the circle of radius r."""
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        fx, fy = p0[0] - self.cx, p0[1] - self.cy
        a = dx * dx + dy * dy
        if a < 1e-12:
            return []
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r
        disc = b * b - 4.0 * a * c
        if disc <= 0.0:  # misses, or only grazes the circle
            return []
        root = math.sqrt(disc)
        return [t for t in ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a)) if 0.0 <= t <= 1.0]

    def _in_opening(self, x, y, z):
        rx, ry = x - self.door_x, y - self.door_y
        along = (x - self.cx) * self.normal[0] + (y - self.cy) * self.normal[1]
        lateral = rx * self.tangent[0] + ry * self.tangent[1]
        return along > 0.0 and abs(lateral) <= self.half_width and 0.0 <= z <= self.door_height

    def occluded(self, body_xyz, point):
        """True if the body-center -> point segment crosses the shell (any of
        the three test circles) outside the doorway opening."""
        p0, p1 = tuple(body_xyz), tuple(point)
        for r in (self.radius - self.half_depth, self.radius, self.radius + self.half_depth):
            for t in self._crossings(p0, p1, r):
                x = p0[0] + t * (p1[0] - p0[0])
                y = p0[1] + t * (p1[1] - p0[1])
                z = p0[2] + t * (p1[2] - p0[2])
                if not self._in_opening(x, y, z):
                    return True
        return False

    def check(self, body_pose, point):
        """None if ``point`` = (x, y, z) may be probed from ``body_pose`` =
        (x, y, z, yaw) (or None when unknown), else a short error string."""
        if body_pose is None:
            return "body pose unavailable: the gas probe cannot be aimed"
        bx, by, bz, yaw = (float(v) for v in body_pose)
        x, y, z = (float(v) for v in point)
        if not all(math.isfinite(v) for v in (bx, by, bz, yaw, x, y, z)):
            return "non-finite body pose or probe point"
        if not self.z_range[0] <= z <= self.z_range[1]:
            return "probe z %.2f outside %s m" % (z, self.z_range)
        fx, fy = self.front_xy(bx, by, yaw)
        reach = math.hypot(x - fx, y - fy)
        if reach > self.max_reach:
            return ("point (%.2f, %.2f, %.2f) is %.2f m from Spot's body front, beyond the "
                    "%.1f m probe reach" % (x, y, z, reach, self.max_reach))
        if self.occluded((bx, by, bz), (x, y, z)):
            return ("point (%.2f, %.2f, %.2f) occluded by tank wall: the probe line from the "
                    "body crosses the shell outside the %.2f x %.2f m doorway"
                    % (x, y, z, 2.0 * self.half_width, self.door_height))
        return None
