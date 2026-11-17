"""Simulated Spot battery for the demo facade (pure Python, no Isaac imports).

What it is
----------
``payload/status`` carries a ``battery_pct`` that the HUD shows as a chip
(amber below 30 %, red below 15 %: hud/static/app.js) and that the agent may
read out. Nothing in the sim consumes energy, so the value is a linear
discharge of SIM time since the last reset, in one of three modes
(``isaac_app.py --battery-drain``):

``demo`` (default)  25 %/h, never below 30 %: a 2 h session ends at 50 %, the
                    chip stays green all day. (v1 drained 95 %/h: amber after
                    ~44 min, red after ~54 min, i.e. mid-demo.)
``real``            about a real Spot's runtime: full to empty in ~90 min
                    (100 %/90 min), floor 5 %.
``off``             100 % constantly.

``reset(sim_time)`` recharges to 100 % (host/reset_spot.sh publishes
``/demo_spot/payload/battery/reset`` between runs).

Unit tests: ``python3 -m unittest tests.test_battery`` (Mac, stdlib only).
"""

BATTERY_MODES = ("demo", "real", "off")
DEFAULT_BATTERY_MODE = "demo"

# mode -> (drain in % per sim second, floor %)
_RATES = {
    "demo": (25.0 / 3600.0, 30.0),
    "real": (100.0 / (90.0 * 60.0), 5.0),
    "off": (0.0, 100.0),
}


class SimBattery:
    """Linear discharge of sim time since the last reset, clamped at a floor."""

    def __init__(self, mode=DEFAULT_BATTERY_MODE, t0=0.0):
        mode = str(mode).strip().lower()
        if mode not in _RATES:
            raise ValueError(f"battery mode must be one of {', '.join(BATTERY_MODES)}, got {mode!r}")
        self.mode = mode
        self.rate_pct_per_s, self.floor_pct = _RATES[mode]
        self.t0 = float(t0)

    def pct(self, sim_time):
        """Charge in % at ``sim_time`` (rounded to 0.01)."""
        elapsed = max(0.0, float(sim_time) - self.t0)
        return round(max(self.floor_pct, 100.0 - elapsed * self.rate_pct_per_s), 2)

    def reset(self, sim_time):
        """Back to 100 % at ``sim_time`` (the drain restarts from there)."""
        self.t0 = float(sim_time)

    def describe(self):
        if self.rate_pct_per_s <= 0.0:
            return f"mode={self.mode} constant 100 %"
        to_floor_min = (100.0 - self.floor_pct) / self.rate_pct_per_s / 60.0
        return (f"mode={self.mode} drain={self.rate_pct_per_s * 3600.0:.1f} %/h "
                f"floor={self.floor_pct:.0f} % (reached after {to_floor_min:.0f} min of sim time)")
