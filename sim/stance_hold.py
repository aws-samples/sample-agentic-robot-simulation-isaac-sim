"""Heading hold for the facade's standing RL policy (pure stdlib, no Isaac).

Why: measured on the sim host 2026-09-27, after any walk the Isaac Spot
flat-terrain policy, commanded zero (watchdog-zeroed or explicit zero
twists), keeps stepping in place and creeps into a self-accelerating
counter-clockwise turn: 0.02 -> 0.16 rad/s, ~40 deg within 12 s, then it
settles (abrupt stop, ramped stop and explicit zeros alike; a fresh stand
with no walk holds within 0.6 deg for 90 s). Real Spot holds its heading
when standing still. At the cleanout door that turn swung the nose off the
door axis during the ~48 s ventilation wait, the 1.0 m-deep probe points
fell out of the 1.1 m reach (1.17 m, then 1.44 m) and the scripted
storyline ABORTED; a 40 deg turn in the doorway could also sweep Spot's
corners into the jambs.

:class:`HeadingHold` stands in for the real robot's stance controller: while
the policy is active and the command is zero, it latches the heading and
feeds the policy a proportional yaw-rate correction back to it (1 deg dead
band, 0.4 rad/s cap). Any non-zero command (a walk, a turn, a retreat)
releases it; the next zero command latches the new heading. Positions are
not corrected (the creep moved the body center only ~3 cm).
"""

import math

HOLD_YAW_DEADBAND_RAD = math.radians(1.0)
HOLD_YAW_GAIN = 1.5            # yaw-rate command per rad of heading error
HOLD_YAW_MAX_RAD_S = 0.4
MOVING_EPS = 1e-3              # |component| above this = an operator command


def wrap(angle):
    """Angle wrapped to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class HeadingHold:
    """Stateful heading hold; one instance per robot.

    ``command(cmd, yaw)`` -> the (vx, vy, wz) to give the policy, where
    ``cmd`` is the facade's command after the watchdog (zeros when stale)
    and ``yaw`` the current body yaw (rad) or None when unknown.
    ``release()`` forgets the latched heading (policy off, stand-up, reset).
    """

    def __init__(self, deadband=HOLD_YAW_DEADBAND_RAD, gain=HOLD_YAW_GAIN,
                 max_rate=HOLD_YAW_MAX_RAD_S):
        self.deadband = float(deadband)
        self.gain = float(gain)
        self.max_rate = float(max_rate)
        self.yaw = None           # latched heading (rad) or None

    def release(self):
        self.yaw = None

    def command(self, cmd, yaw):
        vx, vy, wz = (float(v) for v in cmd)
        if max(abs(vx), abs(vy), abs(wz)) > MOVING_EPS or yaw is None or not math.isfinite(yaw):
            self.yaw = None
            return vx, vy, wz
        if self.yaw is None:
            self.yaw = float(yaw)
            return 0.0, 0.0, 0.0
        err = wrap(self.yaw - float(yaw))
        if abs(err) < self.deadband:
            return 0.0, 0.0, 0.0
        return 0.0, 0.0, max(-self.max_rate, min(self.max_rate, self.gain * err))
