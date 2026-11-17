"""Rehearsal summary: mission JSONL audit log + odometry/gas recording -> one JSON line.

What it is:
    The host-side analysis helper behind the RUNBOOK's "Measured rehearsal
    numbers" tables (was /opt/spot-demo/tools/rh_analyze.py on the sim
    host, copied into the repo 2026-09-27). Pure stdlib; runs anywhere
    (host or Mac) on files pulled with scripts/fetch-log.sh.

Usage:
    python3 tools/rh_analyze.py MISSION.jsonl ODOM.jsonl [LABEL]

Inputs:
    MISSION.jsonl  the mission audit log (agent/mission.py records: kind,
                   message, data, t, and on newer logs wall_t).
    ODOM.jsonl     tools/odom_rec.py output recorded alongside the run
                   ({"t", "k": "odom", "x", "y", "z", "yaw"} at 10 Hz and
                   {"t", "k": "gas", "g": payload/gas}).

Output (stdout, one JSON object): outcome, permit, duration, reason, the
atmospheric tests (pass + worst readings), in-space gas extremes, waypoints,
ventilation wait, injection -> alarm latency (ms), stop-command and reading
age (ms), alarm -> retreat start (ms), retreat result/duration, final sit,
path length, min door clearance (body box to jamb), max |y| at the wall
line, min body z inside / overall, final odometry.

Timestamps: event-to-event latencies use ``wall_t`` (unix wall time) when
both records carry it, else ``t``. Since the P2-9 change mission/agent/guard
``t`` is sim time and presenter events from host scripts stamp ``t`` with sim
time only when they saw payload/status, so ``wall_t`` is the common clock.

Geometry (build_scene.py): tank r = 6 at (0, 0), doorway faces +X, jamb
inner faces at |y| = 0.65 for x in [5.875, 6.125]. Spot body footprint
1.10 x 0.50 m (spec L x W). Door clearance = closest distance between that
box (at each odometry pose within 1 m of the door) and sample points on
the jamb faces; negative = penetration.
"""
import json
import math
import sys

HALF_L, HALF_W = 0.55, 0.25
JAMB_X = [5.875 + 0.025 * i for i in range(11)]
JAMB_Y = 0.65
R = 6.0


def rect_dist(bx, by, yaw, px, py):
    dx, dy = px - bx, py - by
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    ox, oy = abs(lx) - HALF_L, abs(ly) - HALF_W
    if ox <= 0 and oy <= 0:
        return max(ox, oy)  # negative = penetration
    return math.hypot(max(ox, 0.0), max(oy, 0.0))


def main():
    log, rec = sys.argv[1], sys.argv[2]
    label = sys.argv[3] if len(sys.argv) > 3 else ""
    ev = [json.loads(line) for line in open(log) if line.strip()]
    od = [json.loads(line) for line in open(rec) if line.strip()]
    odom = [r for r in od if r["k"] == "odom"]
    out = {"label": label, "events": len(ev)}
    summ = [e for e in ev if e["kind"] == "summary"]
    if summ:
        d = summ[-1]["data"]
        out.update(outcome=d["outcome"], permit=d["permit"], duration_s=d["duration_s"],
                   reason=d["reason"][:160], abort_cause=d["abort_cause"],
                   as_designed=d["aborted_as_designed"], outside=d["outside_permit_space"],
                   wp=f'{d["waypoints_reached"]}/{d["waypoints_total"]}',
                   tests=[{"passed": t["passed"], "worst": {k: round(v, 2) if isinstance(v, (int, float)) else v
                                                           for k, v in (t.get("worst") or {}).items()}}
                          for t in d.get("atmospheric_tests") or []],
                   in_space={k: [round(v["min"], 2), round(v["max"], 2)]
                             for k, v in (d.get("gas_extremes_in_space") or {}).items()},
                   final_pose=[round(v, 2) for v in (d.get("final_pose") or [])][:3])
    vent = [e for e in ev if e["kind"] == "ventilation" and "Purge target reached" in e["message"]]
    if vent:
        out["vent_waited_s"] = (vent[-1].get("data") or {}).get("waited_s")
    def clock(e, ref):
        """wall_t when both events have it (one clock across sources), else t."""
        if e.get("wall_t") is not None and ref.get("wall_t") is not None:
            return e["wall_t"], ref["wall_t"]
        return e["t"], ref["t"]

    inj = [e for e in ev if e["kind"] in ("demo_inject",)]
    alarm = [e for e in ev if e["kind"] == "alarm"]
    if inj and alarm:
        a = [x for x in alarm if clock(x, inj[0])[0] >= clock(x, inj[0])[1] - 0.5]
        if a:
            ta, ti = clock(a[0], inj[0])
            out["inject_to_alarm_ms"] = round(1000 * (ta - ti), 0)
            out["alarm_msg"] = a[0]["message"][:100]
            out["stop_command_ms"] = (None if a[0]["data"].get("stop_command_s") is None
                                      else round(1000 * a[0]["data"]["stop_command_s"], 1))
            out["reading_age_ms"] = (None if a[0]["data"].get("reading_age_s") is None
                                     else round(1000 * a[0]["data"]["reading_age_s"], 0))
    rs = [e for e in ev if e["kind"] == "retreat_start"]
    rc = [e for e in ev if e["kind"] in ("retreat_complete", "retreat_incomplete")]
    if rc:
        out["retreat"] = rc[-1]["kind"]
        out["retreat_s"] = (rc[-1].get("data") or {}).get("duration_s")
        if alarm and rs:
            tr, ta = clock(rs[0], alarm[0])
            out["alarm_to_retreat_start_ms"] = round(1000 * (tr - ta), 0)
    sits = [e for e in ev if e["kind"] == "robot" and (e.get("message") or "").startswith("sit:")]
    out["sit"] = sits[-1]["message"][:40] if sits else None
    L = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(odom, odom[1:]))
    out["path_m"] = round(L, 1)
    clear = []
    for o in odom:
        if abs(o["x"] - 6.0) > 1.0 or abs(o["y"]) > 1.2:
            continue
        for sgn in (1, -1):
            for jx in JAMB_X:
                clear.append(rect_dist(o["x"], o["y"], o["yaw"], jx, sgn * JAMB_Y))
    out["min_door_clearance_m"] = round(min(clear), 3) if clear else None
    cross = [abs(o["y"]) for o in odom if abs(o["x"] - 6.0) < 0.15]
    out["max_abs_y_at_wall"] = round(max(cross), 3) if cross else None
    zin = [o["z"] for o in odom if math.hypot(o["x"], o["y"]) < R - 0.5]
    out["min_z_inside"] = round(min(zin), 2) if zin else None
    out["min_z_all"] = round(min(o["z"] for o in odom), 2) if odom else None
    if odom:
        out["final_odom"] = [round(odom[-1]["x"], 2), round(odom[-1]["y"], 2), round(odom[-1]["z"], 2)]
    print(json.dumps(out))


if __name__ == "__main__":
    main()
