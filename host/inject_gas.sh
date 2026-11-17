#!/usr/bin/env bash
# inject_gas.sh -- presenter console for the SIMULATED tank atmosphere.
#
# Runs ON the EC2 sim host, from a second shell while a mission or the agent
# is driving. Every action is also published as a disclosure event on
# /demo_spot/payload/events, so the HUD transcript shows it was staged /
# operator-driven. The SafetyGuard in agent/mission.py -- not this script --
# reacts to the resulting readings.
#
#   bash inject_gas.sh                 # 80 ppm H2S, 1.0 m radius, z 0.3 m,
#                                      # at Spot's current position
#     (1 m, not 2 m: a 2 m pocket at x~4 still read ~14 ppm H2S at the
#     sit standoff (8, 0) after the retreat -- measured 2026-09-27)
#   bash inject_gas.sh lel 30 1.5 0.5  # gas amount [radius_m] [z_m]
#   bash inject_gas.sh clear           # remove every injected pocket
#   bash inject_gas.sh vent on [tau_s] # start forced ventilation (no tau_s: the sim keeps its
#                                      # current tau -- 15 s unless changed since start/reset)
#   bash inject_gas.sh vent off        # stop it (readings hold, no re-accumulation)
#   bash inject_gas.sh vent reset      # back to the sim's --gas-initial state
#
# gas: h2s|lel|o2|co; amount: ppm (h2s, co), % points (lel), or for o2 the
# % points of oxygen removed, as a negative number from -20.9 up to (not
# including) 0: an O2 pocket is a depletion only. tau_s: 2 to 3600 s.
# The sim refuses the same (positive O2 deltas would mask a depletion, T23);
# this script refuses them first, before a container starts.
# Injected pockets model an active release: ventilation does not dilute
# them; only `clear` (or reset_spot.sh) removes them.
set -euo pipefail
is_num() { [[ "$1" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; }
# in_range VALUE LOW HIGH [open]: LOW <= VALUE <= HIGH (or < HIGH with "open").
in_range() {
  awk -v v="$1" -v lo="$2" -v hi="$3" -v open="${4:-}" \
    'BEGIN { exit !(v + 0 >= lo + 0 && (open == "open" ? v + 0 < hi + 0 : v + 0 <= hi + 0)) }'
}

MODE=inject; GAS=""; AMOUNT=""; RADIUS=""; Z=""; VENT=""; TAU=""
case "${1:-h2s}" in
  clear) MODE=clear ;;
  vent)
    MODE=vent; VENT="${2:-}"; TAU="${3:-}"
    case "$VENT" in on|off|reset) ;; *) echo "usage: inject_gas.sh vent on [tau_s] | off | reset" >&2; exit 2 ;; esac
    if [ -n "$TAU" ]; then
      is_num "$TAU" || { echo "not a number: $TAU" >&2; exit 2; }
      in_range "$TAU" 2 3600 || { echo "tau_s must be from 2 to 3600 s, not $TAU" >&2; exit 2; }
    fi
    ;;
  h2s|lel|o2|co)
    GAS="${1:-h2s}"; AMOUNT="${2:-80}"; RADIUS="${3:-1.0}"; Z="${4:-0.3}"
    for v in "$AMOUNT" "$RADIUS" "$Z"; do is_num "$v" || { echo "not a number: $v" >&2; exit 2; }; done
    if [ "$GAS" = o2 ] && ! in_range "$AMOUNT" -20.9 0 open; then
      echo "o2 amount must be a depletion from -20.9 up to (not including) 0, not $AMOUNT" >&2; exit 2
    fi
    ;;
  *) echo "usage: inject_gas.sh [h2s|lel|o2|co amount radius z] | clear | vent on|off|reset" >&2; exit 2 ;;
esac

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$HERE/sim_lib.sh"
# Own non-root uid, no capabilities, read-only root, DDS isolation (T25, T28).
spot_container_args oneshot || exit 2
spot_pinned_image SPOT_ROS_IMAGE || exit 2

exec docker run --rm --network=host "${SPOT_ARGS[@]}" \
  -e MODE="$MODE" -e GAS="$GAS" -e AMOUNT="$AMOUNT" -e RADIUS="$RADIUS" -e Z="$Z" \
  -e VENT="$VENT" -e TAU="$TAU" \
  "$SPOT_ROS_IMAGE" bash -c 'source /opt/ros/jazzy/setup.bash && python3 - <<"PY"
import json, os, sys, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

mode = os.environ["MODE"]
topic = {"inject": "inject", "clear": "clear", "vent": "ventilation"}[mode]
rclpy.init()
node = rclpy.create_node("presenter_gas")
# Event stamps: "wall_t" = unix wall time; "t" = sim time when payload/status
# was seen (its sim_time + wall time since, capped at 1 s, like
# agent/robot_client.py sim_time()), else wall time; "t_source" says which.
sim_clock = []
node.create_subscription(String, "/demo_spot/payload/status",
                         lambda m: sim_clock.append((time.monotonic(), json.loads(m.data).get("sim_time"))), 10)


def stamps(wait_s=0.0):
    end = time.monotonic() + wait_s  # bounded wait for a status sample (never before a spike)
    while not sim_clock and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    wall = time.time()
    if sim_clock and isinstance(sim_clock[-1][1], (int, float)) and time.monotonic() - sim_clock[-1][0] < 2.0:
        rx, st = sim_clock[-1]
        return {"t": round(st + min(time.monotonic() - rx, 1.0), 4), "wall_t": round(wall, 3), "t_source": "sim"}
    return {"t": round(wall, 3), "wall_t": round(wall, 3), "t_source": "wall"}
pose = []
node.create_subscription(Odometry, "/demo_spot/odometry",
                         lambda m: pose.append((m.pose.pose.position.x, m.pose.pose.position.y)), 10)
pub = node.create_publisher(String, "/demo_spot/payload/gas/" + topic, 10)
events = node.create_publisher(String, "/demo_spot/payload/events", QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
need_pose = mode == "inject"
end = time.monotonic() + 10.0
while ((need_pose and not pose) or pub.get_subscription_count() == 0) and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
if (need_pose and not pose) or pub.get_subscription_count() == 0:
    sys.exit("no odometry / no payload/gas/%s subscriber within 10 s: is the sim running?" % topic)

if mode == "inject":
    x, y = pose[-1]
    body = {"gas": os.environ["GAS"], "ppm": float(os.environ["AMOUNT"]), "x": round(x, 3),
            "y": round(y, 3), "z": float(os.environ["Z"]), "radius": float(os.environ["RADIUS"])}
    source, kind = "presenter", "demo_inject"
    text = "PRESENTER DEMO: simulated %s pocket injected at (%.1f, %.1f)" % (body["gas"].upper(), x, y)
elif mode == "clear":
    body, source, kind = {}, "presenter", "demo_clear"
    text = "PRESENTER DEMO: all injected (simulated) gas pockets cleared"
else:
    vent, tau = os.environ["VENT"], os.environ["TAU"]
    if vent == "reset":
        body = {"reset": True}
        text = "PRESENTER DEMO: simulated tank atmosphere reset to its starting state"
        source = "presenter"
    else:
        body = {"on": vent == "on"}
        if tau:
            body["tau_s"] = float(tau)
        text = "Operator: forced ventilation %s (simulated%s)" % (
            vent.upper(), ", tau %s s" % tau if tau else "")
        source = "operator"
    kind = "ventilation"
pub.publish(String(data=json.dumps(body)))
# The disclosure goes out right after an injection (the guard alarm follows
# within ~0.1 s and must already see it); other controls may wait up to 1 s
# for a sim-time stamp.
events.publish(String(data=json.dumps(dict(
    stamps(0.0 if mode == "inject" else 1.0), source=source, kind=kind, message=text, data=body))))
# Let reliable TRANSIENT_LOCAL delivery finish before exiting: at least 1 s,
# and (up to 3 s) until an events subscriber (HUD / agent / mission) has
# matched this writer plus 0.3 s -- a presenter container is a new DDS
# participant, and a disclosure published before discovery completes would
# otherwise die with it.
t0, matched = time.monotonic(), None
while time.monotonic() - t0 < 3.0:
    now = time.monotonic()
    if matched is None and events.get_subscription_count() > 0:
        matched = now
    if now - t0 >= 1.0 and matched is not None and now - matched >= 0.3:
        break
    rclpy.spin_once(node, timeout_sec=0.05)
print(mode, topic, json.dumps(body))
PY'
