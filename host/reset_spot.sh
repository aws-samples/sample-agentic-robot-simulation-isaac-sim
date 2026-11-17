#!/usr/bin/env bash
# reset_spot.sh -- put the demo back to a clean start between runs, no sim restart.
#
# Runs ON the EC2 sim host. In order:
#   1. clears every injected (presenter) gas pocket   -> payload/gas/clear
#   2. with --reset-ventilation (alias --raw): resets the simulated tank
#      atmosphere to the sim's --gas-initial state     -> payload/gas/ventilation {"reset": true}
#      (default run: unventilated, so the pre-entry test fails again)
#   3. recharges the simulated battery to 100 %      -> payload/battery/reset
#      and puts the GUI presenter camera back on "auto" -> payload/view
#      (both best effort: a sim without them only gets a WARN)
#   4. calls self_right, which re-initializes the policy from the default
#      state (restart_from_default_state): Spot teleports to spawn (14, 0)
#      facing the tank, standing, in ~1 s.
# and publishes one disclosure event (presenter / demo_reset) for the HUD.
#
#   bash reset_spot.sh                      # clear gas spikes + reset Spot
#   bash reset_spot.sh --reset-ventilation  # ...and un-ventilate the tank
#
# Exit 0 only if the gas steps were delivered and self_right succeeded.
set -euo pipefail
RESET_VENT=0
for a in "$@"; do
  case "$a" in
    --reset-ventilation|--raw) RESET_VENT=1 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown option: $a (see --help)" >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$HERE/sim_lib.sh"
# Own non-root uid, no capabilities, read-only root, DDS isolation (T25, T28).
spot_container_args oneshot || exit 2
spot_pinned_image SPOT_ROS_IMAGE || exit 2

exec docker run --rm --network=host "${SPOT_ARGS[@]}" \
  -e RESET_VENT="$RESET_VENT" \
  "$SPOT_ROS_IMAGE" bash -c 'source /opt/ros/jazzy/setup.bash && python3 - <<"PY"
import json, os, sys, time
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

reset_vent = os.environ["RESET_VENT"] == "1"
rclpy.init()
node = rclpy.create_node("presenter_reset")
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
pubs = {"clear": node.create_publisher(String, "/demo_spot/payload/gas/clear", 10)}
if reset_vent:
    pubs["ventilation"] = node.create_publisher(String, "/demo_spot/payload/gas/ventilation", 10)
# Best effort (a sim older than these topics has no subscriber): battery back
# to 100 % (sim/battery.py) and the GUI presenter camera back to "auto".
extras = {"battery": node.create_publisher(String, "/demo_spot/payload/battery/reset", 10),
          "view": node.create_publisher(String, "/demo_spot/payload/view", 10)}
events = node.create_publisher(String, "/demo_spot/payload/events", QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
cli = node.create_client(Trigger, "/demo_spot/self_right")
end = time.monotonic() + 10.0
while any(p.get_subscription_count() == 0 for p in pubs.values()) and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
missing = [k for k, p in pubs.items() if p.get_subscription_count() == 0]
if missing or not cli.wait_for_service(timeout_sec=max(0.1, end - time.monotonic())):
    sys.exit("no subscriber for payload/gas/%s or no self_right service within 10 s: is the sim running?"
             % ",".join(missing or ["-"]))

pubs["clear"].publish(String(data="{}"))
if reset_vent:
    pubs["ventilation"].publish(String(data=json.dumps({"reset": True})))
end = time.monotonic() + 1.0  # same participant as the sim topics above: usually matched already
while any(p.get_subscription_count() == 0 for p in extras.values()) and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
extras_ok = {k: p.get_subscription_count() > 0 for k, p in extras.items()}
extras["battery"].publish(String(data="{}"))
extras["view"].publish(String(data=json.dumps({"camera": "auto"})))
for k, ok_k in extras_ok.items():
    if not ok_k:
        print("WARN: no subscriber for the %s reset (older sim?); skipped" % k, file=sys.stderr)
t = time.monotonic() + 0.5  # gas controls land before the teleport
while time.monotonic() < t:
    rclpy.spin_once(node, timeout_sec=0.05)

fut = cli.call_async(Trigger.Request())
end = time.monotonic() + 10.0
while not fut.done() and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
res = fut.result() if fut.done() else None
ok = bool(res and res.success)
text = ("PRESENTER DEMO: reset -- injected gas pockets cleared%s%s; Spot returned to spawn"
        % ("; tank atmosphere back to its starting (simulated) state" if reset_vent else "",
           "; simulated battery recharged" if extras_ok["battery"] else ""))
events.publish(String(data=json.dumps(dict(
    stamps(1.0), source="presenter", kind="demo_reset", message=text,
    data={"reset_ventilation": reset_vent, "self_right": ok,
          "battery_reset": extras_ok["battery"], "view_reset": extras_ok["view"]}))))
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
print("gas cleared%s%s; self_right: %s" % (" + ventilation reset" if reset_vent else "",
                                          " + battery 100 % + view auto" if all(extras_ok.values()) else "",
                                          res.message if res else "TIMEOUT"))
sys.exit(0 if ok else 1)
PY'
