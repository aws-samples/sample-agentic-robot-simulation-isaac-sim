#!/usr/bin/env bash
# set_view.sh -- switch the GUI presenter camera or layout of the running sim.
#
# Runs ON the EC2 sim host (presenter console). Publishes one message on
# /demo_spot/payload/view (sim/isaac_app.py applies it at the next tick; the
# sim log prints "VIEW camera=... ui=..."). GUI mode only: a headless sim
# accepts it but has no viewport.
#
#   bash set_view.sh auto        # default: low 3/4 chase outside, interior corner camera inside
#   bash set_view.sh chase       # chase camera everywhere (lower and closer inside)
#   bash set_view.sh interior    # fixed corner camera inside the tank, pans/zooms on Spot
#   bash set_view.sh overhead    # under the roof, straight down, follows Spot (route plan view)
#   bash set_view.sh door        # fixed camera beside the cleanout door (probe close-up)
#   bash set_view.sh --ui editor        # full Isaac Sim editor (panels, menus) without a restart
#   bash set_view.sh --ui presentation  # back to the fullscreen viewport
#
# Every view change is a hard cut. host/reset_spot.sh puts the camera back on
# auto. Exit 0 once a subscriber (the sim) has matched and the message is out.
set -euo pipefail
CAMERA=""; UI=""
while [ $# -gt 0 ]; do
  case "$1" in
    auto|chase|interior|overhead|door) CAMERA="$1"; shift ;;
    --ui) UI="${2:-}"; shift 2 || { echo "--ui needs presentation|editor" >&2; exit 2; } ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1 (see --help)" >&2; exit 2 ;;
  esac
done
case "$UI" in ""|presentation|editor) ;; *) echo "--ui must be presentation or editor" >&2; exit 2 ;; esac
[ -n "$CAMERA$UI" ] || { sed -n '2,17p' "$0"; exit 2; }

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$HERE/sim_lib.sh"
# Own non-root uid, no capabilities, read-only root, DDS isolation (T25, T28).
spot_container_args oneshot || exit 2
spot_pinned_image SPOT_ROS_IMAGE || exit 2

exec docker run --rm --network=host "${SPOT_ARGS[@]}" \
  -e VIEW_CAMERA="$CAMERA" -e VIEW_UI="$UI" \
  "$SPOT_ROS_IMAGE" bash -c 'source /opt/ros/jazzy/setup.bash && python3 - <<"PY"
import json, os, sys, time
import rclpy
from std_msgs.msg import String

msg = {k: v for k, v in (("camera", os.environ["VIEW_CAMERA"]), ("ui", os.environ["VIEW_UI"])) if v}
rclpy.init()
node = rclpy.create_node("presenter_view")
pub = node.create_publisher(String, "/demo_spot/payload/view", 10)
end = time.monotonic() + 10.0
while pub.get_subscription_count() == 0 and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
if pub.get_subscription_count() == 0:
    sys.exit("no subscriber for /demo_spot/payload/view within 10 s: is the sim running (and new enough)?")
pub.publish(String(data=json.dumps(msg)))
t = time.monotonic() + 0.5  # let the sample go out before the participant dies
while time.monotonic() < t:
    rclpy.spin_once(node, timeout_sec=0.05)
print("view request sent: %s" % json.dumps(msg))
PY'
