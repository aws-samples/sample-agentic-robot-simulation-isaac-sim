#!/bin/bash
# End-to-end ROS 2 test of the Spot sim facade, run ON the sim host.
#
# Starts sim/isaac_app.py (headless, no cameras) in the background via
# run_sim.sh, then runs host/e2e_client.py inside a ros:jazzy container on the
# host network. Isaac 6.1 ships an internal Jazzy rclpy, so the two talk over
# Fast DDS with the same DDS isolation (host/sim_lib.sh spot_dds_args: UDPv4 on
# loopback only, discovery to 127.0.0.1 only), on the test domain
# ${SPOT_TEST_DOMAIN_ID:-43} rather than the demo's (spot_test_domain, T41).
#
# Usage: ACCEPT_EULA=Y bash e2e_ros_test.sh [--force]   (exit 0 = all checks
#        passed; Y once you accept the NVIDIA licence, see run_sim.sh)
# It replaces any running sim, so it refuses while an agent or mission
# container runs; --force overrides. At exit it removes its own sim only,
# never one started since (T41).
# Logs:  NEW per-run files $SPOT_DEMO_DIR/logs/e2e-sim-<run_id>.log and
#        e2e-client-<run_id>.log (symlinks e2e-{sim,client}-latest.log).
#
# Readiness is keyed to THIS launch: the client starts only after the current
# spot-sim container printed "FACADE-READY ... run_id=<run_id>" (docker logs).
# The old version wrote /tmp/spot-e2e-*.log; a root-owned leftover there
# (fs.protected_regular=2) could not be reopened by ubuntu and a stale
# FACADE-READY line was matched with no sim running.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$HERE/sim_lib.sh"
ROS_IMAGE="${ROS_IMAGE:-$SPOT_ROS_IMAGE}"   # pinned by digest (T31)
spot_pinned_image ROS_IMAGE || exit 2
READY_TIMEOUT="${SPOT_SIM_READY_TIMEOUT:-360}"
# run_sim.sh starts Isaac only if the operator accepted the NVIDIA licence
# (T35); say so now rather than after a failed launch.
if [ "${ACCEPT_EULA:-}" != Y ]; then
  echo "FATAL: set ACCEPT_EULA=Y yourself once you accept the NVIDIA licence (see host/run_sim.sh)" >&2
  exit 2
fi
FORCE=0
case "${1:-}" in
  "") ;;
  --force) FORCE=1 ;;
  *) echo "usage: e2e_ros_test.sh [--force]" >&2; exit 2 ;;
esac
spot_guard_sim_stop "$FORCE" "replace the running sim with the e2e test" || exit 1
spot_test_domain || exit 2

RUN_ID="$(spot_run_id)"
SIM_LOG="$(spot_new_log e2e-sim "$RUN_ID")" || exit 1
CLIENT_LOG="$(spot_new_log e2e-client "$RUN_ID")" || exit 1
echo "e2e: run_id=$RUN_ID sim log=$SIM_LOG client log=$CLIENT_LOG"

# shellcheck disable=SC2329  # invoked by the EXIT trap
cleanup() { spot_rm_own_sim "$RUN_ID"; }
trap cleanup EXIT

docker image inspect "$ROS_IMAGE" >/dev/null 2>&1 || docker pull -q "$ROS_IMAGE"

SPOT_RUN_ID="$RUN_ID" bash "$HERE/run_sim.sh" --force --headless true --no-cameras > "$SIM_LOG" 2>&1 &
SIM_PID=$!

# Wait for THIS launch's facade node (never a line from an older run).
if ! ready="$(spot_wait_sim_ready "$RUN_ID" "$READY_TIMEOUT" "$SIM_LOG" "pid:$SIM_PID")"; then
  echo "FATAL: sim not ready; e2e client not started"
  exit 1
fi
echo "e2e: $ready"

# The client as a one-shot: own uid, no capabilities, read-only root and
# its script mounted read-only (T28), DDS isolation (T25).
spot_container_args oneshot || exit 2
set +e
timeout 600 docker run --rm --network=host \
  "${SPOT_ARGS[@]}" \
  -v "$HERE/e2e_client.py:/e2e_client.py:ro" \
  "$ROS_IMAGE" bash -c 'source /opt/ros/jazzy/setup.bash && python3 /e2e_client.py' \
  2>&1 | tee "$CLIENT_LOG"
rc=${PIPESTATUS[0]}
set -e

echo "---- sim log (errors/tail): $SIM_LOG ----"
grep -iE "error|traceback|exception" "$SIM_LOG" | grep -viE "datastore|material_cache|ogn_generated" | tail -20 || true
tail -15 "$SIM_LOG"
exit "$rc"
