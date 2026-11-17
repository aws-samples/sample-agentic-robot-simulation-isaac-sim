#!/usr/bin/env bash
# smoke_test.sh -- headless end-to-end smoke test of the Spot sim container.
#
# Runs ON the EC2 sim host. Launches run_sim.sh with
# --headless true --smoke --no-cameras under a 20-minute timeout, tees all
# output to a NEW per-run log $SPOT_DEMO_DIR/logs/smoke-<run_id>.log
# (symlink logs/smoke-latest.log), and passes if and only if THIS launch
# printed its exact success marker "SMOKE-OK ... run_id=<run_id>"
# (isaac_app.py guarantees no failure path ever prints a string containing
# "SMOKE-OK"). On failure it prints the last 60 log lines and exits non-zero.
# (The old fixed /tmp/spot-smoke.log could be a root-owned leftover that tee
# could not reopen -- fs.protected_regular=2 -- and the grep then passed on
# a STALE marker.)
#
# NOTE: isaac_app.py's --headless flag TAKES A VALUE (true|false); a bare
# --headless makes argparse exit(2) before the sim even starts.
#
# Usage: ACCEPT_EULA=Y bash smoke_test.sh [--force]   (Y once you accept the NVIDIA licence)
# It replaces any running sim, so it refuses while an agent or mission
# container runs (their mission would lose the sim); --force overrides (T41).
# Env:   SPOT_SMOKE_TIMEOUT (seconds, default 1200)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$SCRIPT_DIR/sim_lib.sh"
TIMEOUT_SECS="${SPOT_SMOKE_TIMEOUT:-1200}"
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
  *) echo "usage: smoke_test.sh [--force]" >&2; exit 2 ;;
esac
spot_guard_sim_stop "$FORCE" "replace the running sim with a smoke test" || exit 1
spot_test_domain || exit 2   # its own DDS domain (T41)
RUN_ID="$(spot_run_id)"
LOG_FILE="$(spot_new_log smoke "$RUN_ID")" || exit 1
echo "smoke: run_id=$RUN_ID log=$LOG_FILE"
# Success = the explicit SMOKE-OK marker printed only by isaac_app.py's
# success path (failure paths print "SMOKE FAILED: ..." and exit 1), for
# THIS run id.
PASS_RE="^SMOKE-OK .*run_id=${RUN_ID}( |\$)"

rc=0
SPOT_RUN_ID="$RUN_ID" timeout --kill-after=60 "$TIMEOUT_SECS" \
  bash "$SCRIPT_DIR/run_sim.sh" --force --headless true --smoke --no-cameras 2>&1 \
  | tee "$LOG_FILE" || rc=$?

# On timeout, `timeout` kills the docker *client*; make sure the container
# itself is gone too, but only if it is this run's (T41).
spot_rm_own_sim "$RUN_ID"

if grep -qE -- "$PASS_RE" "$LOG_FILE"; then
  echo "SMOKE TEST PASSED: found success marker (runner exit code $rc, log: $LOG_FILE)"
  exit 0
fi

echo "SMOKE TEST FAILED: no success marker ($PASS_RE) in $LOG_FILE (runner exit code $rc). Last 60 lines:" >&2
tail -n 60 "$LOG_FILE" >&2
exit 1
