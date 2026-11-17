#!/usr/bin/env bash
# run_hud.sh -- start the presenter HUD (spot-hud image) detached as
# container "spot-hud", listening on the host loopback 127.0.0.1:8080.
#
# Runs ON the EC2 sim host (deployed by scripts/deploy-to-host.sh). Reach it
# from the Mac with scripts/connect-hud.sh (SSM port forward; no inbound ports).
#
#   bash run_hud.sh            # (re)start; builds spot-hud:latest if missing
#   bash run_hud.sh --build    # rebuild the image first
#   bash run_hud.sh --stop     # docker rm -f spot-hud
#   docker logs -f spot-hud    # server log
#
# --network=host for DDS discovery with the sim, isolated to this host's
# loopback like every container (host/sim_lib.sh spot_dds_args, T25); the
# server binds 127.0.0.1 only (hud/Dockerfile CMD), so nothing listens on an
# external interface.
# agent/ is mounted read-only for agent/site_defaults.json (the fallback when
# the latched payload/site topic is not seen within 10 s). The container runs
# as uid 10002 (SPOT_HUD_UID) with no capabilities, no-new-privileges and a
# read-only root filesystem (host/sim_lib.sh spot_container_args, T28).
#
# It warns (does not fail) when no ready sim is running: `docker logs
# spot-sim` of the current container has no FACADE-READY line yet, so the
# HUD shows the agent/site_defaults.json site until payload/site arrives.
#
# Env: SPOT_DEMO_DIR (default /opt/spot-demo), ROS_DOMAIN_ID (default 42),
#      SPOT_HUD_IMAGE (default spot-hud:latest)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$HERE/sim_lib.sh"
DEMO_DIR="${SPOT_DEMO_DIR:-/opt/spot-demo}"
IMAGE="${SPOT_HUD_IMAGE:-spot-hud:latest}"
NAME=spot-hud

BUILD=0
case "${1:-}" in
  --stop) docker rm -f "$NAME" >/dev/null 2>&1 || true; echo "run_hud: $NAME stopped"; exit 0 ;;
  --build) BUILD=1 ;;
  "") ;;
  *) echo "usage: run_hud.sh [--build|--stop]" >&2; exit 2 ;;
esac

if [ "$BUILD" = 1 ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "run_hud: building $IMAGE from $DEMO_DIR/hud/Dockerfile" >&2
  docker build -f "$DEMO_DIR/hud/Dockerfile" -t "$IMAGE" "$DEMO_DIR" >&2
fi

spot_container_args hud || exit 2
docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" --network=host \
  "${SPOT_ARGS[@]}" \
  -v "$DEMO_DIR/agent:/workspace/agent:ro" \
  "$IMAGE" >/dev/null

# Wait for uvicorn to answer (rclpy + FastAPI import takes a few seconds).
for _ in $(seq 1 30); do
  if curl -fs -o /dev/null http://127.0.0.1:8080/api/state; then
    echo "run_hud: $NAME up on http://127.0.0.1:8080 (scripts/connect-hud.sh from the Mac)"
    if ready="$(spot_sim_ready_line)"; then
      echo "run_hud: sim ready (${ready#FACADE-READY })"
    else
      echo "run_hud: WARN: no ready simulator (bash run_sim.sh --status); the HUD shows the" \
           "default site until the sim publishes payload/site" >&2
    fi
    exit 0
  fi
  if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "run_hud: $NAME exited during startup" >&2; exit 1
  fi
  sleep 1
done
echo "run_hud: $NAME did not answer on :8080 within 30 s (docker logs $NAME)" >&2
docker logs --tail 30 "$NAME" >&2 || true
exit 1
