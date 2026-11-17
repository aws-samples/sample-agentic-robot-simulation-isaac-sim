#!/usr/bin/env bash
# run_agent.sh -- run the operator agent (or the scripted mission) in the
# spot-agent image, next to the Isaac Sim container.
#
# Runs ON the EC2 sim host (deployed by scripts/deploy-to-host.sh).
#
#   bash run_agent.sh                       # interactive REPL (needs a TTY)
#   bash run_agent.sh --once "Check the robot status ..."
#   bash run_agent.sh --dry-run --once "..."   # FakeRobot, still calls Bedrock
#   bash run_agent.sh --mission [run_mission.py args]
#        e.g. --mission --log /workspace/logs/mission-clean.jsonl
#             --mission --inject-h2s-at-waypoint 2
#   bash run_agent.sh --build [...]         # (re)build spot-agent:latest first
#   SPOT_DEMO_PRESENTER=1 bash run_agent.sh # presenter session: gas-spike tool
#
# Everything after the run_agent.sh flags passes straight through to
# agent/spot_agent.py (or agent/run_mission.py with --mission). The exit code
# is the container's (run_mission: 0 COMPLETED, 2 ABORTED as designed, 1 fail).
#
# Container settings and why:
#   --network=host   DDS discovery with the sim (also --network=host), and the
#                    instance-role credentials via IMDSv2 stay one hop away
#                    (the instance's hop limit is 1). No keys are passed in.
#   DDS isolation    ROS_DOMAIN_ID ${ROS_DOMAIN_ID:-42}, a
#                    loopback-only Fast DDS profile and UDPv4 (shared memory
#                    silently drops samples between containers): host/sim_lib.sh
#                    spot_dds_args, the same for every container (T25).
#   hardening        uid 10001 (SPOT_AGENT_UID), --cap-drop=ALL,
#                    no-new-privileges, a read-only root filesystem with a
#                    small /tmp: host/sim_lib.sh spot_container_args (T28).
#   -t only when stdin is a terminal: under SSM Run Command there is none and
#                    `docker run -t` would fail.
#   logs             /opt/spot-demo/logs is mounted at /workspace/logs (the
#                    agent's default audit log dir is ./logs under /workspace),
#                    the only writable mount; the container joins that dir's
#                    group (--group-add) and the dir is made group-writable.
#                    Name logs per run (e.g. demo-$(date -u +%Y%m%dT%H%M%S).jsonl).
#   sim check        before starting (unless --dry-run, which uses FakeRobot)
#                    the running spot-sim container must have printed
#                    FACADE-READY (docker logs of the CURRENT container, not a
#                    log file); otherwise it stops with how to start the sim.
#                    SPOT_SKIP_SIM_CHECK=1 skips only this wrapper check (for
#                    tests): the agent itself still refuses to command any
#                    backend that is not the identified simulator (T9).
#   operator         USER, the login that ran sudo (SUDO_USER, as
#                    SPOT_OPERATOR_LOGIN) and SSM_SESSION_ID or
#                    AWS_SSM_SESSION_ID when set (Session Manager sets none by
#                    default; a shell profile can) go into the container, so
#                    every console confirmation in the mission log names who
#                    answered (T5, T38). They stay in the JSONL: never on
#                    payload/events, never sent to the model.
#   guardrail        SPOT_GUARDRAIL_ID and SPOT_GUARDRAIL_VERSION (the outputs of
#                    infra/bedrock-guardrail.yaml) go into the container: the
#                    guardrail is on by default and the agent refuses to start
#                    with neither them nor --guardrail-id, unless the operator
#                    passes --no-guardrail (T11). Set them once per host, e.g. in
#                    /etc/environment or the shell profile that starts the demo.
#   stop timeout     900 s: `docker stop` sends SIGTERM, the agent brings Spot
#                    out of the permit space first, retrying a failed retreat,
#                    and only then exits; it waits up to 900 s itself (T16, T17).
#
# Env: SPOT_DEMO_DIR (default /opt/spot-demo), ROS_DOMAIN_ID (default 42),
#      AWS_REGION (default us-west-2), SPOT_AGENT_IMAGE (default spot-agent:latest),
#      SPOT_GUARDRAIL_ID and SPOT_GUARDRAIL_VERSION (the Bedrock guardrail)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$HERE/sim_lib.sh"
DEMO_DIR="${SPOT_DEMO_DIR:-/opt/spot-demo}"
IMAGE="${SPOT_AGENT_IMAGE:-spot-agent:latest}"
REGION="${AWS_REGION:-us-west-2}"

BUILD=0
MODE=agent
while [ $# -gt 0 ]; do
  case "$1" in
    --build) BUILD=1; shift ;;
    --mission) MODE=mission; shift ;;
    *) break ;;
  esac
done

if [ "$BUILD" = 1 ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "run_agent: building $IMAGE from $DEMO_DIR/agent/Dockerfile" >&2
  docker build -f "$DEMO_DIR/agent/Dockerfile" -t "$IMAGE" "$DEMO_DIR" >&2
fi

DRY_RUN=0
for a in "$@"; do [ "$a" = --dry-run ] && DRY_RUN=1; done
if [ "$DRY_RUN" = 0 ] && [ "${SPOT_SKIP_SIM_CHECK:-0}" != 1 ]; then
  if ! ready="$(spot_sim_ready_line)"; then
    echo "run_agent: FATAL: the simulator is not ready: no running spot-sim container has printed" \
         "FACADE-READY. Start it with 'bash run_sim.sh --background --headless true' (it returns" \
         "once ready) and check with 'bash run_sim.sh --status'." >&2
    exit 1
  fi
  echo "run_agent: sim ready (${ready#FACADE-READY })" >&2
fi

LOG_DIR="$(spot_log_dir)" || exit 1
LOG_GID="$(spot_shared_dir_gid "$LOG_DIR")" || exit 1
spot_container_args agent || exit 2
TTY_ARGS=(-i)
[ -t 0 ] && [ -t 1 ] && TTY_ARGS=(-it)

if [ "$MODE" = mission ]; then
  NAME=spot-mission
  ENTRY=(--entrypoint /bin/bash)
  CMD=(-c 'source /opt/ros/jazzy/setup.bash && exec /opt/agent-venv/bin/python3 /workspace/agent/run_mission.py "$@"' run_mission "$@")
else
  NAME=spot-agent
  ENTRY=()
  CMD=("$@")
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  # Not `docker rm -f`: a container that still exists is most likely an agent
  # that is keeping Spot's guard alive (it refuses to quit while Spot is inside);
  # SIGKILL would end the guard mid-entry (T16).
  echo "run_agent: a '$NAME' container already exists. If Spot is inside the permit space, use" \
       "its console (/retreat, then quit). Otherwise stop it with: docker stop $NAME" >&2
  exit 1
fi

SSM_ID="${SSM_SESSION_ID:-${AWS_SSM_SESSION_ID:-}}"
# The deployed commit (scripts/deploy-to-host.sh writes it), logged with each
# mission run so a record can be tied to the code that produced it (T30).
DEMO_VERSION="$(sed -n 's/^commit=\([0-9a-f]\{7,40\}\(-dirty\)\{0,1\}\)$/\1/p' "$DEMO_DIR/VERSION" 2>/dev/null | head -1 || true)"
exec docker run --rm "${TTY_ARGS[@]}" --name "$NAME" --network=host --stop-timeout 900 \
  "${SPOT_ARGS[@]}" --group-add "$LOG_GID" \
  -e AWS_REGION="$REGION" -e AWS_DEFAULT_REGION="$REGION" \
  -e USER="${USER:-$(id -un)}" ${SSM_ID:+-e SSM_SESSION_ID="$SSM_ID"} \
  ${SUDO_USER:+-e SPOT_OPERATOR_LOGIN="$SUDO_USER"} \
  -e SPOT_DEMO_VERSION="${DEMO_VERSION:-unknown}" \
  ${SPOT_DEMO_PRESENTER:+-e SPOT_DEMO_PRESENTER="$SPOT_DEMO_PRESENTER"} \
  ${SPOT_GUARDRAIL_ID:+-e SPOT_GUARDRAIL_ID="$SPOT_GUARDRAIL_ID"} \
  ${SPOT_GUARDRAIL_VERSION:+-e SPOT_GUARDRAIL_VERSION="$SPOT_GUARDRAIL_VERSION"} \
  -v "$LOG_DIR:/workspace/logs" \
  ${ENTRY[@]+"${ENTRY[@]}"} "$IMAGE" ${CMD[@]+"${CMD[@]}"}
