#!/usr/bin/env bash
# rehearse_rep.sh -- one presenter-cheat-sheet rep of the agent storyline.
#
# What it is:
#   The shell-A half of a leadership rehearsal (docs/RUNBOOK.md, presenter
#   cheat sheet): it resets Spot, starts the agent REPL as ubuntu under a
#   real pty (script -qfc), types the cheat-sheet prompts one at a time,
#   waiting for each turn to finish (the next "operator> " prompt), and
#   records turn timings, odometry and (optionally) screenshots. The staged
#   H2S spike is NOT fired here: like the presenter's shell B, a second,
#   concurrent SSM command runs host/inject_gas.sh as ubuntu once the log
#   shows interior waypoint 1 (see docs/evidence/README.md, v5 rehearsal).
#
# Run ON the sim host as root (SSM), with the sim and the HUD already up and
# no agent session running (it refuses otherwise; a third arg --force overrides):
#   bash /opt/spot-demo/tools/rehearse_rep.sh incident LABEL   # (a)(b1)(b2)(c) + spike from shell B
#   bash /opt/spot-demo/tools/rehearse_rep.sh clean LABEL      # (a)(b1)(b2)(c) + exit, no spike
# Writes /opt/spot-demo/logs/LABEL.jsonl (mission audit, --log-path),
#   LABEL-transcript.txt (pty), LABEL-timing.txt (wall time of each prompt
#   sent / prompt back), odom-LABEL.jsonl (tools/odom_rec.py),
#   LABEL-screens.txt + screens/*-LABEL-*.png when REHEARSE_SHOTS=1 (tools/screen_watch.py).
# Env: REHEARSE_SHOTS=1 (default 0) screenshots (not SNAP: the snap-packaged SSM agent sets SNAP); PAUSE_S presenter pause between turns (default 3).
set -uo pipefail
# noclobber: every file this script creates in logs/ is opened with O_EXCL, so a
# name a container planted there first (a symlink, for example) is refused, not
# followed; each file is opened once and then written through its descriptor.
# The logs directory is sticky (host/sim_lib.sh spot_shared_dir_gid), so a file
# cannot be swapped once it exists. This script runs as root.
set -C

MODE="${1:?usage: rehearse_rep.sh incident|clean LABEL [--force]}"
LABEL="${2:?usage: rehearse_rep.sh incident|clean LABEL [--force]}"
case "${3:-}" in ""|--force) ;; *) echo "usage: rehearse_rep.sh incident|clean LABEL [--force]" >&2; exit 2 ;; esac
case "$MODE" in incident|clean) ;; *) echo "mode must be incident or clean" >&2; exit 2 ;; esac
[[ "$LABEL" =~ ^[A-Za-z0-9._-]{1,48}$ ]] || { echo "bad LABEL" >&2; exit 2; }
D=/opt/spot-demo; L=$D/logs; H=$D/host
SPOT_DEMO_DIR=$D
# shellcheck source=host/sim_lib.sh
source "$H/sim_lib.sh"
# spot-rec: a one-shot (own non-root uid, no capabilities, read-only root,
# DDS isolation; T25, T28) that sees only its own script, read-only, and
# logs/, which it writes through the dir's group. It never mounts the code:
# a root recorder with all of $D read-write could rewrite host/ and agent/.
spot_container_args oneshot || exit 2
LOG_GID="$(spot_shared_dir_gid "$L")" || exit 1
spot_pinned_image SPOT_ROS_IMAGE || exit 2
LOG=$L/$LABEL.jsonl; TR=$L/$LABEL-transcript.txt; TIM=$L/$LABEL-timing.txt; SCR=$L/$LABEL-screens.txt
PAUSE_S="${PAUSE_S:-3}"

# It replaces any agent session; never one that is driving Spot (T41). With
# --force, docker stop (SIGTERM, 900 s) lets the agent bring Spot out first;
# never docker rm -f, which ends its SafetyGuard at once (T16).
FORCE=0; [ "${3:-}" = --force ] && FORCE=1
spot_guard_sim_stop "$FORCE" "start a rehearsal (it removes the agent container)" || exit 1
docker stop -t 900 spot-agent >/dev/null 2>&1; docker stop -t 5 spot-rec >/dev/null 2>&1
docker rm spot-agent spot-rec >/dev/null 2>&1

# Every file this root shell writes in logs/ is created now, while no container
# that writes logs/ is running, after refusing any name that already exists as
# anything (noclobber alone would still open an existing device or FIFO a link
# points to). Each is then written only through its descriptor.
for f in "$LOG" "$TR" "$TIM" "$SCR" "$L/odom-$LABEL.jsonl"; do
  if [ -e "$f" ] || [ -L "$f" ]; then echo "rehearse_rep: $LABEL already used ($f exists)" >&2; exit 1; fi
done
exec 4> "$TIM" || { echo "rehearse_rep: cannot create $TIM" >&2; exit 1; }
exec 5> "$TR" || { echo "rehearse_rep: cannot create $TR" >&2; exit 1; }
if [ "${REHEARSE_SHOTS:-0}" = 1 ]; then
  exec 6> "$SCR" || { echo "rehearse_rep: cannot create $SCR" >&2; exit 1; }
fi

B() { sudo -u ubuntu -i bash -c "cd $H && $*"; }       # the presenter's shell B
stamp() { echo "$(date +%s.%3N) $*" >&4; }
prompts() { grep -o 'operator> ' "$TR" 2>/dev/null | wc -l; }
wait_prompts() {  # N timeout_s -> 0 once N prompts are on screen
  local n=$1 end=$(( $(date +%s) + $2 ))
  while [ "$(prompts)" -lt "$n" ]; do
    [ "$(date +%s)" -ge "$end" ] && return 1
    sleep 0.2
  done
}
PROMPT_N=1
turn() {  # name timeout text
  local name=$1 timeout=$2 text=$3
  stamp "SEND $name $text"
  printf '%s\n' "$text" >&3
  PROMPT_N=$((PROMPT_N + 1))
  if wait_prompts "$PROMPT_N" "$timeout"; then stamp "DONE $name"; else stamp "TIMEOUT $name"; fi
  sleep "$PAUSE_S"
}

B "bash inject_gas.sh clear" | tail -1
B "bash reset_spot.sh --reset-ventilation" | tail -1
stamp "RESET done"
sleep 2
docker run -d --name spot-rec --network=host "${SPOT_ARGS[@]}" --group-add "$LOG_GID" \
  -v "$D/tools/odom_rec.py:/demo/tools/odom_rec.py:ro" -v "$L:/demo/logs" \
  "$SPOT_ROS_IMAGE" bash -c "source /opt/ros/jazzy/setup.bash && \
  python3 /demo/tools/odom_rec.py /demo/logs/odom-$LABEL.jsonl 900" >/dev/null
if [ "${REHEARSE_SHOTS:-0}" = 1 ]; then
  sudo -u ubuntu bash $D/tools/snap_screens.sh "$LABEL-ready" | tail -3
  # shellcheck disable=SC2024  # the log is opened by this root shell on purpose
  sudo -u ubuntu setsid nohup python3 $D/tools/screen_watch.py "$LOG" "$LABEL" 900 \
    >&6 2>&1 < /dev/null &   # $SCR, opened above
fi

FIFO=$(mktemp -u /tmp/rehearse-fifo.XXXXXX); mkfifo -m 600 "$FIFO"
exec 3<>"$FIFO"
# shellcheck disable=SC2024  # the FIFO is opened by this root shell on purpose
# --rehearsal-auto-confirm answers the console's y/N confirmations (T5): an
# automated rehearsal has no operator at the keyboard. The agent accepts it only
# against the identified simulator and logs every auto-confirmation.
# --unattributed-run (T38): this root shell's sudo makes the invoking login
# root, a shared account, so the agent would refuse to start; the flag starts
# it and the session record says the run names no operator.
# The transcript is script's stdout, the descriptor opened above; script itself
# writes no file by name (its typescript goes to /dev/null).
sudo -u ubuntu script -qfc "cd $H && bash run_agent.sh --rehearsal-auto-confirm --unattributed-run --log-path /workspace/logs/$LABEL.jsonl" \
  /dev/null < "$FIFO" >&5 2>/dev/null &
SPID=$!
stamp "AGENT start"
if wait_prompts 1 90; then stamp "AGENT ready"; else stamp "TIMEOUT agent-ready"; fi

turn a 150 "Prepare permit CSE-0427: the attendant is at the entry, isolation and lockout-tagout are in place, rescue plan: on-site standby team. Then run the pre-entry atmospheric test at the tank."
turn b1 150 "Start ventilation, wait 30 seconds, then re-test. Don't wait for the purge."
turn b2 150 "Wait for the purge target, then re-test."
turn c 180 "Proceed with the confined space entry."
if [ "$MODE" = incident ]; then
  end=$(( $(date +%s) + 60 ))   # the turn may end before the retreat does
  until grep -q '"kind": *"retreat_\(complete\|incomplete\)"' "$LOG" 2>/dev/null || [ "$(date +%s)" -ge "$end" ]; do sleep 0.5; done
  stamp "RETREAT seen"
else
  turn exit 180 "Walk back out to the entry point."
fi
stamp "SEND quit"
printf 'quit\n' >&3
for _ in $(seq 1 120); do kill -0 "$SPID" 2>/dev/null || break; sleep 0.5; done
# docker stop, not rm -f: SIGTERM lets the agent bring Spot out first (T16).
kill -0 "$SPID" 2>/dev/null && { stamp "STOP agent"; docker stop -t 900 spot-agent >/dev/null 2>&1; }
exec 3>&-; rm -f "$FIFO"
stamp "AGENT exited"
docker stop -t 5 spot-rec >/dev/null 2>&1; docker rm -f spot-rec >/dev/null 2>&1
B "bash inject_gas.sh clear" | tail -1                 # beat 9: clean up
B "bash reset_spot.sh --reset-ventilation" | tail -1
stamp "CLEANUP done"
cat "$TIM"
grep -c . "$LOG"
