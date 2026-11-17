#!/usr/bin/env bash
# idle_stop.sh -- power the sim host off once nobody has used it for a while (T39).
#
# Optional, opt-in. Runs ON the EC2 sim host, as root (or through sudo). The host
# bills about 1.32 USD an hour while it runs, and forgetting to stop it is the main
# cost risk. apply installs a systemd timer that runs `check` every 5 minutes; check
# powers the host off once, for the whole idle time (default 60 minutes), there has
# been none of:
#   * a Session Manager session: an ssm-session-worker process (a shell or a port
#     forward, which is how DCV and the HUD are reached), or a Run Command in
#     progress (ssm-document-worker: deploy-to-host.sh, ssmrun.sh);
#   * a running demo container: spot-sim, spot-agent, spot-mission, spot-hud,
#     spot-rec (`docker ps`);
#   * an agent or mission process (spot_agent.py, run_mission.py), in a container
#     or not;
#   * an open mission: a mission log (*.jsonl in the logs dir) written within the
#     idle time whose last record has an open permit (PRE_ENTRY_TESTING,
#     ENTRY_AUTHORIZED, ENTRY_IN_PROGRESS), or whose last mission_start has no
#     summary record after it.
# A check that cannot run (docker or pgrep missing or failing) counts as activity:
# it never powers off when it cannot tell. The idle alarm in
# infra/cost-guardrails.yaml is the backstop for that, and for a host where this
# was never applied.
#
#   sudo bash idle_stop.sh apply [--minutes N] [--dry-run]   # install and start the timer
#   sudo bash idle_stop.sh remove [--dry-run]                 # delete exactly what apply added
#   sudo bash idle_stop.sh status                             # exit 0 if applied, 1 if not
#   sudo bash idle_stop.sh check [--dry-run]                  # what the timer runs; --dry-run
#                                                             # says what it would do
#
# Powering off is `shutdown -P now`, an instance-initiated shutdown. An EBS-backed
# instance then STOPS only while its InstanceInitiatedShutdownBehavior attribute is
# `stop`, the EC2 default; with `terminate` this would terminate the host. Check it
# once from the Mac (docs/RUNBOOK.md, "Once per host"):
#   aws ec2 describe-instance-attribute --instance-id <id> --attribute instanceInitiatedShutdownBehavior
# (`halt` would not stop the instance; poweroff does.)
#
# What apply adds (and remove deletes; it prints every change):
#   /usr/local/sbin/spot-idle-stop       a root-owned copy of this script, so the
#                                        timer never runs a file in the deployed
#                                        tree that ubuntu can edit
#   /etc/systemd/system/spot-idle-stop.service   oneshot: spot-idle-stop check,
#                                        with the idle minutes and paths below
#   /etc/systemd/system/spot-idle-stop.timer     5 minutes after boot, then every 5
#   and `systemctl enable --now spot-idle-stop.timer`. A deploy does not update the
#   copy: run apply again after one that changed this script (status says so).
# State: /run/spot-idle-stop/last-active (the last time anything was active). /run
# is emptied at every boot, so the idle clock starts again after each start. Each
# check prints one line to the journal (journalctl -u spot-idle-stop); a power-off
# also appends its reason to /var/log/spot-idle-stop.log, which remove keeps.
#
# Env (tests and unusual layouts): SPOT_DEMO_DIR (default /opt/spot-demo),
#   SPOT_LOG_DIR (default $SPOT_DEMO_DIR/logs), SPOT_IDLE_MINUTES (check only; apply
#   writes it into the unit), SPOT_IDLE_UNIT_DIR (default /etc/systemd/system),
#   SPOT_IDLE_BIN (default /usr/local/sbin/spot-idle-stop), SPOT_IDLE_STATE_DIR
#   (default /run/spot-idle-stop), SPOT_IDLE_LOG (default /var/log/spot-idle-stop.log)
set -euo pipefail

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
DEMO_DIR="${SPOT_DEMO_DIR:-/opt/spot-demo}"
LOG_DIR="${SPOT_LOG_DIR:-$DEMO_DIR/logs}"
UNIT_DIR="${SPOT_IDLE_UNIT_DIR:-/etc/systemd/system}"
BIN="${SPOT_IDLE_BIN:-/usr/local/sbin/spot-idle-stop}"
STATE_DIR="${SPOT_IDLE_STATE_DIR:-/run/spot-idle-stop}"
IDLE_LOG="${SPOT_IDLE_LOG:-/var/log/spot-idle-stop.log}"
UNIT=spot-idle-stop
CONTAINERS="spot-sim spot-agent spot-mission spot-hud spot-rec"
MIN_MINUTES=15
# Seconds a check may spend reading one mission log. The logs directory is shared
# with the containers, which could swap a file for a FIFO (or a huge file) after
# `find` lists it; a read that times out is "cannot tell", never "idle".
READ_TIMEOUT="${SPOT_IDLE_READ_TIMEOUT:-10}"
# Bytes read from one log at most (16 MiB): a larger file, or a FIFO that keeps
# feeding data, is "cannot tell" instead of held in root's memory.
READ_MAX_BYTES="${SPOT_IDLE_READ_MAX_BYTES:-16777216}"
MAX_MINUTES=1440
SUDO=""; [ "$(id -u)" = 0 ] || SUDO=sudo

usage() {
  echo "usage: idle_stop.sh apply [--minutes N] [--dry-run] | remove [--dry-run] | status | check [--dry-run]" >&2
  exit 2
}
log() { echo "idle_stop: $*" >&2; }

ACTION="${1:-}"
[ $# -gt 0 ] && shift
DRY=0
MINUTES=60
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1; shift ;;
    --minutes) [ "$ACTION" = apply ] && [ $# -ge 2 ] || usage; MINUTES="$2"; shift 2 ;;
    *) usage ;;
  esac
done
case "$ACTION" in
  apply|remove|check) ;;
  status) [ "$DRY" = 0 ] || usage ;;
  *) usage ;;
esac
[ "$ACTION" = check ] && MINUTES="${SPOT_IDLE_MINUTES:-60}"
if ! [[ "$MINUTES" =~ ^[0-9]{1,4}$ ]] || [ "$MINUTES" -lt "$MIN_MINUTES" ] || [ "$MINUTES" -gt "$MAX_MINUTES" ]; then
  log "FATAL: idle minutes '$MINUTES' must be a whole number from $MIN_MINUTES to $MAX_MINUTES"; exit 2
fi
# Paths go into systemd unit lines: plain absolute paths only.
for path in "$LOG_DIR" "$UNIT_DIR" "$BIN" "$STATE_DIR" "$IDLE_LOG"; do
  if ! [[ "$path" =~ ^/[A-Za-z0-9._/-]+$ ]] || [[ "$path" == *..* ]]; then
    log "FATAL: '$path' is not a plain absolute path"; exit 2
  fi
done

# --- the activity checks ------------------------------------------------------
# Each prints why the host is in use and returns 0, returns 1 when it is not, and
# returns 2 when it cannot tell (which the caller counts as in use).

# pgrep -f PATTERN: rc 0 match, 1 none; anything else is an error.
process_matching() {
  local what="$1" pattern="$2" pids rc=0
  command -v pgrep > /dev/null 2>&1 || { echo "cannot tell: pgrep not found"; return 2; }
  pids="$(pgrep -f "$pattern" 2> /dev/null)" || rc=$?
  case "$rc" in
    0) echo "$what (pid $(echo "$pids" | tr '\n' ' ' | sed 's/ *$//'))"; return 0 ;;
    1) return 1 ;;
    *) echo "cannot tell: pgrep failed (rc $rc)"; return 2 ;;
  esac
}

demo_container() {
  local names name
  command -v docker > /dev/null 2>&1 || { echo "cannot tell: docker not found"; return 2; }
  names="$(docker ps --format '{{.Names}}' 2> /dev/null)" || { echo "cannot tell: docker ps failed"; return 2; }
  for name in $CONTAINERS; do
    if printf '%s\n' "$names" | grep -qx "$name"; then echo "container $name is running"; return 0; fi
  done
  return 1
}

# The top-level "permit" of a mission.py record follows "permit_id"; nested data
# carries the permit fields as an object, never as one of these strings.
OPEN_PERMIT_RE='"permit_id": ("[^"]*"|null), "permit": "(PRE_ENTRY_TESTING|ENTRY_AUTHORIZED|ENTRY_IN_PROGRESS)"'
open_mission() {
  local f content last start end
  [ -d "$LOG_DIR" ] || return 1
  command -v timeout > /dev/null || { echo "cannot tell: timeout(1) is missing"; return 2; }
  while IFS= read -r -d '' f; do
    # One bounded read per file (T28, T39): a FIFO or a file that never ends
    # cannot hold the check, which runs as root, past READ_TIMEOUT.
    # A file swapped for a link after find listed it is not followed.
    if [ -L "$f" ] || [ ! -f "$f" ]; then echo "cannot tell: $f is no longer a regular file"; return 2; fi
    content="$(timeout "$READ_TIMEOUT" head -c "$((READ_MAX_BYTES + 1))" -- "$f" 2> /dev/null)" \
      || { echo "cannot tell: cannot read $f within ${READ_TIMEOUT}s"; return 2; }
    if [ "$(printf '%s' "$content" | wc -c)" -gt "$READ_MAX_BYTES" ]; then
      echo "cannot tell: $f is larger than $READ_MAX_BYTES bytes"; return 2
    fi
    last="$(printf '%s\n' "$content" | tail -n 1)"
    if printf '%s\n' "$last" | grep -Eq "$OPEN_PERMIT_RE"; then
      echo "open mission in $f (its last record has an open permit)"; return 0
    fi
    start="$(printf '%s\n' "$content" | grep -n '"kind": "mission_start"' | tail -n 1 | cut -d: -f1)"
    end="$(printf '%s\n' "$content" | grep -n '"kind": "summary"' | tail -n 1 | cut -d: -f1)"
    if [ -n "$start" ] && [ "${end:-0}" -lt "$start" ]; then
      echo "open mission in $f (mission_start with no summary after it)"; return 0
    fi
  done < <(find "$LOG_DIR" -maxdepth 1 -type f -name '*.jsonl' -mmin "-$MINUTES" -print0 2> /dev/null)
  return 1
}

# Print every reason the host is in use, one per line; rc 0 if any (or if a
# check cannot tell), 1 if it is idle.
activity() {
  local busy=1 out rc check
  for check in session run_command container agent mission; do
    rc=0
    case "$check" in
      session) out="$(process_matching "a Session Manager session" 'ssm-session-worker')" || rc=$? ;;
      run_command) out="$(process_matching "a Run Command in progress" 'ssm-document-worker')" || rc=$? ;;
      container) out="$(demo_container)" || rc=$? ;;
      agent) out="$(process_matching "an agent or mission process" 'spot_agent\.py|run_mission\.py')" || rc=$? ;;
      mission) out="$(open_mission)" || rc=$? ;;
    esac
    if [ "$rc" != 1 ]; then echo "$out"; busy=0; fi
  done
  return "$busy"
}

# --- check: what the timer runs -------------------------------------------------
now_s() { date +%s; }
now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }

mark_active() {
  [ "$DRY" = 1 ] && return 0
  mkdir -p "$STATE_DIR" && chmod 0700 "$STATE_DIR"
  printf '%s %s\n' "$(now_s)" "$(now_iso)" > "$STATE_DIR/last-active"
}

do_check() {
  local reasons now last="" last_iso="" idle_s line
  if reasons="$(activity)"; then
    mark_active
    log "in use, not stopping: $(printf '%s' "$reasons" | tr '\n' ';' | sed 's/;/; /g')"
    return 0
  fi
  now="$(now_s)"
  read -r last last_iso 2> /dev/null < "$STATE_DIR/last-active" || last=""
  if ! [[ "${last:-}" =~ ^[0-9]{9,11}$ ]] || [ "$last" -gt "$now" ]; then
    mark_active
    log "idle; the idle clock starts now (no earlier activity recorded since boot)"
    return 0
  fi
  idle_s=$((now - last))
  if [ "$idle_s" -lt $((MINUTES * 60)) ]; then
    log "idle for $((idle_s / 60)) of $MINUTES minutes (last activity $last_iso)"
    return 0
  fi
  line="$(now_iso) powering off: idle for $((idle_s / 60)) minutes, at least $MINUTES (no Session Manager session or Run Command, no demo container, no agent or mission process and no open mission since $last_iso)"
  if [ "$DRY" = 1 ]; then
    echo "would power off: ${line#* powering off: }"
    return 0
  fi
  log "$line"
  if ! { mkdir -p "$(dirname "$IDLE_LOG")" && echo "$line" >> "$IDLE_LOG"; } 2> /dev/null; then
    log "WARNING: could not append to $IDLE_LOG; the journal has the reason"
  fi
  shutdown -P now "spot idle stop: idle for at least $MINUTES minutes"
}

# --- apply / remove / status -------------------------------------------------------
service_unit() {
  cat <<EOF
# Installed by host/idle_stop.sh apply (threat model T39). Remove with: idle_stop.sh remove
[Unit]
Description=Spot demo idle stop: power off after $MINUTES idle minutes
After=docker.service

[Service]
Type=oneshot
# A check that cannot finish is killed and retried at the next tick (T39).
TimeoutStartSec=120
Environment=SPOT_IDLE_MINUTES=$MINUTES
Environment=SPOT_LOG_DIR=$LOG_DIR
Environment=SPOT_IDLE_STATE_DIR=$STATE_DIR
Environment=SPOT_IDLE_LOG=$IDLE_LOG
ExecStart=/bin/bash $BIN check
EOF
}

timer_unit() {
  cat <<EOF
# Installed by host/idle_stop.sh apply (threat model T39). Remove with: idle_stop.sh remove
[Unit]
Description=Spot demo idle check, every 5 minutes

[Timer]
OnBootSec=5min
OnUnitActiveSec=5min
AccuracySec=1min

[Install]
WantedBy=timers.target
EOF
}

run() {
  if [ "$DRY" = 1 ]; then echo "would run: $*"; return 0; fi
  log "running: $*"
  $SUDO "$@" < /dev/null
}

# install_file SOURCE DEST MODE: copy unless DEST already has the same content.
install_file() {
  local src="$1" dest="$2" mode="$3"
  if [ -f "$dest" ] && cmp -s "$src" "$dest"; then
    log "$dest unchanged"
    return 0
  fi
  if [ "$DRY" = 1 ]; then
    echo "would write $dest (mode $mode, owner root):"
    sed 's/^/  /' "$src"
    return 0
  fi
  log "writing $dest (mode $mode, owner root)"
  $SUDO mkdir -p "$(dirname "$dest")"
  $SUDO cp "$src" "$dest"
  $SUDO chmod "$mode" "$dest"
  $SUDO chown root:root "$dest"
}

do_apply() {
  local tmp
  tmp="$(mktemp -d)"
  # shellcheck disable=SC2064  # expand now: the trap must remove this run's directory
  trap "rm -rf '$tmp'" EXIT
  service_unit > "$tmp/service"
  timer_unit > "$tmp/timer"
  if [ "$SELF" = "$BIN" ]; then log "$BIN is this script"; else install_file "$SELF" "$BIN" 0755; fi
  install_file "$tmp/service" "$UNIT_DIR/$UNIT.service" 0644
  install_file "$tmp/timer" "$UNIT_DIR/$UNIT.timer" 0644
  run systemctl daemon-reload
  run systemctl enable --now "$UNIT.timer"
  [ -d "$LOG_DIR" ] || log "WARNING: $LOG_DIR does not exist; open missions are looked for there"
  log "applied: the host powers itself off after $MINUTES minutes with no session, demo container," \
      "agent, mission or open mission. This STOPS the instance only if its" \
      "InstanceInitiatedShutdownBehavior is 'stop' (the default): check it once (docs/RUNBOOK.md)."
}

do_remove() {
  local path found=0
  for path in "$UNIT_DIR/$UNIT.timer" "$UNIT_DIR/$UNIT.service" "$BIN" "$STATE_DIR"; do
    [ -e "$path" ] && found=1
  done
  if [ "$found" = 0 ]; then log "nothing to remove"; return 0; fi
  if [ -f "$UNIT_DIR/$UNIT.timer" ]; then run systemctl disable --now "$UNIT.timer"; fi
  for path in "$UNIT_DIR/$UNIT.timer" "$UNIT_DIR/$UNIT.service" "$BIN"; do
    [ -e "$path" ] || continue
    if [ "$path" = "$SELF" ]; then log "not deleting $path: it is the script running now"; continue; fi
    run rm -f "$path"
  done
  run systemctl daemon-reload
  if [ -d "$STATE_DIR" ]; then run rm -rf "$STATE_DIR"; fi
  log "removed: the host no longer powers itself off when idle ($IDLE_LOG is kept)"
}

do_status() {
  local rc=0 reasons last="" last_iso=""
  if [ -f "$UNIT_DIR/$UNIT.service" ] && [ -f "$UNIT_DIR/$UNIT.timer" ] && [ -f "$BIN" ] \
     && $SUDO systemctl is-enabled --quiet "$UNIT.timer" < /dev/null \
     && $SUDO systemctl is-active --quiet "$UNIT.timer" < /dev/null; then
    MINUTES="$(sed -n 's/^Environment=SPOT_IDLE_MINUTES=\([0-9]*\)$/\1/p' "$UNIT_DIR/$UNIT.service")"
    echo "idle_stop: APPLIED (powers the host off after ${MINUTES:-?} idle minutes)"
  else
    echo "idle_stop: NOT APPLIED (the host does not power itself off when idle)"
    rc=1
  fi
  if [ -f "$BIN" ] && [ "$SELF" != "$BIN" ] && ! cmp -s "$SELF" "$BIN"; then
    echo "  the installed $BIN differs from $SELF: run apply again"
  fi
  if read -r last last_iso 2> /dev/null < "$STATE_DIR/last-active"; then
    echo "  last activity: ${last_iso:-$last}"
  fi
  if reasons="$(activity)"; then
    printf '%s\n' "$reasons" | sed 's/^/  in use: /'
  else
    echo "  in use: nothing (idle)"
  fi
  return "$rc"
}

case "$ACTION" in
  apply) do_apply ;;
  remove) do_remove ;;
  status) do_status ;;
  check) do_check ;;
esac
