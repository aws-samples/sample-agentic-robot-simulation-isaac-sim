#!/usr/bin/env bash
# configure_dcv.sh -- keep Amazon DCV on loopback, one viewer at a time (T1).
#
# Runs ON the EC2 sim host. The operator reaches DCV only through the SSM
# port forward (scripts/connect-dcv.sh), which connects to localhost:8443 on
# the host, so DCV never needs to listen on another address. Stock DCV listens
# on every interface (web on TCP 8443, QUIC on UDP 8443 on Linux) and lets any
# number of viewers share the console session; only the security group keeps
# others out.
#
#   sudo bash configure_dcv.sh apply [--restart]    # set the keys below
#   sudo bash configure_dcv.sh restore [--restart]  # put the original file back
#   bash configure_dcv.sh check     # exit 1 if anything listens on 8443 off loopback
#   bash configure_dcv.sh print     # the dcv.conf apply would write, to stdout
#
# apply records the original /etc/dcv/dcv.conf once (dcv.conf.spot-demo.orig,
# or a .spot-demo.absent marker if there was none) and then edits the file in
# place, the way the admin guide's `crudini --set` does: it sets these keys
# and keeps every other line. The guide documents only /etc/dcv/dcv.conf on
# Linux (no drop-in directory), so there is nothing else to write. Keys and
# sections, from the Amazon DCV Administrator Guide, "Amazon DCV Server
# parameter reference" (docs.aws.amazon.com/dcv/latest/adminguide/
# config-param-ref.html) and "Modifying configuration parameters"
# (.../config-param-ref-modify.html), checked 2026-10-29:
#   [connectivity] web-listen-endpoints   web listener: IPv4 and IPv6 loopback
#   [connectivity] enable-quic-frontend   false (the Linux default is true)
#   [connectivity] quic-listen-endpoints  loopback too, if QUIC is ever re-enabled
#   [session-management] create-session   true: keep the automatic console session
#   [session-management/automatic-console-session] owner   the desktop user
#   [session-management/automatic-console-session] max-concurrent-clients   1
#     (a second viewer is refused: client-eviction-policy stays at its default,
#     reject-new-connection)
# DCV reads them at start: --restart (or `sudo systemctl restart dcvserver`)
# applies them, and drops connected viewers.
#
# check lists listening sockets with `sudo ss -Hltnup` (host/sim_lib.sh
# spot_nonloopback_listeners) and fails on any TCP or UDP 8443 socket that is
# not bound to loopback, whatever owns it. run_sim.sh --gui runs it and refuses
# to start on a failure.
#
# Env: SPOT_DCV_CONF (default /etc/dcv/dcv.conf), SPOT_GUI_USER (default ubuntu)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$HERE/sim_lib.sh"

CONF="${SPOT_DCV_CONF:-/etc/dcv/dcv.conf}"
ORIG="$CONF.spot-demo.orig"
ABSENT="$CONF.spot-demo.absent"
GUI_USER="${SPOT_GUI_USER:-ubuntu}"
DCV_PORT=8443
SUDO=""; [ "$(id -u)" = 0 ] || SUDO=sudo

usage() { echo "usage: configure_dcv.sh apply [--restart] | restore [--restart] | check | print" >&2; exit 2; }
log() { echo "configure_dcv: $*" >&2; }

[[ "$GUI_USER" =~ ^[a-z_][a-z0-9_-]{0,31}$ ]] || { log "FATAL: bad SPOT_GUI_USER '$GUI_USER'"; exit 2; }
ACTION="${1:-}"
RESTART=0
case "${2:-}" in
  "") ;;
  --restart) RESTART=1 ;;
  *) usage ;;
esac
[ $# -le 2 ] || usage
case "$ACTION" in
  apply|restore) ;;
  check|print) [ "$RESTART" = 0 ] || usage ;;
  *) usage ;;
esac

# "section key value" for each managed key (value as dcv.conf spells it).
DCV_SETTINGS="connectivity web-listen-endpoints ['127.0.0.1:$DCV_PORT', '[::1]:$DCV_PORT']
connectivity enable-quic-frontend false
connectivity quic-listen-endpoints ['127.0.0.1:$DCV_PORT', '[::1]:$DCV_PORT']
session-management create-session true
session-management/automatic-console-session owner \"$GUI_USER\"
session-management/automatic-console-session max-concurrent-clients 1"

# The file with the managed keys set (stdin: the current file, possibly empty).
render() {
  DCV_SETTINGS="$DCV_SETTINGS" python3 -c '
import os, sys
lines = sys.stdin.read().splitlines()
for entry in os.environ["DCV_SETTINGS"].splitlines():
    section, key, value = entry.split(" ", 2)
    want = "%s=%s" % (key, value)
    header = "[%s]" % section
    starts = [i for i, line in enumerate(lines) if line.strip() == header]
    if not starts:
        if lines and lines[-1].strip():
            lines.append("")
        lines += [header, want]
        continue
    start = starts[0]
    end = next((i for i in range(start + 1, len(lines)) if lines[i].lstrip().startswith("[")), len(lines))
    hits = [i for i in range(start + 1, end)
            if not lines[i].lstrip().startswith("#") and "=" in lines[i]
            and lines[i].split("=", 1)[0].strip() == key]
    if hits:
        lines[hits[0]] = want
        for i in reversed(hits[1:]):
            del lines[i]
    else:
        lines.insert(start + 1, want)
sys.stdout.write("\n".join(lines) + "\n")
'
}

current() { if $SUDO test -f "$CONF"; then $SUDO cat "$CONF"; fi; }

restart_hint() {
  if [ "$RESTART" = 1 ]; then
    log "restarting dcvserver (connected viewers are dropped)"
    $SUDO systemctl restart dcvserver
  else
    log "DCV reads this at start: sudo systemctl restart dcvserver (drops connected viewers), or pass --restart"
  fi
}

case "$ACTION" in
  print)
    current | render ;;
  apply)
    log "setting in $CONF (original kept as $ORIG):"
    while read -r section key value; do log "  [$section] $key=$value"; done <<< "$DCV_SETTINGS"
    if ! $SUDO test -e "$ORIG" && ! $SUDO test -e "$ABSENT"; then
      if $SUDO test -f "$CONF"; then $SUDO cp -p "$CONF" "$ORIG"; else $SUDO touch "$ABSENT"; fi
    fi
    new="$(current | render)"
    if [ "$(current)" = "$new" ]; then
      log "already set; $CONF unchanged"
    else
      printf '%s\n' "$new" | $SUDO tee "$CONF" > /dev/null
      log "written"
    fi
    restart_hint ;;
  restore)
    if $SUDO test -e "$ORIG"; then
      $SUDO cp -p "$ORIG" "$CONF"
      $SUDO rm -f "$ORIG"
      log "$CONF put back as it was before apply"
    elif $SUDO test -e "$ABSENT"; then
      $SUDO rm -f "$CONF" "$ABSENT"
      log "$CONF removed (there was none before apply)"
    else
      log "nothing to restore: apply has not been run (no $ORIG)"
      exit 0
    fi
    restart_hint ;;
  check)
    if ! out="$(spot_nonloopback_listeners)"; then
      log "FATAL: cannot list listening sockets (sudo ss -Hltnup); refusing to call DCV safe"
      exit 2
    fi
    bad="$(printf '%s\n' "$out" | awk -v port=":$DCV_PORT" 'substr($2, length($2) - length(port) + 1) == port')"
    if [ -n "$bad" ]; then
      log "FATAL: something listens on port $DCV_PORT off loopback (proto local process pid):"
      printf '%s\n' "$bad" | sed 's/^/configure_dcv:   /' >&2
      log "fix: sudo bash $HERE/configure_dcv.sh apply --restart"
      exit 1
    fi
    if ! $SUDO test -e "$ORIG" && ! $SUDO test -e "$ABSENT"; then
      log "WARN: apply has not been run here, so DCV still allows any number of viewers"
    fi
    log "OK: nothing listens on port $DCV_PORT off loopback" ;;
esac
