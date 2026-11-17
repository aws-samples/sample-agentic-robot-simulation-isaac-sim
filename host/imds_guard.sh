#!/usr/bin/env bash
# imds_guard.sh -- let only root, the agent and the DCV server reach the instance metadata
# service (T26).
#
# Required: every launch wrapper (host/sim_lib.sh spot_require_imds_guard) refuses
# to start a container until `status` succeeds; SPOT_IMDS_GUARD=off skips that
# check with a warning. Runs ON the EC2 sim host, as root (or through sudo). Every container
# here runs with --network=host, so any of them can ask the instance metadata
# service (IMDS, 169.254.169.254) for the instance role's credentials; the
# IMDSv2 hop limit of 1 does not separate host-network containers. Only the
# agent calls AWS (Bedrock). AWS_EC2_METADATA_DISABLED=true in the other
# containers stops SDK clients only; this guard stops the packets.
#
#   sudo bash imds_guard.sh apply [--dry-run]    # add the rules (idempotent)
#   sudo bash imds_guard.sh remove [--dry-run]   # delete exactly what apply added
#   sudo bash imds_guard.sh status               # exit 0 if applied, 1 if not
#
# What apply adds, for IPv4 (iptables) and, where ip6tables exists, for the
# IPv6 endpoint fd00:ec2::254 (ip6tables):
#   a chain SPOT-IMDS, rebuilt on every apply:
#     -m owner --uid-owner 0 -j RETURN          root: the SSM agent, sudo aws ...
#     -m owner --uid-owner 10001 -j RETURN      the agent container (SPOT_AGENT_UID)
#     -m owner --uid-owner <dcv> -j RETURN      the Amazon DCV server, if the dcv
#                                               account exists (it reads its EC2
#                                               licence through the instance role)
#     -p tcp -j REJECT --reject-with tcp-reset  everyone else, at once
#     -j REJECT
#   and one jump at the top of OUTPUT: -d 169.254.169.254/32 -j SPOT-IMDS
# It matches the uid that owns each socket. Containers on --network=host share
# the host's network stack and run as host uids (no user-namespace remap), so
# their packets pass OUTPUT with their own uid. The uids are stated once, in
# host/sim_lib.sh; who is refused:
#   1234   the sim (isaac-sim)          10002  the HUD
#   10003  the presenter one-shots, the e2e client, spot-rec, Playwright
#   1000   ubuntu, and every other host user (run the AWS CLI with sudo)
# Not covered: containers on a Docker bridge network (docker build steps) go
# through FORWARD, not OUTPUT; the documented IMDSv2 hop limit of 1 keeps them
# out. The Amazon Time Sync address (169.254.169.123) is not touched.
# The DCV server (the system account dcv, created by the nice-dcv-server package)
# checks its licence on EC2 through the instance role; without access it retries
# IMDS for ever and never listens (measured on the GPU host, 2026-11-01). No
# container runs as that account, and apply refuses if its uid is a container's,
# ubuntu's or root's. Re-run apply after installing DCV.
# The rules live in the kernel only: re-run apply after a reboot (status says
# whether they are in place). remove deletes the jump and the chain and
# nothing else.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$HERE/sim_lib.sh"

CHAIN=SPOT-IMDS
SUDO=""; [ "$(id -u)" = 0 ] || SUDO=sudo
usage() { echo "usage: imds_guard.sh apply [--dry-run] | remove [--dry-run] | status" >&2; exit 2; }
log() { echo "imds_guard: $*" >&2; }

ACTION="${1:-}"
DRY=0
case "${2:-}" in "") ;; --dry-run) DRY=1 ;; *) usage ;; esac
[ $# -le 2 ] || usage
case "$ACTION" in
  apply|remove) ;;
  status) [ "$DRY" = 0 ] || usage ;;
  *) usage ;;
esac
if ! [[ "$SPOT_AGENT_UID" =~ ^[1-9][0-9]{3,}$ ]]; then
  log "FATAL: SPOT_AGENT_UID '$SPOT_AGENT_UID' must be a non-root uid of 1000 or more"; exit 2
fi

# The DCV server's account, if DCV is installed (see the header).
DCV_UID=""
if command -v getent >/dev/null 2>&1 && entry="$(getent passwd dcv 2>/dev/null)"; then
  DCV_UID="$(printf '%s\n' "$entry" | cut -d: -f3)"
fi
if [ -n "$DCV_UID" ]; then
  if ! [[ "$DCV_UID" =~ ^[1-9][0-9]*$ ]]; then
    log "FATAL: the dcv account's uid '$DCV_UID' is not a non-root uid"; exit 2
  fi
  for taken in "$SPOT_SIM_UID" "$SPOT_HUD_UID" "$SPOT_ONESHOT_UID" "$SPOT_AGENT_UID" 1000; do
    if [ "$DCV_UID" = "$taken" ]; then
      log "FATAL: the dcv account's uid $DCV_UID is also a container's or ubuntu's uid; refusing to let it reach IMDS"; exit 2
    fi
  done
  ALLOWED="uid 0, uid $SPOT_AGENT_UID and the DCV server (uid $DCV_UID)"
else
  ALLOWED="uid 0 and uid $SPOT_AGENT_UID"
fi

# Each address family: "tool endpoint".
FAMILIES="iptables 169.254.169.254/32"
if command -v ip6tables >/dev/null 2>&1; then
  FAMILIES="$FAMILIES
ip6tables fd00:ec2::254/128"
else
  log "ip6tables not found: guarding IPv4 only (the IPv6 endpoint must be enabled per instance)"
fi

# Run one rule command, or print it under --dry-run.
xt() {
  if [ "$DRY" = 1 ]; then echo "$*"; return 0; fi
  $SUDO "$@" < /dev/null
}
quiet() { $SUDO "$@" < /dev/null > /dev/null 2>&1; }

apply_family() {
  local tool="$1" dest="$2"
  if quiet "$tool" -w -L "$CHAIN" -n; then xt "$tool" -w -F "$CHAIN"; else xt "$tool" -w -N "$CHAIN"; fi
  xt "$tool" -w -A "$CHAIN" -m owner --uid-owner 0 -j RETURN
  xt "$tool" -w -A "$CHAIN" -m owner --uid-owner "$SPOT_AGENT_UID" -j RETURN
  if [ -n "$DCV_UID" ]; then xt "$tool" -w -A "$CHAIN" -m owner --uid-owner "$DCV_UID" -j RETURN; fi
  xt "$tool" -w -A "$CHAIN" -p tcp -j REJECT --reject-with tcp-reset
  xt "$tool" -w -A "$CHAIN" -j REJECT
  if ! quiet "$tool" -w -C OUTPUT -d "$dest" -j "$CHAIN"; then
    xt "$tool" -w -I OUTPUT 1 -d "$dest" -j "$CHAIN"
  fi
  log "$tool: $dest only for $ALLOWED"
}

remove_family() {
  local tool="$1" dest="$2" n=0
  while quiet "$tool" -w -C OUTPUT -d "$dest" -j "$CHAIN"; do
    xt "$tool" -w -D OUTPUT -d "$dest" -j "$CHAIN"
    n=$((n + 1))
    if [ "$DRY" = 1 ] || [ "$n" -ge 16 ]; then break; fi
  done
  if quiet "$tool" -w -L "$CHAIN" -n; then
    xt "$tool" -w -F "$CHAIN"
    xt "$tool" -w -X "$CHAIN"
  fi
  log "$tool: removed (IMDS reachable by every uid again)"
}

status_family() {
  local tool="$1" dest="$2"
  if quiet "$tool" -w -C OUTPUT -d "$dest" -j "$CHAIN" && quiet "$tool" -w -L "$CHAIN" -n; then
    echo "$tool: APPLIED ($dest only for $ALLOWED)"
    $SUDO "$tool" -w -S "$CHAIN" < /dev/null | sed 's/^/  /'
    return 0
  fi
  echo "$tool: NOT APPLIED ($dest open to every uid)"
  return 1
}

rc=0
while read -r tool dest; do
  case "$ACTION" in
    apply) apply_family "$tool" "$dest" ;;
    remove) remove_family "$tool" "$dest" ;;
    status) status_family "$tool" "$dest" || rc=1 ;;
  esac
done <<< "$FAMILIES"
exit "$rc"
