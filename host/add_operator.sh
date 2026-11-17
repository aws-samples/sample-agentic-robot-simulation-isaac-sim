#!/usr/bin/env bash
# add_operator.sh -- one OS account per operator, so the mission record names a person (T38).
#
# Runs ON the EC2 sim host, as root (sudo). With the Session Manager preferences
# in docs/iam/session-preferences.json (Run As on, no default user), Session
# Manager starts each operator's shell as the OS account that the SSMSessionRunAs
# tag on their IAM role or user names, and refuses the session if that account
# does not exist on the host; it never creates it. This script does:
#
#   sudo bash add_operator.sh NAME [--dry-run]          # create NAME (idempotent)
#   sudo bash add_operator.sh remove NAME [--dry-run]   # delete exactly what add created
#   sudo bash add_operator.sh status NAME               # exit 0 if NAME is set up, 1 if not
#
# NAME is a plain POSIX user name: a lower-case letter, then up to 31 lower-case
# letters, digits, '_' or '-'. The shared accounts (root, ubuntu, ssm-user) and any
# existing account that this script did not create are refused.
#
# What add creates:
#   the account NAME: a home directory, /bin/bash, its own group, the comment
#     "spot-demo operator" (how remove and status know it), and no password (it
#     stays locked): Session Manager starts the shell without one;
#   /etc/sudoers.d/spot-operator-NAME, mode 0440, owned by root, checked with
#     `visudo -cf` before it is installed and with `visudo -c` after (removed
#     again if that fails):
#       NAME ALL=(ubuntu) NOPASSWD: <ubuntu's login shell> ""
#     that is `sudo -iu ubuntu` (the login shell, no arguments) and nothing else.
#     The shell it opens keeps NAME in SUDO_USER, which host/run_agent.sh passes
#     as SPOT_OPERATOR_LOGIN and the mission JSONL records as the operator's login.
# NAME is not in the docker group and has no other sudo rule. ubuntu keeps sudo
# and docker, so every operator can still do all that ubuntu can: this gives
# attribution, not containment. remove deletes the sudoers file first, then the
# account and its home (userdel --remove); it refuses an account without the
# comment above. Nothing else on the host is changed.
#
# Env: SPOT_SUDOERS_DIR (default /etc/sudoers.d), SPOT_DEMO_USER (default ubuntu)
set -euo pipefail

SUDOERS_DIR="${SPOT_SUDOERS_DIR:-/etc/sudoers.d}"
DEMO_USER="${SPOT_DEMO_USER:-ubuntu}"
MARKER="spot-demo operator"
SHARED="root ubuntu ssm-user"   # agent/operator_console.py SHARED_LOGINS

usage() { echo "usage: add_operator.sh NAME [--dry-run] | remove NAME [--dry-run] | status NAME" >&2; exit 2; }
log() { echo "add_operator: $*" >&2; }
plain_name() { [[ "$1" =~ ^[a-z][a-z0-9_-]{0,31}$ ]]; }

ACTION=add
case "${1:-}" in
  remove|status) ACTION="$1"; shift ;;
esac
[ $# -ge 1 ] && [ $# -le 2 ] || usage
NAME="$1"
DRY=0
case "${2:-}" in "") ;; --dry-run) DRY=1 ;; *) usage ;; esac
[ "$ACTION" != status ] || [ "$DRY" = 0 ] || usage

if ! plain_name "$NAME"; then
  log "FATAL: the name given is not a plain user name (a lower-case letter, then up to 31" \
      "lower-case letters, digits, _ or -); nothing changed"
  exit 2
fi
for shared in $SHARED "$DEMO_USER"; do
  if [ "$NAME" = "$shared" ]; then
    log "FATAL: $NAME is a shared account; give each operator an account of their own"
    exit 2
  fi
done
plain_name "$DEMO_USER" || { log "FATAL: SPOT_DEMO_USER is not a plain user name"; exit 2; }
[ "$(id -u)" = 0 ] || { log "FATAL: run as root: sudo bash $0 $*"; exit 2; }

DROPIN="$SUDOERS_DIR/spot-operator-$NAME"

# Run one change, or print it under --dry-run.
run() {
  if [ "$DRY" = 1 ]; then echo "$*"; return 0; fi
  "$@" < /dev/null
}
passwd_entry() { getent passwd "$1" || true; }        # empty when there is none
field() { printf '%s\n' "$1" | cut -d: -f"$2"; }
ours() { [ -n "$1" ] && [ "$(field "$1" 5)" = "$MARKER" ]; }

demo_shell() {
  local demo shell
  demo="$(passwd_entry "$DEMO_USER")"
  [ -n "$demo" ] || { log "FATAL: the demo account $DEMO_USER does not exist on this host"; return 1; }
  shell="$(field "$demo" 7)"
  [[ "$shell" =~ ^/[A-Za-z0-9/_.-]+$ ]] || { log "FATAL: $DEMO_USER's login shell is not a plain path"; return 1; }
  echo "$shell"
}

# The sudoers file add installs: one comment and the one rule.
dropin_text() {
  printf '# host/add_operator.sh (T38): %s may run only "sudo -iu %s"\n' "$NAME" "$DEMO_USER"
  printf '%s ALL=(%s) NOPASSWD: %s ""\n' "$NAME" "$DEMO_USER" "$1"
}

install_dropin() {
  local want="$1" tmp
  # The temporary name contains a '.', so sudo never reads it (sudoers.d skips such files).
  tmp="$(mktemp "$SUDOERS_DIR/.spot-operator-$NAME.XXXXXX")"
  trap 'rm -f "$tmp"' EXIT
  printf '%s\n' "$want" > "$tmp"
  chmod 0440 "$tmp"
  chown root:root "$tmp"
  if ! visudo -cf "$tmp" > /dev/null; then
    log "FATAL: visudo rejects the rule for $NAME; nothing installed"
    exit 1
  fi
  mv -f "$tmp" "$DROPIN"
  trap - EXIT
  if ! visudo -c > /dev/null; then
    rm -f "$DROPIN"
    log "FATAL: the sudoers configuration does not parse with $DROPIN; removed it again"
    exit 1
  fi
}

case "$ACTION" in
  add)
    shell="$(demo_shell)" || exit 1
    current="$(passwd_entry "$NAME")"
    if [ -z "$current" ]; then
      run useradd --create-home --shell /bin/bash --user-group --comment "$MARKER" "$NAME"
      log "created the account $NAME (home directory, /bin/bash, its own group)"
    elif ours "$current"; then
      log "the account $NAME exists already (created by this script)"
    else
      log "FATAL: an account $NAME exists already and was not created by this script; nothing changed"
      exit 1
    fi
    run usermod --lock "$NAME"
    log "$NAME has no usable password (locked): Session Manager starts its shell without one"
    want="$(dropin_text "$shell")"
    if [ -f "$DROPIN" ] && [ "$(cat "$DROPIN")" = "$want" ]; then
      log "$DROPIN already allows $NAME only 'sudo -iu $DEMO_USER'"
    elif [ "$DRY" = 1 ]; then
      echo "install $DROPIN (0440, root, after visudo -cf):"
      printf '%s\n' "$want"
    else
      install_dropin "$want"
      log "installed $DROPIN (0440, root): $NAME may run only 'sudo -iu $DEMO_USER'"
    fi
    log "next: tag the IAM role or user of this operator with SSMSessionRunAs=$NAME" \
        "(docs/iam/README.md)" ;;
  remove)
    current="$(passwd_entry "$NAME")"
    if [ -n "$current" ] && ! ours "$current"; then
      log "FATAL: the account $NAME was not created by this script; nothing removed"
      exit 1
    fi
    if [ -e "$DROPIN" ] || [ -L "$DROPIN" ]; then
      run rm -f "$DROPIN"
      log "removed $DROPIN"
    else
      log "no $DROPIN"
    fi
    if [ -n "$current" ]; then
      run userdel --remove "$NAME"
      log "deleted the account $NAME and its home directory"
    else
      log "no account $NAME"
    fi ;;
  status)
    rc=0
    current="$(passwd_entry "$NAME")"
    if ours "$current"; then
      echo "account: $NAME (spot-demo operator)"
      case "$(passwd --status "$NAME" | cut -d' ' -f2)" in
        L) echo "password: locked" ;;
        *) echo "password: NOT LOCKED (run add again)"; rc=1 ;;
      esac
    else
      echo "account: MISSING, or not created by this script"; rc=1
    fi
    shell="$(demo_shell)" || exit 1
    if [ -f "$DROPIN" ] && [ "$(cat "$DROPIN")" = "$(dropin_text "$shell")" ] \
        && [ -n "$(find "$DROPIN" -maxdepth 0 -perm 0440)" ]; then
      echo "sudo: only 'sudo -iu $DEMO_USER' ($DROPIN)"
    else
      echo "sudo: $DROPIN MISSING or changed (run add again)"; rc=1
    fi
    exit "$rc" ;;
esac
