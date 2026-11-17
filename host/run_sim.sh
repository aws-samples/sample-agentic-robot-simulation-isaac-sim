#!/usr/bin/env bash
# run_sim.sh -- start the Isaac Sim 6.1 container running the Spot demo app.
#
# Runs ON the EC2 sim host (deployed there by scripts/deploy-to-host.sh).
# Wraps `docker run` of nvcr.io/nvidia/isaac-sim:6.1.0 (pinned by digest) with:
#   - every Isaac cache bind-mounted from /opt/isaac-cache (warm starts:
#     kit, ov, pip, GLCache, ComputeCache, omniverse logs, ov data)
#   - /opt/spot-demo/sim mounted read-only at /workspace/sim
#   - entrypoint /isaac-sim/python.sh /workspace/sim/isaac_app.py
#
# Other args go to isaac_app.py, but only its own options (checked here, before
# the running sim is touched; anything else, including Kit settings and
# extension flags, is refused with exit 2), e.g.:
#   bash run_sim.sh                                    # full run, headless
#   bash run_sim.sh --headless true --smoke --no-cameras
#   bash run_sim.sh --cameras all                      # all 5 body cameras
#   bash run_sim.sh --gui                              # window on the DCV desktop
#   bash run_sim.sh --editor                           # same, stock Isaac editor layout
# (isaac_app.py's --headless takes a value: true|false, default true)
#
# --gui (consumed here, not passed through) runs the app non-headless on the
# X display of the ubuntu desktop session that the NICE DCV console session
# shows. It changes no host security setting itself, and refuses to start
# while DCV listens off loopback (host/configure_dcv.sh check, T1). It (1) needs that
# desktop to exist (if the console is at the GDM greeter it stops and points
# to --prepare-desktop), (2) grants
# X access to the container uid ONLY (xhost +si:localuser:#1234, never a
# blanket `xhost +`) and refuses a display whose access list lets in anyone
# but ubuntu and that uid, (3) passes DISPLAY + that display's socket
# (read-only) + all NVIDIA driver
# capabilities, and (4) appends --headless false --ui presentation: the Kit
# window goes fullscreen showing only the viewport (no Stage / Property /
# Content / Console panels, no menus, no viewport toolbar; the fullscreen
# window also covers the GNOME top bar and dock). If the DCV console is
# smaller than SPOT_GUI_LAYOUT (default 1920x1080; no viewer attached yet) it
# is set to that layout first, as a 1080p viewer would. --editor = --gui
# with --ui editor (the full Isaac Sim editor, for debugging). Switch at
# runtime without a restart: bash set_view.sh --ui editor|presentation.
# Operator steps and troubleshooting: host/gui_notes.md.
#
# --background (consumed here; the presenter path) starts the container
# detached, streams its output to a NEW per-run log
# $SPOT_DEMO_DIR/logs/sim-<run_id>.log (symlink logs/sim-latest.log), and
# blocks until THIS launch prints "FACADE-READY ... run_id=<run_id>" in
# `docker logs spot-sim` (the current container, never an old file); exit 0
# = ready, 1 = the container died / timed out (log tail printed), or something
# listens off loopback that should not (see check_listeners, T44). Without it
# the container runs in the foreground (e2e_ros_test.sh / smoke_test.sh
# capture that output themselves).
#   bash run_sim.sh --background --headless true      # ~40 s warm, then returns
#   bash run_sim.sh --background --gui
# --status (alone) prints the running sim's FACADE-READY line (exit 0) or
# says it is not ready (exit 1).
#
# --prepare-desktop (alone, once per host, opt-in) gets the console desktop
# ready for --gui and changes the host's desktop security settings to do it:
# GDM autologin for ubuntu (/etc/gdm3/custom.conf), screen lock, idle blank
# and dim off, notification banners and crash pop-ups off, no first-login
# wizard, and a GDM + dcvserver restart only if the console is at the
# greeter. It prints each change first and records the originals (root-owned,
# /var/lib/spot-demo/desktop) before touching anything.
# --restore-desktop (alone) puts the recorded originals back and deletes the
# record. Autologin goes off when GDM next starts (reboot, or a gdm3 restart,
# which logs ubuntu out and drops DCV viewers).
#
# Stop it from another shell with:
#   bash run_sim.sh --stop
# --stop, and any launch (which replaces a running sim), refuses while an
# agent or mission container (spot-agent, spot-mission) is running, because
# the mission would lose gas and odometry mid-entry; --force overrides (T41).
#
# Env: ACCEPT_EULA      must be Y, set by the operator after reading the NVIDIA
#                       licence (the script refuses to start without it)
#      PRIVACY_CONSENT  Y opts in to NVIDIA data collection; default off
#      SPOT_RUN_ID      (default: new per launch) nonce isaac_app.py prints
#      SPOT_SIM_READY_TIMEOUT (default 300 s) for --background
#      ISAAC_IMAGE      (default nvcr.io/nvidia/isaac-sim:6.1.0, pinned by
#                       digest in host/sim_lib.sh; an override must be name@sha256:...)
#      ISAAC_CACHE_ROOT (default /opt/isaac-cache)
#      SPOT_DEMO_DIR    (default /opt/spot-demo)
#      SPOT_GUI_USER    (default ubuntu)  desktop owner for --gui
#      SPOT_GUI_DISPLAY (default: auto-detect that user's X server)
#      SPOT_GUI_LAYOUT  (default 1920x1080) minimum DCV console layout for
#                       --gui/--editor; empty = leave the layout alone
#      SPOT_DESKTOP_STATE_DIR (default /var/lib/spot-demo/desktop) where
#                       --prepare-desktop records the originals
#      SPOT_GDM_CONF    (default /etc/gdm3/custom.conf)
#      ROS_DOMAIN_ID    (default 42) DDS domain; discovery stays on this
#                       host's loopback (host/sim_lib.sh spot_dds_args)
set -euo pipefail

CACHE_ROOT="${ISAAC_CACHE_ROOT:-/opt/isaac-cache}"
DEMO_DIR="${SPOT_DEMO_DIR:-/opt/spot-demo}"
GUI_USER="${SPOT_GUI_USER:-ubuntu}"
[[ "$GUI_USER" =~ ^[a-z_][a-z0-9_-]{0,31}$ ]] || { echo "run_sim: FATAL: bad SPOT_GUI_USER '$GUI_USER'" >&2; exit 2; }
DESKTOP_STATE="${SPOT_DESKTOP_STATE_DIR:-/var/lib/spot-demo/desktop}"
GDM_CONF="${SPOT_GDM_CONF:-/etc/gdm3/custom.conf}"

# The 6.1 image runs as isaac-sim (uid/gid 1234) with HOME=/isaac-sim, so the
# caches live under /isaac-sim, and the host dirs must be owned by 1234 or
# Kit can't write its shader/OGN/datastore caches.
ISAAC_UID=1234

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPOT_DEMO_DIR="$DEMO_DIR"
# shellcheck source=host/sim_lib.sh
source "$HERE/sim_lib.sh"
# isaac-sim:6.1.0, pinned by digest (T31); an override must be pinned too.
ISAAC_IMAGE="${ISAAC_IMAGE:-$SPOT_ISAAC_IMAGE}"

# isaac_app.py's own options (tests/test_hardening_host.py keeps these equal to
# its argparse). Anything else is refused before the running sim is touched:
# a typo of --status used to reach `docker rm -f spot-sim` (T41), and Kit
# settings or extension flags (--/..., --enable, --ext-folder) could turn on
# a remote viewer or another listener inside the sim (T44).
SIM_APP_FLAGS=" --smoke --no-cameras "
SIM_APP_VALUE_OPTS=" --headless --cameras --camera-hz --scene --device --render-hz --fabric --realtime --actuators --gas-initial --ui --view --gui-res --battery-drain "
bad_arg() {
  echo "run_sim: FATAL: $1 (run_sim.sh options: --gui --editor --background --status --stop" \
       "--force --prepare-desktop --restore-desktop; isaac_app.py options:$SIM_APP_FLAGS${SIM_APP_VALUE_OPTS# })" >&2
  exit 2
}

GUI=0
UI_MODE=presentation
BACKGROUND=0
FORCE=0
ACTION=run
NARGS=$#
APP_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --gui) GUI=1 ;;
    --editor) GUI=1; UI_MODE=editor ;;
    --background|--bg) BACKGROUND=1 ;;
    --force) FORCE=1 ;;
    --status|--stop|--prepare-desktop|--restore-desktop) ACTION="${1#--}" ;;
    --*=*)
      case "$SIM_APP_VALUE_OPTS" in
        *" ${1%%=*} "*) [ -n "${1#*=}" ] || bad_arg "${1%%=*} needs a value"; APP_ARGS+=("$1") ;;
        *) bad_arg "unknown option '${1%%=*}'" ;;
      esac ;;
    --*)
      if [[ "$SIM_APP_FLAGS" == *" $1 "* ]]; then
        APP_ARGS+=("$1")
      elif [[ "$SIM_APP_VALUE_OPTS" == *" $1 "* ]]; then
        if [ $# -lt 2 ] || [ -z "$2" ] || [ "${2#-}" != "$2" ]; then bad_arg "$1 needs a value"; fi
        APP_ARGS+=("$1" "$2"); shift
      else
        bad_arg "unknown option '$1'"
      fi ;;
    *) bad_arg "unexpected argument '$1'" ;;
  esac
  shift
done
if [ "$ACTION" != run ] && [ $((NARGS - FORCE)) != 1 ]; then
  echo "run_sim: --$ACTION takes no other arguments" >&2; exit 2
fi
if [ "$FORCE" = 1 ] && [ "$ACTION" != run ] && [ "$ACTION" != stop ]; then
  echo "run_sim: --force applies to a launch or --stop only" >&2; exit 2
fi

log() { echo "run_sim: $*" >&2; }

# Licence and telemetry are the operator's choices, never this script's (T35).
# The NVIDIA image does not start without ACCEPT_EULA=Y, so the operator must
# set it after reading the licence; the script only checks it. Telemetry is
# opt-in and off unless PRIVACY_CONSENT=Y: NVIDIA's container guide says
# "You may opt-out by not setting this flag", so it is then not passed, and
# OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1 turns off Kit's anonymous data
# collection (docs.isaacsim.omniverse.nvidia.com/latest/installation/
# install_container.html and .../latest/common/data-collection.html).
check_licence_and_consent() {
  if [ "${ACCEPT_EULA:-}" != Y ]; then
    log "FATAL: Isaac Sim needs the NVIDIA licence accepted, and this script does not accept it" \
        "for you. Read the licence the Isaac Sim container guide links" \
        "(https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html)," \
        "and the additional software licence" \
        "(https://docs.isaacsim.omniverse.nvidia.com/latest/common/license-isaac-sim-additional.html);" \
        "if you accept them, set ACCEPT_EULA=Y yourself, e.g. ACCEPT_EULA=Y bash run_sim.sh ..."
    exit 2
  fi
  case "${PRIVACY_CONSENT:-}" in
    Y) PRIVACY_ARGS=(-e PRIVACY_CONSENT=Y)
       log "telemetry: PRIVACY_CONSENT=Y set by the operator, NVIDIA data collection is ON" ;;
    ""|N) PRIVACY_ARGS=(-e OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1)
       log "telemetry: off (opt in with PRIVACY_CONSENT=Y)"
       if [ -e "$CACHE_ROOT/config/privacy.toml" ]; then
         log "WARN: $CACHE_ROOT/config/privacy.toml from an earlier launch may still record a consent;" \
             "check it (it is mounted into the container)"
       fi ;;
    *) log "FATAL: PRIVACY_CONSENT must be Y (opt in to NVIDIA data collection) or N/unset, not '$PRIVACY_CONSENT'"
       exit 2 ;;
  esac
}

# --- GUI desktop helpers -----------------------------------------------------

# Print ":N" for the X server owned by $GUI_USER (GDM starts the user's Xorg
# with -displayfd, so the display number is only visible via its socket).
find_user_display() {
  local s
  for s in /tmp/.X11-unix/X*; do
    [ -S "$s" ] || continue
    if [ "$(stat -c %U "$s")" = "$GUI_USER" ]; then echo ":${s##*/X}"; return 0; fi
  done
  return 1
}

# -auth file of that user's Xorg (needed to talk to it as the user for xhost).
find_user_xauth() {
  local pid auth
  for pid in $(pgrep -u "$GUI_USER" -x Xorg 2>/dev/null); do
    auth="$(tr '\0' '\n' < "/proc/$pid/cmdline" | grep -A1 -x -- '-auth' | tail -1)"
    [ -n "$auth" ] && { echo "$auth"; return 0; }
  done
  return 1
}

# --- desktop settings: opt-in, recorded, restorable (T28) --------------------
# The user settings --prepare-desktop changes, as "schema key value". Why
# each: DCV's own login already authenticates the operator, so the console
# lock and blank only interrupt a demo; banners ("Printer added" at every DCV
# connect, seen 2026-09-27) and Ubuntu's crash pop-up (the first GDM restart
# leaves a /var/crash report, so it WILL appear) would cover the viewport.
DESKTOP_SETTINGS="org.gnome.desktop.screensaver lock-enabled false
org.gnome.desktop.session idle-delay 0
org.gnome.settings-daemon.plugins.power idle-dim false
com.ubuntu.update-notifier show-apport-crashes false
org.gnome.desktop.notifications show-banners false"
# Cosmetic, per launch and only on a prepared desktop: in presentation mode
# the dock shows only on hover (the fullscreen Kit window covers it anyway;
# this keeps it off screen while Kit starts). Editor mode puts back the
# recorded values.
DOCK_SETTINGS="org.gnome.shell.extensions.dash-to-dock dock-fixed false
org.gnome.shell.extensions.dash-to-dock autohide true
org.gnome.shell.extensions.dash-to-dock intellihide true"

# gsettings as the desktop user: on its session bus, or, with no session,
# through a private bus (dconf then writes the user's database directly).
user_gsettings() {
  local bus
  bus="/run/user/$(id -u "$GUI_USER")/bus"
  if [ -S "$bus" ]; then
    sudo -u "$GUI_USER" env DBUS_SESSION_BUS_ADDRESS="unix:path=$bus" gsettings "$@"
  elif command -v dbus-run-session >/dev/null 2>&1; then
    sudo -u "$GUI_USER" dbus-run-session -- gsettings "$@"
  else
    return 1
  fi
}

user_home() { getent passwd "$GUI_USER" | cut -d: -f6; }

desktop_prepared() { sudo test -f "$DESKTOP_STATE/gsettings"; }

# Set AutomaticLoginEnable/AutomaticLogin in [daemon], dropping any earlier
# (or commented) copies. Why autologin: DCV's console session mirrors the
# physical console, which only shows a desktop once a user is logged in
# there; without it the operator lands on the GDM greeter and the Isaac
# window has no X server to open on. Access stays gated by DCV's own login
# (ubuntu password) over the SSM tunnel.
enable_gdm_autologin() {
  local tmp
  if sudo grep -qE "^AutomaticLogin=${GUI_USER}\$" "$GDM_CONF" 2>/dev/null \
     && sudo grep -qE '^AutomaticLoginEnable=(true|True)$' "$GDM_CONF"; then
    return 0
  fi
  tmp="$(mktemp)" || return 1
  { sudo cat "$GDM_CONF" 2>/dev/null || true; } | awk -v user="$GUI_USER" '
    /^[[:space:]]*#?[[:space:]]*AutomaticLogin(Enable)?[[:space:]]*=/ { next }
    { print }
    /^\[daemon\][[:space:]]*$/ { print "AutomaticLoginEnable=true"; print "AutomaticLogin=" user; done = 1 }
    END { if (!done) { print "[daemon]"; print "AutomaticLoginEnable=true"; print "AutomaticLogin=" user } }' > "$tmp"
  sudo cp "$tmp" "$GDM_CONF"; rm -f "$tmp"
  sudo grep -qE "^AutomaticLogin=${GUI_USER}\$" "$GDM_CONF" \
    || { log "FATAL: could not write autologin to $GDM_CONF"; return 1; }
}

# Bring the user's desktop up on the console (prepare only): restart GDM
# once if only the greeter is up, then dcvserver, whose console agent was
# attached to the greeter's X server.
ensure_console_desktop() {
  if [ -n "${SPOT_GUI_DISPLAY:-}" ] || find_user_display >/dev/null; then return 0; fi
  log "no X desktop for $GUI_USER (console is at the GDM greeter): restarting gdm3 (DCV viewers reconnect)"
  sudo systemctl restart gdm3
  for _ in $(seq 1 45); do
    if find_user_display >/dev/null; then break; fi
    sleep 2
  done
  find_user_display >/dev/null || { log "FATAL: $GUI_USER desktop did not come up after GDM restart"; return 1; }
  if systemctl list-unit-files dcvserver.service >/dev/null 2>&1; then
    sleep 3
    sudo systemctl restart dcvserver || log "WARN: dcvserver restart failed (see host/gui_notes.md)"
  fi
}

prepare_desktop() {
  local home schema key value cur rec=""
  home="$(user_home)"
  [ -n "$home" ] || { log "FATAL: no user $GUI_USER"; exit 1; }
  log "--prepare-desktop changes these host settings (undo: bash run_sim.sh --restore-desktop):"
  log "  $GDM_CONF: AutomaticLoginEnable=true, AutomaticLogin=$GUI_USER (the console logs in with no password; DCV's login still gates remote access)"
  while read -r schema key value; do
    log "  gsettings ($GUI_USER) $schema $key -> $value"
  done <<< "$DESKTOP_SETTINGS"
  log "  $home/.config/gnome-initial-setup-done (no first-login wizard), and stops a running update-notifier"
  log "  only if the console is at the GDM greeter: restarts gdm3 and dcvserver (drops DCV viewers)"
  log "  originals recorded first in $DESKTOP_STATE (root-owned)"
  if ! { sudo mkdir -p "$DESKTOP_STATE" && sudo chmod 0755 "$DESKTOP_STATE"; }; then
    log "FATAL: cannot create $DESKTOP_STATE"; exit 1
  fi
  # GDM: record the original once. An earlier run_sim.sh --gui kept its own
  # backup (custom.conf.spot-demo.bak) before it turned autologin on: that is
  # the original.
  if ! sudo test -e "$DESKTOP_STATE/gdm-custom.conf" && ! sudo test -e "$DESKTOP_STATE/gdm-custom.conf.absent"; then
    if sudo test -f "$GDM_CONF.spot-demo.bak"; then
      sudo cp -p "$GDM_CONF.spot-demo.bak" "$DESKTOP_STATE/gdm-custom.conf"
      log "WARN: an earlier run_sim.sh --gui already changed this desktop; the user settings recorded now may be its values, not the stock ones (gsettings reset <schema> <key> gives the stock value)"
    elif sudo test -f "$GDM_CONF"; then
      sudo cp -p "$GDM_CONF" "$DESKTOP_STATE/gdm-custom.conf"
    else
      sudo touch "$DESKTOP_STATE/gdm-custom.conf.absent"
    fi
  fi
  enable_gdm_autologin || exit 1
  ensure_console_desktop || exit 1
  # User settings: record the originals once (the first prepare wins).
  if ! sudo test -e "$DESKTOP_STATE/gsettings"; then
    while read -r schema key value; do
      if cur="$(user_gsettings get "$schema" "$key" 2>/dev/null < /dev/null)"; then
        rec="$rec$schema $key $cur"$'\n'
      fi
    done <<< "$DESKTOP_SETTINGS"$'\n'"$DOCK_SETTINGS"
    printf '%s' "$rec" | sudo tee "$DESKTOP_STATE/gsettings" > /dev/null
  fi
  if [ ! -e "$home/.config/gnome-initial-setup-done" ]; then
    sudo -u "$GUI_USER" mkdir -p "$home/.config"
    sudo -u "$GUI_USER" touch "$home/.config/gnome-initial-setup-done"
    sudo touch "$DESKTOP_STATE/initial-setup-created"
  fi
  while read -r schema key value; do
    user_gsettings set "$schema" "$key" "$value" < /dev/null || log "WARN: could not set $schema $key"
  done <<< "$DESKTOP_SETTINGS"
  # Dismiss a crash dialog already on screen (update-notifier restarts at the
  # next login and then honours the setting above).
  sudo pkill -u "$GUI_USER" -x update-notifier 2>/dev/null || true
  log "desktop prepared for --gui; originals in $DESKTOP_STATE"
}

restore_desktop() {
  local home schema key value failed=0
  if ! sudo test -d "$DESKTOP_STATE" || [ -z "$(sudo ls -A "$DESKTOP_STATE" 2>/dev/null)" ]; then
    log "nothing to restore: no record in $DESKTOP_STATE (--prepare-desktop was not run)"
    return 0
  fi
  home="$(user_home)"
  log "--restore-desktop puts back what --prepare-desktop recorded:"
  if sudo test -f "$DESKTOP_STATE/gsettings"; then
    while read -r schema key value; do
      [ -n "$schema" ] || continue
      if ! [[ "$schema" =~ ^[A-Za-z0-9.-]+$ && "$key" =~ ^[a-z0-9-]+$ ]]; then
        log "WARN: skipping a malformed record line: $schema $key"; failed=1; continue
      fi
      if user_gsettings set "$schema" "$key" "$value" < /dev/null; then
        log "  gsettings ($GUI_USER) $schema $key -> $value"
      else
        log "WARN: could not put back $schema $key ($value)"; failed=1
      fi
    done <<< "$(sudo cat "$DESKTOP_STATE/gsettings")"
    [ "$failed" = 1 ] || sudo rm -f "$DESKTOP_STATE/gsettings"
  fi
  if sudo test -e "$DESKTOP_STATE/initial-setup-created"; then
    sudo rm -f "$home/.config/gnome-initial-setup-done" "$DESKTOP_STATE/initial-setup-created"
    log "  removed $home/.config/gnome-initial-setup-done"
  fi
  if sudo test -f "$DESKTOP_STATE/gdm-custom.conf"; then
    sudo cp -p "$DESKTOP_STATE/gdm-custom.conf" "$GDM_CONF"
    sudo rm -f "$DESKTOP_STATE/gdm-custom.conf" "$GDM_CONF.spot-demo.bak"
    log "  $GDM_CONF put back as recorded"
  elif sudo test -e "$DESKTOP_STATE/gdm-custom.conf.absent"; then
    sudo rm -f "$GDM_CONF" "$DESKTOP_STATE/gdm-custom.conf.absent"
    log "  $GDM_CONF removed (it did not exist before)"
  fi
  log "GDM reads $GDM_CONF when it starts: autologin goes off at the next reboot or" \
      "'sudo systemctl restart gdm3' (which logs $GUI_USER out and drops DCV viewers)"
  [ "$failed" = 0 ] || { log "some settings were not put back; their record stays in $DESKTOP_STATE"; return 1; }
  sudo rmdir "$DESKTOP_STATE" 2>/dev/null || true
}

# Per --gui launch, only on a prepared desktop: the dock (see DOCK_SETTINGS).
tune_dock() {
  local schema key value
  if [ "$UI_MODE" = presentation ]; then
    while read -r schema key value; do
      user_gsettings set "$schema" "$key" "$value" < /dev/null 2>/dev/null || true
    done <<< "$DOCK_SETTINGS"
  else
    while read -r schema key value; do
      case "$schema" in *dash-to-dock) user_gsettings set "$schema" "$key" "$value" < /dev/null 2>/dev/null || true ;; esac
    done <<< "$(sudo cat "$DESKTOP_STATE/gsettings")"
  fi
}

ensure_gui_desktop() {
  if [ -n "${SPOT_GUI_DISPLAY:-}" ]; then GUI_DISPLAY="$SPOT_GUI_DISPLAY"; return 0; fi
  if GUI_DISPLAY="$(find_user_display)"; then return 0; fi
  log "FATAL: no X desktop for $GUI_USER: the console is at the GDM greeter. --gui changes no host" \
      "settings itself. Run 'bash run_sim.sh --prepare-desktop' once (it lists what it changes:" \
      "GDM autologin, screen lock and blank off, banners off; undo with --restore-desktop), then retry."
  return 1
}

# With no DCV viewer attached the console desktop is 1024x768, so a
# fullscreen Kit window would be too; raise it to SPOT_GUI_LAYOUT (a viewer
# that connects later resizes it to its own window as usual). Never shrinks,
# and never touches the layout of an attached viewer: the desktop is then the
# viewer's window, and forcing 1920x1080 on a smaller one would crop or
# scale the fullscreen render in it (tests/test_sensors.py TestRunSimLayout).
ensure_gui_layout() {
  local want="${SPOT_GUI_LAYOUT-1920x1080}" cur ww wh cw ch conns
  [ -n "$want" ] || return 0
  [[ "$want" =~ ^([0-9]{3,4})x([0-9]{3,4})$ ]] || { log "WARN: bad SPOT_GUI_LAYOUT '$want'"; return 0; }
  ww="${BASH_REMATCH[1]}"; wh="${BASH_REMATCH[2]}"
  command -v dcv >/dev/null 2>&1 || return 0
  cur="$(sudo dcv describe-session console 2>/dev/null | sed -n 's/.*display layout: *\([0-9]*\)x\([0-9]*\).*/\1 \2/p' | head -1)"
  read -r cw ch <<< "${cur:-0 0}"
  # "There are no clients connected to the session." (DCV 2024+, host 2026-09-27)
  if conns="$(sudo dcv list-connections console 2>/dev/null)" && [ -n "$conns" ] \
      && ! grep -qi "no clients" <<< "$conns"; then
    log "DCV viewer attached: keeping its layout ${cw}x${ch}"
    return 0
  fi
  if [ "${cw:-0}" -lt "$ww" ] || [ "${ch:-0}" -lt "$wh" ]; then
    log "DCV console layout ${cw}x${ch} -> ${ww}x${wh}+0+0 (no viewer attached?)"
    sudo dcv set-display-layout --session console "${ww}x${wh}+0+0" || log "WARN: dcv set-display-layout failed"
  fi
}

# The X server must let in only its owner and the sim's uid (T28). Every
# --network=host container can reach the display through the abstract X
# socket whatever is mounted, so its access list is the real boundary: an
# open server, a LOCAL: or INET: entry, or another user would hand the
# desktop (and through it the docker group) to more containers.
# Reads an `xhost` listing on stdin; prints each entry that is not allowed.
x_acl_unexpected() {
  local line first=1 bad=0
  while IFS= read -r line; do
    if [ "$first" = 1 ]; then
      first=0
      case "$line" in "access control enabled"*) continue ;; esac
    fi
    case "$line" in
      # The Amazon DCV server adds its own account (dcv) to the console it
      # streams; no container runs as that account (host/sim_lib.sh uids).
      ""|"SI:localuser:$GUI_USER"|"SI:localuser:#$ISAAC_UID"|"SI:localuser:dcv") ;;
      *) echo "$line"; bad=1 ;;
    esac
  done
  if [ "$first" = 1 ]; then echo "(no access list)"; bad=1; fi
  return "$bad"
}

setup_gui() {
  # The DCV viewer is how the operator sees this window: it must listen on
  # loopback only (T1; host/configure_dcv.sh apply sets it up).
  if ! bash "$HERE/configure_dcv.sh" check; then
    log "FATAL: refusing --gui while DCV (port 8443) is reachable off loopback" \
        "(sudo bash $HERE/configure_dcv.sh apply --restart)"
    exit 1
  fi
  ensure_gui_desktop || exit 1
  if desktop_prepared; then
    tune_dock
  else
    log "WARN: this desktop was not prepared, so the screen may lock or blank mid-demo" \
        "(bash run_sim.sh --prepare-desktop turns that off and records the originals)"
  fi
  ensure_gui_layout || true
  local xauth acl extra
  if [[ "$GUI_DISPLAY" =~ ^:([0-9]+)(\.[0-9]+)?$ ]]; then
    X_SOCKET="/tmp/.X11-unix/X${BASH_REMATCH[1]}"
  else
    log "FATAL: DISPLAY '$GUI_DISPLAY' is not a local X display (:N)"; exit 1
  fi
  xauth="$(find_user_xauth || true)"
  # Container runs as uid 1234 with no matching host account, so grant the
  # numeric uid via the server-interpreted localuser family (unix socket
  # peer credentials; nothing is opened on the network). Re-run each launch:
  # the grant lives in the X server and is lost when it restarts.
  if ! sudo -u "$GUI_USER" env DISPLAY="$GUI_DISPLAY" ${xauth:+XAUTHORITY="$xauth"} \
         xhost "+si:localuser:#$ISAAC_UID" >/dev/null; then
    log "FATAL: xhost grant for uid $ISAAC_UID on $GUI_DISPLAY failed"; exit 1
  fi
  if ! acl="$(sudo -u "$GUI_USER" env DISPLAY="$GUI_DISPLAY" ${xauth:+XAUTHORITY="$xauth"} xhost 2>&1)"; then
    log "FATAL: cannot read the X access list of $GUI_DISPLAY"; exit 1
  fi
  if ! extra="$(x_acl_unexpected <<< "$acl")"; then
    log "FATAL: the X server on $GUI_DISPLAY lets in more than $GUI_USER, uid $ISAAC_UID and the DCV server:" \
        "$(echo "$extra" | tr '\n' ' ')- remove those entries (xhost, each with a leading minus) and retry"
    exit 1
  fi
  log "GUI on DISPLAY=$GUI_DISPLAY (user $GUI_USER, X access granted to uid $ISAAC_UID only), ui=$UI_MODE"
}

# --- listener check after READY (T44, M44) -----------------------------------
# run_sim.sh has no remote-viewer option and refuses the Kit flags that could
# add one, so nothing should listen off loopback because of the sim. Checked
# once the sim is READY, in the background and the foreground modes: fail loudly when anything listens off
# loopback on the DCV (8443) or HUD (8080) port, a Kit or remote-viewer port
# (Kit HTTP 8011/8211, NVIDIA streaming 47998/49100, a ROS bridge 8765), or
# this domain's RTPS ports; and when any other process listens off loopback
# unless it is the SSM agent, DCV itself or the DHCP client (extend with
# SPOT_LISTEN_ALLOW="name ..."; ss prints process names). A listener the sim
# opened stops the sim (fail closed); any other leaves it running.
SENSITIVE_PORTS=" 8443 8080 8011 8211 8765 47998 49100 "
LISTEN_ALLOW=" amazon-ssm-agent ssm-agent-worker ssm-session-worker dcvserver dcvagent systemd-network ${SPOT_LISTEN_ALLOW:-} "

check_listeners() {
  local out proto laddr name pid port lo ours bad=0 stop_sim=0 sim_pids
  lo=$((7400 + 250 * ${ROS_DOMAIN_ID:-42}))
  if ! out="$(spot_nonloopback_listeners)"; then
    log "FATAL: cannot list listening sockets (sudo ss -Hltnup)"; return 1
  fi
  sim_pids=" $(docker top spot-sim -eo pid 2>/dev/null | tail -n +2 | tr '\n' ' ') "
  while read -r proto laddr name pid; do
    [ -n "$proto" ] || continue
    port="${laddr##*:}"
    ours=0
    case "$sim_pids" in *" $pid "*) ours=1; stop_sim=1 ;; esac
    if [[ "$SENSITIVE_PORTS" == *" $port "* ]] || { [ "$port" -ge "$lo" ] && [ "$port" -lt $((lo + 250)) ]; }; then
      log "FATAL: $proto $laddr ($name pid $pid) listens off loopback on a DCV, HUD, Kit, remote-viewer or DDS port"
      bad=1
      case "$port" in 8443|8080) ;; *) stop_sim=1 ;; esac
    elif [ "$ours" = 1 ] || [[ "$LISTEN_ALLOW" != *" $name "* ]]; then
      log "FATAL: $proto $laddr ($name pid $pid) listens off loopback and is not the SSM agent, DCV or" \
          "the DHCP client (stop it, or accept it with SPOT_LISTEN_ALLOW=\"$name\")"
      bad=1
    fi
  done <<< "$out"
  if [ "$bad" = 1 ] && [ "$stop_sim" = 1 ]; then
    log "stopping the sim: a listener above is the sim's, or on a Kit, remote-viewer or DDS port"
    docker rm -f spot-sim >/dev/null 2>&1 || true
  fi
  [ "$bad" = 0 ] || return 1
  log "listeners: none off loopback except allowed ones (checked after READY)"
}

# --- one-shot actions --------------------------------------------------------

case "$ACTION" in
  status)
    if line="$(spot_sim_ready_line)"; then
      echo "run_sim: READY ($line; container started $(docker inspect -f '{{.State.StartedAt}}' spot-sim 2>/dev/null))"
      exit 0
    fi
    if docker inspect spot-sim >/dev/null 2>&1; then
      echo "run_sim: NOT READY: container spot-sim is up but has not printed FACADE-READY yet (docker logs -f spot-sim)"
    else
      echo "run_sim: NOT RUNNING: no spot-sim container (start: bash run_sim.sh --background --headless true)"
    fi
    exit 1 ;;
  stop)
    spot_guard_sim_stop "$FORCE" "stop the sim" || exit 1
    if ! docker inspect spot-sim >/dev/null 2>&1; then
      echo "run_sim: no spot-sim container"; exit 0
    fi
    docker stop -t 10 spot-sim >/dev/null 2>&1 || true
    docker rm -f spot-sim >/dev/null 2>&1 || true
    echo "run_sim: spot-sim stopped"
    exit 0 ;;
  prepare-desktop) prepare_desktop; exit 0 ;;
  restore-desktop) restore_desktop; exit $? ;;
esac

# --- launch -----------------------------------------------------------------

[ -d "$DEMO_DIR/sim" ] || { log "FATAL: $DEMO_DIR/sim not found -- run scripts/deploy-to-host.sh first"; exit 1; }
check_licence_and_consent
# A launch replaces a running sim (T41).
spot_guard_sim_stop "$FORCE" "replace the running sim" || exit 1

RUN_ID="${SPOT_RUN_ID:-$(spot_run_id)}"
[[ "$RUN_ID" =~ ^[A-Za-z0-9._-]{1,64}$ ]] || { log "FATAL: bad SPOT_RUN_ID '$RUN_ID' (A-Z a-z 0-9 . _ -, <= 64)"; exit 2; }
SIM_LOG=""
if [ "$BACKGROUND" = 1 ]; then
  # Create (and write-check) the per-run log BEFORE touching the running sim.
  SIM_LOG="$(spot_new_log sim "$RUN_ID")" || exit 1
  log "run_id=$RUN_ID log=$SIM_LOG"
fi


for d in kit cache computecache logs config data pkg; do
  [ -d "$CACHE_ROOT/$d" ] || sudo mkdir -p "$CACHE_ROOT/$d"
done
if [ "$(stat -c %u "$CACHE_ROOT/kit")" != "$ISAAC_UID" ]; then
  sudo chown -R "$ISAAC_UID:$ISAAC_UID" "$CACHE_ROOT"
fi

# Camera frames are 921 KB (640x480 rgb8), i.e. ~15 UDP datagrams each. With
# the kernel's 208 KB default receive buffer, best-effort subscribers
# (ros2 topic hz, sensor-data QoS, likely the HUD) lose whole frames when a
# fragment overflows: measured 8.0 Hz best-effort vs 10.0 Hz reliable, and
# 10.1 Hz best-effort after this change. Fast DDS sizes its sockets from the
# OS default, so the default (not just the max) must rise. Host-global,
# runtime-only (reset on reboot), and only ever raised, never lowered.
DDS_RMEM=8388608
for key in net.core.rmem_max net.core.rmem_default; do
  cur="$(sysctl -n "$key" 2>/dev/null || echo 0)"
  if [ "$cur" -lt "$DDS_RMEM" ]; then
    sudo sysctl -q -w "$key=$DDS_RMEM" || log "WARN: could not raise $key (camera frames may drop for best-effort subscribers)"
  fi
done

GUI_DOCKER_ARGS=()
if [ "$GUI" = 1 ]; then
  setup_gui
  # Only this display's socket, read-only (connecting needs no write access
  # to the mount); the access list checked in setup_gui is the real control.
  GUI_DOCKER_ARGS=(-e "DISPLAY=$GUI_DISPLAY" -v "$X_SOCKET:$X_SOCKET:ro")
  # argparse keeps the last occurrence, so these win over any earlier value.
  APP_ARGS+=(--headless false --ui "$UI_MODE")
fi

# DDS isolation (T25, host/sim_lib.sh spot_dds_args): ROS_DOMAIN_ID
# ${ROS_DOMAIN_ID:-42}, a loopback-only Fast DDS profile, and
# UDPv4 (Fast DDS's shared-memory transport silently drops data between
# containers -- discovery works but no samples arrive; verified 2026-09-27).
# NVIDIA_DRIVER_CAPABILITIES=all (graphics + display + compute) is what lets
# the Vulkan swapchain present to X in --gui mode; harmless headless.
# --no-healthcheck: the image's HEALTHCHECK greps the Kit log for "AppReady",
# which this standalone app never prints, so `docker ps` showed a running,
# ready sim as "(unhealthy)" (presenter dry run 2026-09-27). Readiness is
# FACADE-READY run_id=... (bash run_sim.sh --status), never the health flag.
# Privileges (T28): the image's own user, isaac-sim (uid 1234, checked in the
# image config), so the container holds no effective capabilities. It does
# not get --cap-drop=ALL, no-new-privileges or a read-only root like the
# other containers: the GPU path (the NVIDIA runtime hook, Vulkan presenting
# to X under --gui) has not been verified with them on the sim host, and Kit
# writes into its home outside the cache mounts. Mounts: only sim/, read-only
# (the app writes nothing there), and the Kit caches it must write.
# AWS_EC2_METADATA_DISABLED=true (T26): the sim calls no AWS API. It stops
# SDK clients only, not a raw request to 169.254.169.254; the documented
# instance-role policy and host/imds_guard.sh (uid 1234 has no IMDS access)
# are the real control.
spot_dds_args || exit 2
spot_pinned_image ISAAC_IMAGE || exit 2
spot_require_imds_guard "start the sim" || exit 2

# Every check above runs before the running sim is touched. A previous run
# (crashed client, timed-out smoke test) can leave a container named
# spot-sim behind; `--name spot-sim` would then fail with a name conflict,
# so clear any leftover now.
docker rm -f spot-sim >/dev/null 2>&1 || true
DOCKER_MODE=(--rm)
[ "$BACKGROUND" = 1 ] && DOCKER_MODE=(-d --rm)
SIM_DOCKER=(docker run "${DOCKER_MODE[@]}" --name spot-sim --no-healthcheck --gpus all --network=host \
  -e ACCEPT_EULA=Y "${PRIVACY_ARGS[@]}" \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e AWS_EC2_METADATA_DISABLED=true \
  "${SPOT_DDS_ARGS[@]}" \
  -e SPOT_PROFILE_AT="${SPOT_PROFILE_AT:-}" \
  -e SPOT_RUN_ID="$RUN_ID" \
  ${GUI_DOCKER_ARGS[@]+"${GUI_DOCKER_ARGS[@]}"} \
  -v "$CACHE_ROOT/kit:/isaac-sim/kit/cache:rw" \
  -v "$CACHE_ROOT/cache:/isaac-sim/.cache:rw" \
  -v "$CACHE_ROOT/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$CACHE_ROOT/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$CACHE_ROOT/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$CACHE_ROOT/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$CACHE_ROOT/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
  -v "$DEMO_DIR/sim:/workspace/sim:ro" \
  --entrypoint /isaac-sim/python.sh \
  "$ISAAC_IMAGE" /workspace/sim/isaac_app.py ${APP_ARGS[@]+"${APP_ARGS[@]}"})

if [ "$BACKGROUND" = 0 ]; then
  log "run_id=$RUN_ID (foreground; stop: bash run_sim.sh --stop)"
  # A foreground launch gets the same listener check: a watcher waits for this
  # run's READY line, then checks (and stops the sim on a listener of its own);
  # it gives up if the sim exits first (T44).
  LAUNCHER=$$
  ( if spot_wait_sim_ready "$RUN_ID" "${SPOT_SIM_READY_TIMEOUT:-300}" /dev/null "pid:$LAUNCHER" >/dev/null 2>&1; then
      check_listeners || log "FATAL: listener check failed after READY (see above)"
    fi ) < /dev/null &
  exec "${SIM_DOCKER[@]}"
fi

# --background: detached container (its lifetime no longer depends on this
# shell or SSM session), output streamed to the per-run log by a follower in
# its own session, then wait for THIS run's readiness line.
CID="$("${SIM_DOCKER[@]}")" || { log "FATAL: docker run failed"; exit 1; }
setsid nohup docker logs -f "$CID" >> "$SIM_LOG" 2>&1 < /dev/null &
log "container ${CID:0:12} started; waiting for FACADE-READY run_id=$RUN_ID (up to ${SPOT_SIM_READY_TIMEOUT:-300} s; ~40 s warm)"
if ready="$(spot_wait_sim_ready "$RUN_ID" "${SPOT_SIM_READY_TIMEOUT:-300}" "$SIM_LOG" "cid:$CID")"; then
  check_listeners || { log "FATAL: listener check failed after READY (see above)"; exit 1; }
  echo "run_sim: READY run_id=$RUN_ID log=$SIM_LOG"
  echo "run_sim: $ready"
  exit 0
fi
log "not ready. Container still running? bash run_sim.sh --status / docker logs -f spot-sim; stop: bash run_sim.sh --stop"
exit 1
