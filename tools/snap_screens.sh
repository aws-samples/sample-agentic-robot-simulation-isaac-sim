#!/usr/bin/env bash
# snap_screens.sh -- capture the DCV desktop and the HUD as PNGs for one moment.
#
# What it is:
#   The evidence helper behind docs/evidence/ (presenter dry run 2026-09-27).
#   Run it ON the sim host as ubuntu (or root: it switches to ubuntu for X).
#     * sim-<NAME>.png  the Isaac Sim window on ubuntu's desktop session (the
#                       one the NICE DCV console shows; run_sim.sh --gui puts
#                       the Isaac window there), via ImageMagick `import`. Only
#                       that window: a terminal, a notification or another
#                       application on the desktop stays out of the evidence
#                       (T36)
#     * hud-<NAME>.png  the HUD (http://127.0.0.1:8080, host/run_hud.sh) in
#                       headless Chromium at 1920x1080, via a long-lived
#                       Playwright container "spot-shot" (started on first use)
#   Both captures start at the same time, but the HUD one lands ~3 s later
#   (npx + Chromium launch, then SPOT_HUD_SETTLE_MS, default 1500, for the
#   page's WebSocket to fill in): in the 2026-09-27 dry run a HUD shot
#   triggered 0.3 s after interior waypoint 1 already showed the guard
#   alarm that fired 2.2 s after the waypoint. Label HUD shots by what they show, not the trigger.
#
# Usage: bash tools/snap_screens.sh NAME        (NAME: A-Z a-z 0-9 . _ -)
#        bash tools/snap_screens.sh --stop      (remove the spot-shot container)
# Needs: imagemagick (import), xwininfo (x11-utils), the ubuntu desktop on X
#        (run_sim.sh --gui sets it up), docker image mcr.microsoft.com/playwright:v1.49.0-noble
#        (pinned by digest: SPOT_PLAYWRIGHT_IMAGE in host/sim_lib.sh).
# Env:   SPOT_SCREENS_DIR (default /opt/spot-demo/logs/screens)
#        SPOT_GUI_USER (default ubuntu), SPOT_GUI_DISPLAY (default :0)
#        SPOT_HUD_URL (default http://127.0.0.1:8080/)
#        SPOT_SNAP_HUD=0   desktop only (no HUD shot, no Playwright container)
#        SPOT_SNAP_WINDOW  the window to capture: the first whose name starts
#                          with it (default "Isaac Sim"); none found = fail
#        SPOT_SNAP_ROOT=1  capture the whole desktop instead; check the shot
#                          for anything private before you keep it
#        SPOT_SNAP_WIDTH=N also write sim-<NAME>-<N>w.png, the desktop shot
#                          scaled to N px wide (small enough to pull over SSM
#                          quickly; the full-size PNG is kept)
# Exit:  0 all requested captures written, 1 one failed (the rest are kept).
set -uo pipefail

OUT="${SPOT_SCREENS_DIR:-/opt/spot-demo/logs/screens}"
GUI_USER="${SPOT_GUI_USER:-ubuntu}"
DISP="${SPOT_GUI_DISPLAY:-:0}"
URL="${SPOT_HUD_URL:-http://127.0.0.1:8080/}"
SETTLE_MS="${SPOT_HUD_SETTLE_MS:-1500}"
SNAP_HUD="${SPOT_SNAP_HUD:-1}"
SNAP_WIDTH="${SPOT_SNAP_WIDTH:-}"
SNAP_WINDOW="${SPOT_SNAP_WINDOW:-Isaac Sim}"
SNAP_ROOT="${SPOT_SNAP_ROOT:-0}"
[[ -z "$SNAP_WIDTH" || "$SNAP_WIDTH" =~ ^[0-9]{2,4}$ ]] || { echo "snap_screens: bad SPOT_SNAP_WIDTH '$SNAP_WIDTH'" >&2; exit 2; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=host/sim_lib.sh
source "$HERE/../host/sim_lib.sh"
PW_IMAGE="$SPOT_PLAYWRIGHT_IMAGE"   # playwright:v1.49.0-noble, pinned by digest (T31)
SHOT=spot-shot

if [ "${1:-}" = --stop ]; then docker rm -f "$SHOT" >/dev/null 2>&1; echo "snap_screens: $SHOT removed"; exit 0; fi
NAME="${1:?usage: snap_screens.sh NAME | --stop}"
[[ "$NAME" =~ ^[A-Za-z0-9._-]{1,64}$ ]] || { echo "snap_screens: bad NAME '$NAME'" >&2; exit 2; }
mkdir -p "$OUT"
# Each capture is written into a private directory first and then moved into
# $OUT with `mv -n`, which renames without following or replacing anything
# already there: a name a container planted in the shared screens directory
# (a symlink, for example) is left alone and the capture fails (T28, T36).
PRIV=""
MV_T=(); mv --version >/dev/null 2>&1 && MV_T=(-T)   # GNU mv: never move *into* a planted directory link
publish() {  # SRC DEST: move SRC into place unless DEST already exists, as anything
  if [ -e "$2" ] || [ -L "$2" ]; then echo "snap_screens: $2 already exists; not overwritten" >&2; return 1; fi
  mv -n ${MV_T[@]+"${MV_T[@]}"} "$1" "$2" && [ ! -e "$1" ] \
    || { echo "snap_screens: $2 already exists; not overwritten" >&2; return 1; }
}

# X capture as the desktop owner, with that Xorg's -auth file: the named
# window only, unless SPOT_SNAP_ROOT=1 asks for the whole desktop (T36).
snap_x() {
  local pid xa target
  pid="$(pgrep -u "$GUI_USER" -x Xorg | head -1)"
  [ -n "$pid" ] || { echo "snap_screens: no Xorg for $GUI_USER (run_sim.sh --gui sets it up)" >&2; return 1; }
  xa="$(tr '\0' '\n' < "/proc/$pid/cmdline" | grep -A1 -x -- '-auth' | tail -1)"
  as_gui() {
    if [ "$(id -un)" = "$GUI_USER" ]; then
      DISPLAY="$DISP" XAUTHORITY="$xa" "$@"
    else
      sudo -u "$GUI_USER" env DISPLAY="$DISP" XAUTHORITY="$xa" "$@"
    fi
  }
  target=root
  if [ "$SNAP_ROOT" != 1 ]; then
    target="$(as_gui xwininfo -root -tree | awk -v n="\"$SNAP_WINDOW" 'index($0, n) { print $1; exit }')"
    if [[ ! "$target" =~ ^0x[0-9a-f]+$ ]]; then
      echo "snap_screens: no window named '$SNAP_WINDOW...' on $DISP; not capturing the whole desktop" \
           "(SPOT_SNAP_WINDOW names another window; SPOT_SNAP_ROOT=1 takes the whole desktop)" >&2
      return 1
    fi
  fi
  PRIV="$(as_gui mktemp -d)" || return 1
  as_gui import -silent -window "$target" "$PRIV/sim.png" || return 1
  publish "$PRIV/sim.png" "$OUT/sim-$NAME.png"
}

# HUD capture: reuse one Playwright container (npx caches the CLI inside it).
snap_hud() {
  if [ "$(docker inspect -f '{{.State.Running}}' "$SHOT" 2>/dev/null)" != true ]; then
    docker rm -f "$SHOT" >/dev/null 2>&1 || true
    # A one-shot like the presenter consoles (T28): its own non-root uid, not
    # ubuntu's (uid 1000 would reach ubuntu's X display through the abstract
    # socket), no capabilities, no-new-privileges, and the screens dir as
    # its only mount, written through the dir's group. The root filesystem
    # stays writable: npx installs and runs the Playwright CLI from $HOME.
    # No ROS in it; it still gets the DDS isolation every container does (T25).
    local gid
    gid="$(spot_shared_dir_gid "$OUT")" || return 1
    spot_container_args oneshot --writable-root || return 1
    docker run -d --name "$SHOT" --network=host "${SPOT_ARGS[@]}" --group-add "$gid" \
      -v "$OUT:/out" "$PW_IMAGE" sleep infinity >/dev/null || return 1
  fi
  docker exec "$SHOT" npx -y playwright@1.49.0 screenshot --browser chromium \
    --viewport-size "1920, 1080" --wait-for-timeout "$SETTLE_MS" "$URL" "/out/hud-$NAME.png" >/dev/null
}

rc=0
hud_pid=""
if [ "$SNAP_HUD" != 0 ]; then snap_hud & hud_pid=$!; fi
snap_x || { echo "snap_screens: X capture failed" >&2; rc=1; }
if [ -n "$SNAP_WIDTH" ] && [ -n "$PRIV" ] && [ -s "$OUT/sim-$NAME.png" ]; then
  { convert "$OUT/sim-$NAME.png" -resize "${SNAP_WIDTH}x" "$PRIV/sim-w.png" \
      && publish "$PRIV/sim-w.png" "$OUT/sim-$NAME-${SNAP_WIDTH}w.png"; } \
    || { echo "snap_screens: resize failed" >&2; rc=1; }
fi
[ -z "$PRIV" ] || rm -rf "$PRIV"
if [ -n "$hud_pid" ]; then wait "$hud_pid" || { echo "snap_screens: HUD capture failed" >&2; rc=1; }; fi
for f in "$OUT/sim-$NAME.png" "$OUT/sim-$NAME-${SNAP_WIDTH:-x}w.png" "$OUT/hud-$NAME.png"; do
  [ -s "$f" ] && echo "snap_screens: $(date -u +%H:%M:%S.%3N) $f ($(stat -c %s "$f") bytes)"
done
exit "$rc"
