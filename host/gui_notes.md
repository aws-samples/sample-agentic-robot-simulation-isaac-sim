# GUI mode on the NICE DCV desktop — operator notes

How to put the live Isaac Sim window (presenter-camera view of Spot) on the
DCV desktop of the sim host (us-west-2). Everything below
was exercised on the host on 2026-09-27; measured numbers are from those runs
(the presentation-mode section: second session that day, commit 70a2eaf +
working tree).

## TL;DR

```bash
# Mac: open the tunnel, then browse https://localhost:8443 (user ubuntu)
SPOT_SIM_INSTANCE=<instance-id> scripts/connect-dcv.sh

# Host (DCV terminal, or via SSM as root):
cd /opt/spot-demo/host && export ACCEPT_EULA=Y        # your acceptance of the NVIDIA licence
bash run_sim.sh --prepare-desktop                    # once per host (opt-in; see below)
bash run_sim.sh --background --gui   # returns once THIS launch prints
                                     # FACADE-READY (~40 s warm, ~3 min first time; SPOT_SIM_READY_TIMEOUT)
                                     # presentation mode: fullscreen viewport, nothing else on screen
bash run_sim.sh --background --editor   # the same with the full Isaac Sim editor (debugging)
bash set_view.sh interior               # presenter camera: auto (default) | chase | interior | overhead | door
bash set_view.sh --ui editor            # editor panels back without a restart (--ui presentation: back again)
bash run_sim.sh --stop                               # stop (only one sim at a time; refuses while the agent runs)
# Log: /opt/spot-demo/logs/sim-<run_id>.log (sim-latest.log). Never redirect to
# /tmp: a root-owned leftover there cannot be reopened by ubuntu
# (fs.protected_regular=2) and grepping it finds a STALE FACADE-READY line.
```

The window titled **"Isaac Sim Python 6.1.0"** appears on the DCV desktop and,
about 5 s after FACADE-READY, goes fullscreen showing only its viewport:
`/World/ChaseCamera`, driven by the presenter-camera director (see
[Presenter camera](#presenter-camera-v2)). All ROS topics and services are
identical to headless mode, so the agent, mission and HUD do not care which
mode is running.

## What `run_sim.sh --prepare-desktop` and `--gui` do (and why)

Since the hardening pass the host's desktop security settings change only in
`--prepare-desktop`, which you run once per host: it prints every change and
records the originals (root-owned, in `/var/lib/spot-demo/desktop`) before
applying them, and `--restore-desktop` puts them back. `--gui` changes no
security setting: it stops with a hint if the console is still at the login
screen, refuses to start while DCV listens off loopback
(`host/configure_dcv.sh check`), and refuses an X server whose access list
lets in anyone but `ubuntu` and the container's uid. Steps 2 and 3 below are
what `--prepare-desktop` does; steps 1, 4 and 5 are `--gui`.

1. **Finds the ubuntu desktop.** It looks for an X socket in `/tmp/.X11-unix`
   owned by `ubuntu` (GDM starts Xorg with `-displayfd`, so the socket is the
   only reliable place to read the display number from). Observed: `:0`.
   Override with `SPOT_GUI_DISPLAY=:N`.
2. **If only the GDM login screen is up, it enables autologin once.** It sets
   `AutomaticLoginEnable=true` and `AutomaticLogin=ubuntu` under `[daemon]` in
   `/etc/gdm3/custom.conf` (the original is kept at
   `/var/lib/spot-demo/desktop`, and restored by `--restore-desktop`), then runs `systemctl restart gdm3`,
   waits for ubuntu's X server, and runs `systemctl restart dcvserver`. The
   restart is needed because a DCV *console* session mirrors the physical
   console. That console shows a desktop only when a user is logged in on it;
   without one, the Isaac window has no X server to open on. Access is still
   gated by the DCV login (ubuntu password, fetched from Secrets Manager by
   `scripts/connect-dcv.sh`) over an SSM tunnel with no inbound ports.
   After the restart, `dcvagent --session-id console` runs as ubuntu. Anyone
   connected to DCV at that moment is dropped and must reconnect.
3. **Tunes the desktop for a demo.** It turns off the GNOME screen lock, the
   idle blank and dimming, the first-login wizard, notification banners
   (DCV's "Printer added" popup covered the viewport), and crash-report popups
   (`com.ubuntu.update-notifier show-apport-crashes false`), and it closes any
   crash dialog already on screen. In presentation mode it also sets the Ubuntu
   dock to auto-hide (`org.gnome.shell.extensions.dash-to-dock` `dock-fixed
   false`, `autohide true`, `intellihide true`; `--editor` resets them); the
   fullscreen Kit window covers the dock and top bar anyway. If the DCV console
   is smaller than `SPOT_GUI_LAYOUT` (default 1920x1080: no viewer attached,
   1024x768) it runs `dcv set-display-layout --session console 1920x1080+0+0`
   first, as a 1080p viewer would (not exercised in the 2026-09-27 runs: the
   layout was already 1920x1080). Only when `dcv list-connections console`
   says no client is connected: an attached viewer's layout is its own window
   and is never overridden (a forced 1920x1080 would crop or scale the render
   in a smaller viewer). Expect that dialog: the first GDM restart
   leaves `/var/crash/_usr_lib_x86_64-linux-gnu_dcv_dcvagent.127.crash`,
   because the greeter's dcvagent dies with its X server. It is harmless and
   the file is left in place.
4. **Grants X access to the container user only.** It runs
   `xhost +si:localuser:#1234`. The container runs as uid 1234 (isaac-sim),
   which has no host account, hence the numeric form. The grant is checked
   against unix-socket peer credentials, so nothing is opened on the network.
   Never use a bare `xhost +`. The grant lives inside the X server, so it is
   repeated on every launch (it is lost if X restarts).
5. **Runs the container** with `-e DISPLAY=:0`,
   that display's socket only, mounted read-only, and `NVIDIA_DRIVER_CAPABILITIES=all`
   (the display capability lets the Vulkan swapchain present to X). It then
   appends `--headless false --ui presentation` (`--editor`: `--ui editor`)
   to the `isaac_app.py` arguments.
6. `isaac_app.py --headless false` points the active viewport at
   `/World/ChaseCamera`, pins its render resolution to 1280x720 (`--gui-res`)
   so the render cost does not depend on the window or DCV client size, and
   applies `--ui` (next section).

## Presentation mode (V1)

`--ui presentation` (the `--gui` default) runs Kit's own fullscreen action,
the one behind Window > Fullscreen Mode: `omni.kit.ui.actions` /
`toggle_fullscreen` (source in the container:
`extscache/omni.kit.ui.actions-1.0.5+00c488ae/omni/kit/ui/actions/actions.py`).
It saves the workspace, calls `omni.appwindow` `set_fullscreen(True)` and sets
`/app/window/hideUi`, which makes the viewport window fill the main window
without title bar and hides its menubar (renderer / camera / "Stage Lights"
buttons) and axis gizmo (`omni.kit.viewport.window` `window.py __hide_ui`).
The Stage, Property, Render Settings, Content browser, Console, main menu,
left toolbar and status bar are not drawn. The one overlay hideUi leaves, the
"m" stage-units badge of `omni.usd.metrics.assembler.ui`, is switched off
through `/app/viewport/<id>/hud/metrics/assembler/visible`. A fullscreen X
window covers the GNOME top bar and the dock. Measured (2026-09-27, 6
launches): `UI mode=presentation via=action`, then `UI check
mode=presentation fullscreen=True hideUi=True window=1920x1080` about 5 s
later; every desktop screenshot after that is the render only
(`docs/evidence/v5/sim-after-*.png`; before: `sim-before-*.png`).

Kit binds Esc to "cancel fullscreen" on the viewport when it goes fullscreen
(`update_cancel_hotkey`, setting `/exts/omni.kit.ui.actions/ui_hotkey_escape`,
default true): one stray Esc in the DCV viewer put every editor panel back on
screen (host 2026-09-27, `xdotool key Escape`: 1920x1080 fullscreen ->
1440x863 editor). `isaac_app.py` sets it false before every layout change
(`sensors.KIT_UI_SETTINGS`); re-measured: two Esc presses, window stays
1920x1080 fullscreen. F11 still toggles. The fullscreen window follows a DCV
layout change (a viewer connecting later): `set-display-layout 1600x900`
-> Kit window 1600x896 fullscreen, back to 1920x1080 likewise. A terminal on
the DCV desktop is covered by it: use the SSM shells for the presenter
console.

Getting the editor back:

- at launch: `bash run_sim.sh --background --editor` (stock layout, window
  maximized);
- at runtime, no restart: `bash set_view.sh --ui editor` restores the saved
  workspace and leaves fullscreen (measured: panels back, window 1440x863,
  then maximized); `bash set_view.sh --ui presentation` returns to
  fullscreen (measured `fullscreen=True window=1920x1080` 5 s later).

## Presenter camera (V2)

`sim/sensors.py` `CameraDirector` (pure Python, `tests/test_sensors.py`)
moves the one viewport camera every render tick. Pick a view with
`bash set_view.sh NAME` (publishes `/demo_spot/payload/view`
`{"camera": NAME}`; the sim prints `VIEW camera=...`); `reset_spot.sh` puts
it back on `auto`. A view change is a hard cut.

| View | Shot |
|---|---|
| `auto` (default) | Outside: low 3/4 rear chase, lens 1.15 m high, 3.4 m from Spot, 25 deg round to its right, aiming 1 m ahead (tank and door in frame). Walking out of the tank (exit, retreat) the lens goes ahead of Spot (it walks toward the camera), kept 0.5 m outside the shell. Lens kept 0.6 m from the barrels and the pipe rack with a clear line to Spot (`build_scene` `camera_keepouts`), else on Spot's left. At the wall crossing ONE cut to the interior corner camera, and one back on the way out. |
| `chase` | The chase everywhere; inside at 1.5 m, 2.4 m back, kept 0.5 m inside the shell. |
| `interior` | Fixed lens inside the shell on the door side (5.3 m from the centre, 50 deg right of the door seen from the centre, 2.6 m high), panning and zooming (28-70 deg) on Spot: curved wall, ladder on the far wall, pipes, sludge and the headlight pool stay in frame. |
| `overhead` | Under the roof (3.85 m), straight down, gliding with Spot; door side on the right of the image. |
| `door` | Fixed lens outside beside the door (2.3 m out, 2.0 m to the side, 1.25 m high), aiming at Spot near the door (probe close-up). |

Smoothing is dt-exact low-pass (lens 0.8 s, aim 0.5 s, zoom 1.2 s), and the
lens orbits round Spot (capped at 0.6 rad/s) on the side it is allowed on,
never through the shell. Unit-tested on the scripted storyline path (spawn,
6 s probe dwell, 5 waypoints, back out, sit) and the retreat: auto cuts
exactly 3 times (first frame, in, out), moves < 0.12 m per 0.1 s otherwise,
keeps Spot within half the half-FOV of the frame centre, never lets a pipe
hide it and lets a door jamb hide it for at most 0.3 s while it walks out.

Lighting (`sim/build_scene.py`, dressing only; nothing the mission uses
moved): the tank now has a fixed cone roof (visual only, no collider), so the
interior is a dark confined space instead of half sunlit (`09` was
near-black). Inside: daylight spill through the door (rect light), a roof
manway above the ladder (rect light) and 4 dim cool fill lights under the
roof, all with `visibleInPrimaryRay = false` so the emitters themselves are
not drawn (the first try showed the door rect light as a white panel).
Spot's payload lights (`sim/sensors.py`): a 45 deg flood (700000, was 60000)
plus a 14 deg beam (2500000) that throws a hot spot on the wall or ladder
ahead. Values were set by eye on the host in 3 iterations
(`docs/evidence/v5/sim-after-*.png`). The body cameras see the same light
(`sim-after-hud-camera-mid.png`: HUD camera tile with Spot mid-tank).

Leadership rehearsal 2026-09-28 (`docs/evidence/v5/lr-*.png`), two more
lighting changes and one reset fix:

- **Payload lights fade with distance from the shell** (`PayloadLights`,
  `payload_light_scale`): full inside and up to 0.8 m outside (the door-probe
  seat is ~0.6 m out), off from 1.6 m out (spawn, and where Spot sits after an
  exit or a retreat). Outside in the sun they only blew the HUD camera tile
  out to white and put a white hot spot on the ground.
- **Entry work light** (`build_scene.py` `WORK_LIGHT_*`): a shaped sphere
  light on a (hidden) mast 2.6 m out and 1.6 m to the side, aimed at the door
  mouth. The door face is on the shadow side of the tank, so the probe beat
  (Spot at the door for ~2 min) and the sit after a retreat were near-black
  on a projector. Render cost within noise (37-40 ms per render, RTF 0.99-1.01).
- **Reset heading**: `reset_spot.sh` teleports Spot to spawn facing the tank,
  but the stance hold latched the PRE-teleport heading (the 50 Hz pose read
  had not run yet) and turned Spot back to where the previous run left it,
  i.e. facing away from the tank after an exit or a retreat (180 -> ~10 deg
  within 5 s). `isaac_app.py` now drops the stale pose on a policy restart;
  measured after the fix: 2/2 resets from a Spot facing away held 179-180 deg
  for 10 s.

## Simulated battery (V5)

`--battery-drain` (`sim/battery.py`): `demo` (default) 25 %/h, floor 30 %
(a 2 h session ends at 50 %; the HUD chip turns amber below 30 %, so it stays
green); `real` 100 % in 90 min like a real Spot, floor 5 %; `off` 100 %.
`reset_spot.sh` recharges it (`/demo_spot/payload/battery/reset`); measured
with `real`: 99.94 % a few seconds after a reset at sim time ~35 s (99.35 %
without it). The v1 fake drain was 95 %/h (amber after ~44 min, red after
~54 min).

## Screen size and the Isaac window

Presentation mode needs none of this: the window is fullscreen on whatever
the console layout is, and `run_sim.sh --gui` raises a no-viewer 1024x768
layout to 1920x1080 first. The render resolution stays pinned at 1280x720 and
is scaled to the screen. For `--editor`: with no DCV viewer connected the
console desktop is 1024x768 and the Isaac window opens at 938x678; the app
maximizes it. A viewer normally resizes the desktop to its own window; to set
it by hand (as the 2026-09-27 dry run did):

```bash
sudo dcv set-display-layout --session console 1920x1080+0+0
XA=$(tr '\0' '\n' < /proc/$(pgrep -u ubuntu -x Xorg)/cmdline | grep -A1 -x -- -auth | tail -1)
W=$(sudo -u ubuntu DISPLAY=:0 XAUTHORITY=$XA xdotool search --name '^Isaac Sim Python' | tail -1)
sudo -u ubuntu DISPLAY=:0 XAUTHORITY=$XA xdotool windowactivate --sync $W key --clearmodifiers super+Up
```

(or double-click the title bar in the viewer). Kit remembers the maximized
size (1854x1011) for the next launch. The viewport's render resolution stays
pinned at 1280x720, so maximizing did not change RTF. Evidence screenshots:
`bash /opt/spot-demo/tools/snap_screens.sh NAME` (needs `imagemagick`, installed
on the host 2026-09-27, and the Playwright image for the HUD shot).

## Verify without seeing the screen

```bash
XA=$(tr '\0' '\n' < /proc/$(pgrep -u ubuntu -x Xorg)/cmdline | grep -A1 -x -- -auth | tail -1)
sudo -u ubuntu DISPLAY=:0 XAUTHORITY=$XA xwininfo -root -tree | grep -i isaac
#   "Isaac Sim Python 6.1.0": ("IsaacSim" "Isaac Sim Python 6.1.0")  938x678+14+49
sudo -u ubuntu DISPLAY=:0 XAUTHORITY=$XA xhost        # expect SI:localuser:#1234
docker logs spot-sim 2>&1 | grep -E '^VIEWPORT|^CAMERAS'   # VIEWPORT gui: camera=/World/ChaseCamera ... view=auto
docker logs spot-sim 2>&1 | grep -E '^UI|^VIEW|^BATTERY'   # UI check mode=presentation fullscreen=True hideUi=True window=1920x1080
sudo dcv describe-session console                      # owner ubuntu, display layout
# Screenshot: sudo -u ubuntu DISPLAY=:0 XAUTHORITY=$XA xwd -root -silent > /tmp/s.xwd
```

## Performance (L4, default 2 cameras, 10 Hz render)

| Mode | PERF rtf | Notes |
|---|---|---|
| headless | 0.95-1.03 | the hidden viewport is disabled so the cameras fit the budget |
| GUI, `--render-hz 10` (default) | 0.98-1.02 | the viewport runs at 10 fps |
| GUI, default, window maximized at 1920x1080 (dry run 2026-09-27) | 0.95-1.04, median 1.00 (355 samples, none < 0.95) | two launches, three agent sessions incl. walks and a retreat; render 26-27 ms median, 41 ms max; no DCV viewer attached |
| GUI, `--render-hz 20` | 0.78-0.94 | smoother, but **below real time**; do not use for the demo |
| before this change, same host and runs (2026-09-27 evening): v1 code, editor window 1920x1011, clean mission + staged-spike retreat | 0.97-1.03, mean 1.00 (40 samples) | render 27.2 ms mean, 39.2 max |
| GUI presentation mode (fullscreen, roof, interior lights, 2 nose lights), same runs | 0.97-1.02, mean 1.00 (38 samples, none < 0.95) | render 38.8 ms mean, 49.3 max; HUD running; no DCV viewer attached |
| GUI presentation mode, one launch, 3 missions + view and UI switching | 0.95-1.04, mean 1.00 (130 samples) | the one 0.95 is the 2nd sample after launch; all others >= 0.97 |
| GUI presentation mode, leadership rehearsal 2026-09-28, final build (work light, light fade): 5 agent reps with spikes + 2 clean runs + host checks, screenshots on 3 runs | 0.95-1.06, median 1.00 (394 samples, none < 0.95) | render 36 ms median, 50 ms max; DCV server running, no viewer attached |
| headless, 2 cameras, after this change (clean mission) | 0.94-1.05, mean 1.00 (28 samples) | the one 0.94 is the `reset_spot.sh` teleport; render 34 ms mean; frontleft 10.4-10.7 Hz |

Each GUI render costs about 25-35 ms (v1 scene) or 30-50 ms (roofed tank, 6
interior lights, 2 nose lights) on top of about 460 ms/s of physics.
`--render-hz 15` is roughly the ceiling (not measured). `--cameras none`
lowers the per-render cost if the HUD video is not needed. Cameras always
publish at `--camera-hz` (10), whatever the render rate.

## Troubleshooting

- **"FATAL: xhost grant ... failed"**: ubuntu's X server is gone (logged out,
  or GDM restarted). Re-run `run_sim.sh --gui`; it re-detects the display and
  grants access again.
- **The Kit window never appears / "cannot open display"**: check
  `ls -la /tmp/.X11-unix/`. The `X0` socket must be owned by `ubuntu`. If it is
  owned by `gdm`, the console is at the greeter: autologin is off or failed
  (`grep -A2 '^\[daemon\]' /etc/gdm3/custom.conf`).
- **DCV shows a black screen or the login screen after the first `--gui`**:
  reconnect. If it persists, run `sudo systemctl restart dcvserver`.
- **OmniHub warnings in the first ~10 s** ("Hub failed to launch"): GUI Kit
  tries to start the Omniverse Hub cache, which cannot run in the container.
  They stop by themselves and do not matter.
- **Undo autologin and the other desktop changes**: `bash run_sim.sh --restore-desktop`, then `sudo systemctl restart gdm3` (logs `ubuntu` out and drops DCV viewers).
- **Editor panels still visible / not fullscreen**: check `docker logs spot-sim
  | grep '^UI'`. `UI mode=... failed` means Kit refused (the sim keeps running
  in the editor layout); retry with `bash set_view.sh --ui presentation`.
- **Camera stuck or wrong view**: `bash set_view.sh auto` (or
  `reset_spot.sh`). `CHASE-CAMERA disabled after error` in the log means the
  director raised once; the viewport then stays where it was until a restart.

## Camera transport note (applies to both modes)

`run_sim.sh` raises `net.core.rmem_default` and `rmem_max` to 8 MiB. It only
ever raises them, the change is runtime-only, and it is reset on reboot. Each
640x480 rgb8 frame is 921 KB, about 15 UDP datagrams. With the kernel's
208 KB default receive buffer, **best-effort** subscribers lose whole frames.
Measured on `camera/frontleft/image`:

- best-effort: 8.0 Hz before the change, 10.1 Hz after
- reliable: 10.0 Hz either way

Consumers should subscribe RELIABLE, or best-effort after this sysctl.
