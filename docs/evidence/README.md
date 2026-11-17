# Evidence: recorded runs in simulation

Every reading in these files is simulated. The permit details in the transcripts,
event logs and audit exports (permit `CSE-0427`, attendant "Alex Rivera") are
fictional placeholders from the demo script, not real people or permits.

**Before you add evidence** (threat model T36, T37), check each file against this
list, and leave it out if any item fails:

1. No real names of people. Use role labels such as "attendant ATT-01".
2. No AWS account, instance, security-group or resource IDs, ARNs, IP addresses,
   host names or internal URLs, in text or visible in a screenshot.
3. No secrets, tokens or passwords, and no screenshot of a terminal that printed one.
4. Screenshots show only the Isaac window or the HUD, not the whole desktop
   (other windows, notifications or a taskbar can show what you did not mean to share).
   `tools/snap_screens.sh` captures only the Isaac Sim window unless you set
   `SPOT_SNAP_ROOT=1`; check any whole-desktop shot before you keep it.
5. Fetched logs are copied from `~/spot-demo-logs/` deliberately, file by file, never
   with `git add -A`.

# Presenter dry run, 2026-09-27 (GUI mode, agent REPL)

What was run: the RUNBOOK presenter path, on the sim host,
executed over SSM as `ubuntu`. Components: the sim
started with `run_sim.sh --background --gui` on the DCV console desktop (:0,
1920x1080, Isaac window maximized), the HUD from `run_hud.sh`, and the agent
REPL (`run_agent.sh`, Bedrock `us.anthropic.claude-sonnet-5`) under a real
pty (`script -qfc`). Each instruction was sent as one line through a FIFO.
The staged H2S spike was fired by `host/inject_gas.sh` as ubuntu from a
second, concurrent SSM command, 1.0 s after the log showed interior
waypoint 1. No DCV viewer was attached, so the RTF numbers do not include
streaming load.

Sessions (the permit is always CSE-0427/0428, attendant Alex Rivera):

| Session | Code | Path | Result |
|---|---|---|---|
| inc1 | before the seat fix | permit, FAIL, ventilate, re-test **rejected**, extra prompt, PASS, entry, spike | ABORTED as designed; needed the cheat-sheet fallback at beat 4 |
| inc2 | after the fix, fresh sim | cheat-sheet storyline (a)(b)(c) + spike | ABORTED as designed, no extra prompt |
| ok1 | after the fix | permit, FAIL, early re-tests (x2), purge wait, PASS, entry, exit | COMPLETED |

## Measured numbers

The source is the event logs below. Times are wall clock; `t` is sim time.

- Sim start as ubuntu, GUI: READY 35.6 s (2/2 launches, warm caches),
  `FACADE-READY ... run_id=` matched on the current container. HUD up in
  2.7 s. Each reset (`reset_spot.sh --reset-ventilation`) took 2.9 s.
- RTF, GUI (`perf-gui.txt`): 355 five-second PERF samples over two
  launches: min 0.95, median 1.00, max 1.04, none below 0.95; render
  26-27 ms median, 41 ms max. Defaults were kept (`--render-hz 10`, cameras
  frontleft+frontright, 1280x720 viewport). No tuning was needed.
- inc2, turn (a) permit + test: 33.3 s. FAIL on H2S 51.6 ppm (worst at
  0.2 m height, 1.0 m inside). O2 19.9 %, LEL 8.0 %, CO 0.4 ppm pass.
- inc2, turn (b) ventilate + re-test: 60.0 s. Purge 45.5 s to 8.0 %. The
  re-test started 1 ms after the tool call (no re-seat hop) and PASSED:
  H2S 3.6 ppm, LEL 0.7 %, O2 20.8 %, CO 0.8 ppm. Result ENTRY_AUTHORIZED.
- inc2, turn (c) entry + spike: 24.2 s. Door threshold at +1.1 s, waypoint 1
  at +4.1 s. Shell B Enter to guard alarm (H2S 78.5 ppm): 1.23 s. Presenter
  disclosure event to alarm: 34 ms (inc1: 53 ms). Alarm to retreat start:
  14 ms. Retreat took 12.1 s, ending with Spot sitting outside. Permit
  ABORTED. The agent's explanation ends the turn and states that the spike
  was presenter-staged.
- ok1, early re-test: at 2 s into ventilation the test FAILS (H2S 38.9 ppm).
  At 46 s, every gas was within limits at the door (H2S 4.6 ppm) with 8.5 %
  remaining. The mission logged `purge_incomplete` ("Purge incomplete — wait:
  ... entry needs <= 8 %; keep ventilating about 2 s more, then re-test"),
  the agent relayed it, and the permit stayed PRE_ENTRY_TESTING. The next
  re-test at 5.2 % PASSED.
- ok1, entry turn: 38.8 s, 5/5 waypoints. Exit turn ("Walk back out to the
  entry point."): 43.9 s. Result COMPLETED, Spot sitting at (13.7, 0.2).
- Door clearance, body box to jamb (`rh-analyze.txt`): inc2 0.230 m, ok1
  0.276 m, inc1 **0.091 m** (during the bad re-seat, see below). Body z
  inside >= 0.51 m.
- Probe rejections after the fix: 0 in 6 tests (2 first tests, 4 re-tests).
- Every mission/agent/guard event carries both `t` (sim) and `wall_t`
  (unix). The HUD CSV exports have `t`, `wall_t`, `wall_utc` and
  `rx_wall_utc` columns. Exception: presenter `demo_inject` events had
  `t` = wall time, because the script had not yet seen payload/status
  (`t_source`).

## Found and fixed during the dry run

1. **Re-test re-seat hop (inc1).** During the purge wait Spot had stood
   0.147 m from the seat target. The heading hold worked: yaw 175.0 deg
   +-0.1 over 70 s. The seat tolerance is 0.14 m, so the re-test walked a
   1 cm hop. The stop transient carried the body 7 cm outward, and the
   facade rejected the 1.0 m points (1.10 m). The re-seat from 0.2 m away
   swung the heading 175 -> 114 deg, the second probe was rejected too, and
   entry stayed denied until the presenter asked "Check the ventilation
   status and re-test." (re-seated and PASSED in 18 s). Fix:
   `agent/mission.py` `_probe_seated()` skips the pre-test re-seat when the
   probe already works from where Spot stands. Tests:
   `tests/test_mission.py` `TestSeatedInProbeEnvelope` (4). On the host
   afterwards: 0 rejections in 6 tests.
2. **HUD covered its own "Gas readings simulated" footer at 1920x1080**
   whenever the permit was PRE-ENTRY TESTING: the badge wrapped to two
   lines and the gauges ran about 30 px into the footer (see `04`, `06`).
   Fix: `hud/static/style.css` keeps the badge on one line, the left column
   clips, and the gauge padding is trimmed. Checked on the host with a
   layout-check event (`14`). Test:
   `TestStaticAndCamera.test_left_column_cannot_cover_the_footer_disclosures`.
3. **`docker ps` showed the ready sim as "(unhealthy)".** The image's
   HEALTHCHECK waits for a Kit "AppReady" line that this app never prints.
   Fix: `run_sim.sh` adds `--no-healthcheck`. Verified on the second launch.
4. **DCV "Printer added" banner over the viewport** (first screenshot).
   `run_sim.sh --gui` now also sets `org.gnome.desktop.notifications
   show-banners false`. It was applied by hand and verified; the scripted
   line has not been re-run.
5. Tooling: `tools/` is now deployed (`scripts/deploy-to-host.sh`
   overlays it, and host-only scratch files survive). New helpers:
   `tools/snap_screens.sh` and `tools/screen_watch.py`, with
   `tests/test_screen_watch.py`. `scripts/fetch-log.sh` now accepts `.png`
   and one `SUBDIR/` (`screens/*.png`), with tests.

## Artifacts

Screenshots: 1600 px wide, each at most 400 KB. The sim PNG is the whole
DCV desktop; the HUD PNG is headless Chromium at 1920x1080. **HUD shots land
about 3 s after the desktop shot of the same moment**, so they are labelled
by what they show.

| File | Session | What it shows / proves |
|---|---|---|
| `01-sim-ready.png` | inc2 | GUI mode is up: Isaac window maximized with the ChaseCamera. Spot stands at spawn outside the tank, the cleanout door is visible, no popups. |
| `02-hud-ready.png` | inc2 | HUD before the permit: NOT STARTED, LIVE sim link, Spot at spawn on the site map. |
| `03-sim-door-probe.png` | inc2 | Spot stopped with its nose at the 1.3 m cleanout door, still outside, during the probe dwell of the first test. |
| `04-hud-door-probe-fail.png` | inc2 | Six-point probe table (0.5/1.0 m deep x 0.2/0.6/1.0 m high): H2S 51.4-51.6 ppm at 0.2 m, 13.6-13.7 at 0.6 m. Probe points are drawn inside the door. Permit PRE-ENTRY TESTING, checklist 4/4. Also shows fix 2 before it was made: the gauges cover the footer. |
| `05-hud-test-fail.png` | inc2 | The same FAIL (H2S 51.6 ppm, entry denied) about 1 s later. |
| `06-hud-purge-incomplete.png` | ok1 | Early re-test: every probe reading passes (H2S worst 4.6 ppm), yet the agent reports "entry is still not authorized ... 8.5 percent ... versus the 8 percent target". Ventilation ON at 7 %. Also shows the footer overlap before fix 2. |
| `07-sim-inside.png` | inc2 | Spot inside the tank past waypoint 1, under the pipe run, 0.3 s after the waypoint and before the spike. |
| `08-hud-alarm-onset.png` | inc2 | About 1 s after the guard alarm: full-screen "SAFETY GUARD: RETREAT", triggering reading H2S 78.5 ppm, "simulated excursion staged by the presenter", permit ABORTED, speed 0.32 m/s (retreating). |
| `09-sim-far-end.png` | ok1 | Spot at the far end of the interior route (waypoint 5, by the ladder). The chase view inside the unlit tank is dark. |
| `10-sim-incident-retreat.png` | inc2 | Retreat in progress, 3 s after the alarm: Spot turning back toward the door inside the shell. |
| `11-hud-incident-retreat.png` | inc2 | HUD retreat overlay at about 6 s: H2S at the body still 65.3 ppm (ALARM). |
| `12-hud-retreat-complete.png` | inc2 | After the retreat: trigger banner, Spot sitting outside, SITTING, H2S 0.4 ppm. The mission log shows the agent's explanation. |
| `13-hud-permit-completed.png` | ok1 | Non-incident path: permit COMPLETED ("entrant exited the permit space"), full path walked on the map, `walk_to_named -> ok (39.0 s)`. |
| `14-hud-layout-fix-1080p.png` | host | After fix 2: PRE-ENTRY TESTING on one line, CO gauge complete, footer disclosures visible. The permit text is a layout-check event, not a mission. |

Transcripts and logs:

| File | What it is |
|---|---|
| `transcript-inc1-before-fix.txt` | REPL pty transcript, inc1 (4 prompts + quit). Shows the probe rejection, the fallback prompt, and the spike. |
| `transcript-inc2-incident.txt` | REPL transcript, inc2: the cheat-sheet storyline. |
| `transcript-ok1-completed.txt` | REPL transcript, ok1: early re-tests, purge gate, entry, exit. |
| `events-{inc1,inc2,ok1}.jsonl` | Mission audit logs (`--log-path`), every record with `t` + `wall_t`. |
| `hud-audit-{inc1,inc2,ok1}.csv` | `/api/audit.csv` after each session. The HUD keeps its last 200 events, so ok1's file also contains inc2. It includes the presenter `demo_inject` / `demo_clear` / `demo_reset` disclosures. |
| `screen-moments-{inc1,inc2,ok1}.txt` | `tools/screen_watch.py` output: which log record triggered each screenshot. |
| `rh-analyze.txt` | `tools/rh_analyze.py` per session: path, door clearance, retreat timing, body z. |
| `perf-gui.txt` | Every PERF / VIEWPORT / CAMERAS / FACADE-READY line of both GUI sims. |

Not captured: the HUD at the exact moment Spot was inside before the
spike (the HUD capture lag, see above), and any DCV-client-side view.

# v5: leadership rehearsal, 2026-09-28 (GUI presentation mode, agent REPL)

What was run: the RUNBOOK presenter cheat sheet (four prompts: permit + test,
ventilate + early re-test, wait for the purge + re-test, entry) 5 times with a
human-timed staged H2S spike, plus clean runs to COMPLETED, on the sim host as
`ubuntu`: the sim in presentation mode on the DCV console (1920x1080, DCV
server running, no viewer attached), the HUD from `run_hud.sh`, the REPL under
`script -qfc` driven by `tools/rehearse_rep.sh`, and the spike fired by
`inject_gas.sh` from a second, concurrent SSM command 1.0 s after the log
showed interior waypoint 1. Per-rep numbers, the anomalies and the eight
fixes made during the rehearsal: RUNBOOK.md, "Leadership rehearsal
2026-09-28". In short (final build, 5 reps): 5/5 FAIL -> PURGE HOLD (11.0-11.4 %
remaining, the HUD and the agent quoting the same number) -> PASS -> entry ->
guard alarm (disclosure -> alarm 23-123 ms, stop command 0.1 ms after
detection) -> retreat complete (11.3-14.9 s) -> ABORTED, explanation names the
staged spike; clean runs COMPLETED 5/5 waypoints. GUI RTF over the final
launch: 394 samples, min 0.95, median 1.00, max 1.06.

## Frames (`v5/lr-*.png`)

Desktop PNG = the whole DCV console X display (the Isaac window, fullscreen
presentation mode; these frames predate the window-only default of
`tools/snap_screens.sh`); HUD PNG = headless Chromium at 1920x1080 on the host
(`tools/snap_screens.sh`). The HUD shot lands ~3 s after the desktop shot of
the same trigger, so it is labelled by what it shows. Rep = `lr7-rep1`
(incident) unless marked `lr8-clean`.

| Files | Moment | What it shows |
|---|---|---|
| `lr-01-ready-{sim,hud}` | before prompt (a) | Spot standing at spawn facing the tank (fix 6), door and pipe rack in frame, no editor panels; HUD NOT STARTED after the reset (fix 2), empty checklist, no path trail, battery 100 %, camera tile without glare (fix 3). |
| `lr-02-door-probe-fail-{sim,hud}` | first test FAIL | Spot's nose at the cleanout door, the doorway lit by the work light (fix 5) and its headlight; HUD six-point probe table, H2S 51.5 ppm at 0.2 m (13.7-14.1 at 0.6 m), FAIL, permit PRE-ENTRY TESTING, 4/4 checklist, the agent's answer. |
| `lr-03-purge-hold-{sim,hud}` | early re-test | Amber PURGE HOLD: every probe reading within limits (worst H2S 6.4 ppm) but "11.2 % left at test, target <= 8 %"; the agent's answer quotes 11.2 percent and says it will wait. |
| `lr-04-retest-authorized-{sim,hud}` | re-test PASS | ENTRY AUTHORIZED, probe PASS (H2S 4.0 ppm), ventilation 6.0 % remaining; the agent: "The readings are within permissible entry limits". |
| `lr-05-past-door-sim` | waypoint 1 | The single camera cut to the interior corner camera: Spot just inside, curved wall, headlight pool, daylight door. |
| `lr-06-incident-banner-{sim,hud}` | guard alarm | Spot stopped and turning inside; HUD full-screen SAFETY GUARD: RETREAT, triggering reading H2S 77.7 ppm, "simulated excursion staged by the presenter, stop commanded < 1 ms after detection". |
| `lr-07-retreat-complete-{sim,hud}` | retreat complete | Spot sitting outside the door in the work light; HUD ABORTED with the TRIGGER vs now header, SITTING, the agent's explanation. |
| `lr-08-mid-tank-{sim,hud}` (`lr8-clean`) | waypoint 3 | Spot mid-tank beside the pipe run and the sludge, dark but legible; HUD ENTRY IN PROGRESS, path on the map, guard monitoring lines. |
| `lr-09-clean-completed-{sim,hud}` (`lr8-clean`) | permit COMPLETED | Spot out of the door sitting down; HUD COMPLETED, full path walked, "Entry complete: Spot exited the confined space and the permit is closed." (fix 8). |

Known cosmetic leftovers in these frames: the HUD body-camera tile shows the
ground when Spot sits; the mission log fades out the top line of a long
answer; the door frames are low-key by design (dark tank, lit entry).

## Text artifacts

| File | What it is |
|---|---|
| `v5/lr-reps-summary.jsonl` | One line per final session (`lr7-rep1..5`, `lr7-clean`, `lr8-clean`): turn times, test results, purge-hold percent, seat decisions (walk / step / in_envelope with yaw before and after), probe rejections, permit path, alarm, stop command and reading age, Enter -> alarm, alarm -> retreat start, retreat time, whether the explanation names the staged spike, whether "safe" was spoken, tools per turn. |
| `v5/lr7-rep1-transcript.txt`, `v5/lr8-clean-transcript.txt` | REPL pty transcripts (ANSI codes stripped) of the incident rep and the clean run. |
| `v5/lr7-rep1-events.jsonl`, `v5/lr8-clean-events.jsonl` | Mission audit logs (`--log-path`), incl. the `probe_seat` decisions. |

## Earlier v5 files (the gui_presentation and hud_agent_wording modules, same day)

These predate the rehearsal fixes (no work light, payload lights always on,
the old reset heading).

| Files | What they are |
|---|---|
| `sim-before-*.png` / `sim-after-*.png`, `sim-sheet-{before,after}.png` | V1/V2 before/after at approach, probe, inside door, mid, far end, retreat, exit: the editor layout and old camera vs presentation mode, presenter camera and interior lighting. |
| `sim-after-hud-camera-mid.png` | HUD camera tile with Spot mid-tank (payload lights inside). |
| `sim-view-{chase,door,interior,overhead}.png` | The named `set_view.sh` cameras. `overhead` is near-black (not used in the storyline). |
| `sim-ui-editor-toggle.png` | `set_view.sh --ui editor` from an older run. |
| `hud-{ready,entry-limits,purge-hold,purge-reached,alarm,retreat-complete,completed}-{1920x1080,1440x900,1280x720}.png` | `hud/replay_screens.py`: the HUD replaying the 2026-09-27 dry-run events on the Mac at three window sizes (REPLAY badge, placeholder camera tile), for V3/V4 and the P2 polish. |
