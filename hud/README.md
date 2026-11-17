# Presenter HUD

A single-page dashboard for the Spot confined-space entry demo. It's served by
FastAPI + uvicorn on port 8080 and fed read-only from ROS 2 topics. The page
is laid out for a 1920x1080 projector:

- **Left:** the entry permit state (NOT_STARTED is gray, PRE_ENTRY_TESTING amber,
  ENTRY_AUTHORIZED and ENTRY_IN_PROGRESS green, COMPLETED blue, ABORTED red),
  the pre-entry checklist (isolation, LOTO, attendant, rescue plan: a check
  mark when confirmed, a cross when not set, and a dash when no permit event
  has reported the field), the forced-ventilation status with a bar for the
  fraction of the unventilated contamination remaining (always to 0.1 %,
  the same rounding the mission and the agent use), and four gas gauges
  ("Gas at robot", tagged **Now**) in OSHA sampling order: O2, LEL, H2S,
  CO. Each gauge shows its OSHA 1910.146 App. E threshold line and turns
  red on alarm.
- **Purge hold (amber, never red):** when the mission refuses entry on a
  test whose readings all pass because the ventilation purge is not done,
  it publishes a `purge_hold` event `{fraction_remaining, purge_target,
  eta_s, ...}`. The ventilation card turns into **PURGE HOLD — waiting for
  ventilation to reach target** with exactly those numbers ("8.5 % left at
  test · target ≤ 8 %") and a progress bar to the target driven by the live
  ventilation block; it flips to **TARGET REACHED — ready to re-test** when
  the live fraction is at the target, and clears when a re-test starts, the
  permit leaves PRE_ENTRY_TESTING, a new permit opens or the demo resets.
  The agent's tool result carries the same event data, so the screen and the
  spoken answer agree. The probe table's badge reads PURGE HOLD (amber)
  instead of FAIL, and the permit card says what entry waits for. An older
  mission's `purge_incomplete` event (same numbers) is accepted as a
  fallback.
- **Center:** the live front-left camera, and a top-down map of the tank,
  doorway, planned waypoints, Spot's pose and heading, and the path walked.
- **Right:** robot status chips (lease, power, stance, e-stop, battery, speed,
  sim time, sim link), the door-mouth gas probe table (the last probe result
  from the extendable gas probe, simulated; arm-mounted on the real robot:
  one row per sampled point, 2 depths (0.5 and 1.0 m inside the door) x 3
  heights x 4 gases, plus a worst-per-gas row, each cell red when it's over
  its OSHA limit), and the mission log. Agent `say` lines show as speech,
  tool calls as compact one-line entries in plain words ("ACTION Walk to
  entry", "DONE Walk to entry · 39.0 s"; the raw call is the tooltip), and
  safety-guard events in red. The log follows the newest entry; the cut-off
  entry at the top edge fades out instead of showing a sliced line.
  **Download audit log** (JSONL) and **CSV** export what this HUD received.
- **Alarms:** an alarm or abort flashes a full-screen red banner for 8 s
  (click or press Esc to dismiss). The banner shows the triggering reading
  and, once a retreat has actually started, "Spot is retreating
  automatically — no human entered the space", which becomes "Spot
  retreated automatically — ..." only on `retreat_complete` (or "on
  command" for an operator or agent retreat; nothing in rehearsal mode or
  after a failed retreat). It also
  shows the guard's stop-command time ("< 1 ms" below a millisecond, never
  "0 ms") and whether a presenter staged the excursion. While the gas alarm
  stays active, a red strip stays in the header. After an abort, the header
  keeps the reading, tagged **Trigger**, next to the live value of the same
  gas ("now H2S 0.3 ppm"), and the retreat note.
- **Map labels:** SPOT takes the first free position around its arrow, so
  it never prints over SPAWN, ENTRY, DOOR or a waypoint.
- **Footer:** "Simulation — NVIDIA Isaac Sim on AWS" and "Gas readings
  simulated". In `--fake` mode the page also shows a **SYNTHETIC DATA** badge
  (**REPLAY · RECORDED EVENTS** in `--replay` mode), and the connection pill
  reads CONNECTED instead of LIVE. Permit states in mission text show as
  the permit card names them ("PRE-ENTRY TESTING → ENTRY AUTHORIZED");
  the audit downloads keep the raw text.

The HUD never commands the robot. It creates no publishers and no service
clients, so a HUD crash can't leave Spot moving.

## Files

| File | Purpose |
|---|---|
| `server.py` | FastAPI app, WebSocket push, and CLI (`--fake`, `--host`, `--port`, ...) |
| `ros_bridge.py` | `HudState` (thread-safe store), `RosBridge` (rclpy), `FakeBridge` (synthetic scenario) and its replay mode `ReplayBridge` (recorded dry runs) |
| `audit.py` | JSONL/CSV formatting for `/api/audit.jsonl` and `/api/audit.csv` |
| `replay_screens.py` | Mac-only: `--replay` the evidence recordings and shoot the page with headless Google Chrome (from `/Applications`) at 1920x1080, 1440x900, 1280x720 |
| `static/index.html`, `app.js`, `style.css` | Frontend in vanilla JS/CSS. No build step and no CDN, so it works offline. |
| `Dockerfile`, `requirements.txt` | `ros:jazzy-ros-base` image with FastAPI, uvicorn, websockets and Pillow |
| `../scripts/connect-hud.sh` | SSM port-forward from the Mac to the HUD |
| `../scripts/fetch-log.sh` | Pull the mission's JSONL audit logs from the host into `./logs/` (SSM only) |
| `../tests/test_hud.py` | Unit tests (no ROS needed) |

## Run locally (Mac, no ROS)

```bash
python3 -m venv .venv-hud    # the same hash-locked packages as the HUD image
.venv-hud/bin/pip install --require-hashes --no-deps --only-binary :all: -r hud/requirements.lock
.venv-hud/bin/python hud/server.py --fake            # http://127.0.0.1:8080
.venv-hud/bin/python hud/server.py --fake --fake-speed 3   # rehearse the arc faster
```

Fake mode plays a 45 s scenario on a loop, with the same geometry as
`sim/build_scene.py`:

- The permit opens with an empty checklist, and all four fields are
  confirmed at 0.6 s.
- Spot stands, forced ventilation starts at 3 s (tau 1.5 s, compressed for
  the fake; the fake's gas values don't depend on it), Spot walks to the
  entry point, a 6-point probe result (2 depths x 3 heights) arrives on the
  probe topic at 10.5 s,
  and the O2, LEL, H2S and CO tests report the probe's worst values.
- The permit goes to ENTRY_AUTHORIZED, then ENTRY_IN_PROGRESS.
- A presenter `demo_inject` event and an H2S spike start at t = 20 s. The
  safety guard fires at about t = 20.2 s and the permit goes to ABORTED.
- Spot retreats along the path it walked in, then sits.

The synthetic camera frames go through the same `encode_ros_image` path as
real frames.

### Replay a recorded dry run (FakeBridge replay mode)

`--replay` re-publishes a REAL event sequence (the mission's audit log from
`docs/evidence/events-*.jsonl`, optionally plus that session's HUD audit
CSV, which adds the agent transcript and the gas samples the HUD held)
through the same `HudState` ingest paths, so the page can be checked on a
Mac against what really happened:

```bash
python3 hud/server.py --replay docs/evidence/events-ok1.jsonl \
    --replay docs/evidence/hud-audit-ok1.csv --replay-at 445   # hold at sim t = 445 (purge hold)
python3 hud/server.py --replay docs/evidence/events-inc2.jsonl --replay-from 190   # play on
python3 hud/replay_screens.py     # headless Chrome, 3 window sizes -> docs/evidence/v5/hud-*.png
```

The recordings hold events, not telemetry: pose, gas, ventilation and
status are reconstructed from them (see `ReplayBridge`), and the HUD's
clock runs on the recording's timeline, so durations and the alarm-flash
freshness match the moment. `--replay-speech-filter` passes the recorded
agent lines through today's `agent/spot_agent.sanitize_speech` (what the
current agent would publish). The page shows a **REPLAY · RECORDED EVENTS**
badge, the sim-link chip reads REPLAY, and the camera tile is a placeholder.

## Run on the sim host (when the orchestrator says so)

These commands need the repo at `/opt/spot-demo` (`scripts/deploy-to-host.sh`
ships `sim/ host/ agent/ hud/ tests/`).

```bash
bash /opt/spot-demo/host/run_hud.sh           # builds spot-hud:latest if missing, starts detached
bash /opt/spot-demo/host/run_hud.sh --build   # after changing hud/ (code is baked into the image)
bash /opt/spot-demo/host/run_hud.sh --stop    # docker rm -f spot-hud
docker logs -f spot-hud
```

`run_hud.sh` wraps `docker run -d --name spot-hud --network=host
-v /opt/spot-demo/agent:/workspace/agent:ro spot-hud:latest` and waits until
`/api/state` answers. Start the HUD before the mission or agent: the events
history (TRANSIENT_LOCAL) only survives while its publisher is alive.

- `--network=host` puts the HUD on the same DDS network as the Isaac
  container. `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` is baked into the image, and
  `run_hud.sh` adds the loopback-only Fast DDS profile and domain 42
  (`host/sim_lib.sh`). This matters:
  shared memory drops samples between containers that run as different uids,
  and without the profile the sockets bind every interface.
- The server binds to **127.0.0.1:8080** on purpose. The SSM tunnel reaches
  the instance's loopback, so nothing listens on an external interface, and
  the security group has no ingress rules either.
- Mounting `agent/` gives the HUD the mission module's
  `agent/site_defaults.json`. The HUD falls back to that file if the latched
  `payload/site` topic doesn't arrive within 10 s. If the file is missing
  too, it uses a built-in copy of the `build_scene.py` values.

## Open it from the Mac (SSM port forwarding)

```bash
export SPOT_SIM_INSTANCE=<instance-id>
scripts/connect-hud.sh                  # then browse http://localhost:8080
```

`connect-hud.sh` runs this command:

```bash
aws ssm start-session --region us-west-2 --target "$SPOT_SIM_INSTANCE" \
  --document-name AWS-StartPortForwardingSession \
  --parameters "portNumber=8080,localPortNumber=8080"
```

It needs the AWS CLI, the session-manager-plugin, and credentials for the sim
account. Keep the local port equal to the HUD's port (8080): the HUD answers
only `localhost:8080` and `127.0.0.1:8080`, which is what stops a DNS-rebinding
page from reading it, so `connect-hud.sh` refuses a different `LOCAL_PORT`. If
8080 is busy on the Mac, free it. Once it's open, press F11 (Chrome) for full
screen on the projector.

## What it consumes (shared contract 5)

All topics are under `/demo_spot`.

| Topic | Type | QoS |
|---|---|---|
| `payload/status` | `std_msgs/String` JSON | reliable, depth 10 |
| `payload/gas` | `std_msgs/String` JSON (incl. `ventilation`) | reliable, depth 10 |
| `payload/gas/probe_result` | `std_msgs/String` JSON | reliable + volatile, depth 10 (matches volatile and TRANSIENT_LOCAL reliable publishers) |
| `payload/events` | `std_msgs/String` JSON | reliable + TRANSIENT_LOCAL, depth 100 (matches `agent/robot_client.py`, so a restarted HUD gets recent history back; a volatile publisher such as a bare `ros2 topic pub` won't match, so add `--qos-durability transient_local`) |
| `payload/site` | `std_msgs/String` JSON | reliable + TRANSIENT_LOCAL, depth 1 (latched) |
| `odometry` | `nav_msgs/Odometry` | sensor-data (best effort) |
| `camera/frontleft/image` | `sensor_msgs/Image` (rgb8, bgr8, rgba8, bgra8, mono8) | sensor-data (best effort) |

Camera frames are encoded to JPEG on a worker thread, at most 5 fps (the
default). Frames wider than 800 px are downsampled first.

### Event conventions (for the MISSION / AGENT modules)

The sim facade knows nothing about permits, so the HUD derives the permit
state and the alarm flash from `payload/events`
(`{"t","source","kind","message","data"}`):

- **Permit state.** Any event whose `data.permit_state` holds one of
  `NOT_STARTED`, `PRE_ENTRY_TESTING`, `ENTRY_AUTHORIZED`,
  `ENTRY_IN_PROGRESS`, `COMPLETED` or `ABORTED` updates the permit. `data.state`
  and `data.to` also work, and matching is case-insensitive. An optional
  `data.permit_id` is displayed.
- **Full-screen flash.** Any of these triggers it:
  - a `kind` containing `alarm`, `abort`, `estop`, `retreat`, `hazard`,
    `excursion` or `conflict` (the agent's `backend_conflict`: the backend's
    identity was contradicted, so forward motion and entry are held), unless the kind also contains `complete`, `done`, `clear`,
    `resolved`, `finished`, `end`, `ok`, `release` or `reset`. `incomplete`
    doesn't count as `complete`, so the mission's `retreat_incomplete` still
    flashes;
  - `data.alarm: true` on any event;
  - a permit transition to ABORTED;
  - a rising edge of `payload/gas` `alarm`, or of `payload/status`
    `estopped` (false to true).

  One incident flashes once. It stays open while alert signals keep coming
  less than 5 s apart, and for as long as the gas alarm stays on. So the gas
  edge, the guard's `alarm` event, the ABORTED transition, `retreat_start`
  and the guard's 5 s `monitor` readings during the retreat produce a single
  flash. Two things flash again inside an open incident: an E-stop and a
  failed retreat (`RETREAT INCOMPLETE`). Events delivered in the first 5 s
  after the HUD subscribes are treated as TRANSIENT_LOCAL history: they set
  the permit and fill the log, but they never flash.
- **Styling.** `kind: "say"` shows as agent speech. A kind starting with
  `tool` shows as a compact tool line, using `message`, or `data.tool(args)`
  if `message` is empty. Alarm events (the flash rule above) show in red.
  Other `source: "guard"` lines, such as the routine monitor readings, get an
  amber edge. `data.pass` / `data.passed` (a boolean) adds a PASS or FAIL
  tag, which is useful for the O2, LEL, H2S and CO test events. If `data` is
  over 4000 characters it's truncated, but the scalar keys the HUD reads
  (`state`, `permit_state`, `permit_id`, `alarm`, `passed`, ...) are kept.
- **Event time.** `t` is either sim seconds (shown as `T+mm:ss.s`) or a wall
  epoch (shown as local time).
- **Permit checklist.** The HUD reads the mission's Permit dict from
  `data.permit` on any event (`agent/mission.py` sends it on every `permit`
  and `permit_prepared` event), or the same keys at the top level of a kind
  containing `permit`. Accepted keys: `isolation_confirmed` / `isolation`,
  `loto_confirmed` / `loto`, `attendant`, `rescue_plan` / `rescue`, plus
  `permit_id` (or `id` inside `data.permit`). `true` or a non-empty string
  shows a check mark. `false`, `null` or `""` shows a cross. A key that no
  event has carried shows a dash (unknown). When `data.permit` carries the
  mission's `missing` list (and `complete`), that verdict wins: a row is a
  check mark iff its field is not missing, so the HUD never contradicts the
  gate that lets Spot approach. Fields persist across later events and
  reset when a different `permit_id` arrives.
- **Retreat note.** A `retreat*` kind (other than `retreat_incomplete`),
  or an event with `data.auto_retreat: true` or `data.action: "retreat"`,
  sets "Spot is retreating automatically" (source `guard`, or `data.cause`
  `atmosphere` / `telemetry`; otherwise "on command"). Only
  `retreat_complete` (up to 300 s later, above the mission's 240 s retreat
  timeout) turns it into "Spot retreated ...". `retreat_incomplete`, or a
  guard/mission `error` whose message starts with "Retreat" (the mission's
  retreat-worker fault), flashes RETREAT INCOMPLETE and drops the note. The
  triggering reading comes from `data.reading` (the guard's sample) or the
  `payload/gas` edge. The HUD also reads `data.stop_command_s`.
- **Staged excursions.** An alert is "staged by the presenter" if
  `data.staged_by_presenter` is non-empty, or the latest `payload/gas`
  reports `injected_pockets > 0` (the facade's count, authoritative when
  present). Without that count: an event of any source whose kind contains
  `inject` (presenter `demo_inject`, agent `gas_injected`, run_mission
  `demo_inject`) stages alerts for 120 s, also one raised up to 10 s
  before it (the gas edge can beat the disclosure); `demo_clear`,
  `gas_cleared` or `sim_control` ends that window.

### Ventilation and probe (for the SIM / MISSION modules)

- **`payload/gas` → `ventilation`**, the facade's shape: `{"on": bool,
  "t_on", "tau_s", "fraction_remaining" (0..1 of the built-in pocket
  strength), "residual_fraction"}`. The HUD also accepts a bool, a mode
  string (`raw` or `unventilated` means off, `ventilating` means on), a bare
  fraction, or `active` / `mode` / `residual` keys. It's normalized to the
  snapshot key `ventilation` = `{state: on|off|unknown, mode,
  fraction_remaining, tau_s, residual, elapsed_s}`.
- **`payload/gas/probe_result`**, the facade's shape: `{"id", "samples":
  [{"point": [x, y, z], "o2_pct", "lel_pct", "h2s_ppm", "co_ppm",
  "alarms"}], "sim_time", "ventilation", "simulated"}`, or `{"id", "error",
  "samples": []}` for a rejected request, which shows as "Probe request
  rejected". Rows may also use `z` or `height_m`, `readings` with a parallel
  `heights` list, or a dict keyed by height. An event whose kind contains
  `probe` with the same shape in `data` also works (the mission's
  `atmos_probe`: one `samples` row per point with `z` and `inside_m`, plus
  `passed`, `xy`, `points_xy`, `depths_m` and `inside_m`). Rows without a
  depth get one from the site's door plane (`inside_distance`); the table
  shows up to 8 rows, highest first and the nearer depth first. A facade reply that arrives within 3 s after such an event
  doesn't replace it. The table's cell colors are the HUD's own check
  against OSHA App. E, OR'ed with any `alarms` flags. The PASS or FAIL tag
  is shown only when the publisher sends `passed`. Otherwise the tag reads
  WITHIN LIMITS or OVER LIMIT, labeled as the HUD's check.

## HTTP / WebSocket API

| Endpoint | Response |
|---|---|
| `GET /api/state` | Full snapshot: `server, permit, status, gas, pose, site, camera, link` (seconds since each topic was last seen), `alert, thresholds, events_seq, trail_epoch, events` (every protected safety record plus the newest 200 others), `trail`, `guard` (the guard heartbeat status, below), `ingest` (per-topic dropped, malformed and evicted counts) and `world` (undisclosed simulator changes) |
| `GET /api/camera.jpg` | The latest JPEG (`X-Frame-Seq` header). Returns 503 until the first frame arrives. |
| `GET /api/audit.jsonl` | Download (`Content-Disposition: attachment`) of the event buffers, one JSON object per line. They hold per allowlisted source, up to 100 safety records (alarms, permit, retreat, test verdicts, presenter disclosures) and 200 others; up to 50 from unverified sources; up to 100 of the HUD's own records (the simulator's gas alarm edges, refused resets), so a flood on DDS cannot push a real alarm out. Each event has `seq`, `rx_wall`, and `hud` = `{permit_state, replay, gas, gas_alarm, gas_age_s}`: the HUD's view when the event arrived, with the latest `payload/gas` sample if it was under 5 s old. Replayed history gets no gas stamp. Each event's `hud` stamp also carries `source_check` (`allowlisted`, `unverified` or `local`). The `X-Audit-Events`, `X-Audit-Scope`, `X-Audit-Dropped`, `X-Audit-Rejected` and `X-Audit-Evicted` headers are included. |
| `GET /api/audit.csv` | The same events flattened into `seq, t, wall_t, wall_utc, rx_wall_utc, source, source_check, kind, message, permit_state, retreat, o2_pct, lel_pct, h2s_ppm, co_ppm, gas_values_from, gas_alarm, data_json`. `source_check` says whether the event's self-declared `source` is one the HUD expects; `retreat` is `automatic` only when the mission's own fields say so, never because of the `source` field. `t` is the publisher's time (sim time for mission/agent/guard events while the facade reports it), `wall_t` / `wall_utc` the publisher's unix wall time (empty if it sends none), `rx_wall_utc` when this HUD received the event. `gas_values_from` is `event` (values the event carries), `hud_latest` (the receipt stamp), or empty. The file is UTF-8 with a BOM, and cells that start with `= + - @` are prefixed with `'` (formula injection). |
| `WS /ws` | See below. Only an `Origin` of exactly `http://localhost:<port>` or `http://127.0.0.1:<port>` is accepted; a missing or any other Origin is closed with code 1008. At most 8 clients; the ninth is closed with 1013 and the page falls back to polling. |

The WebSocket sends these messages:

- `{"type":"hello","state":<snapshot>}` once, on connect.
- `{"type":"events","events":[...]}`, pushed as soon as events arrive.
- `{"type":"diff","changes":{<changed top-level keys>}}` about 4 times a
  second. When an alert is raised, a diff goes out immediately.
- `{"type":"ping"}` after 2 s with nothing else to send.

The client handles failure like this:

- **Silent socket.** After 6 s with no messages, it drops the socket and
  reconnects with backoff (1 s, doubling up to 5 s).
- **Socket down.** It polls `/api/state` about once a second until the
  socket is back.
- **Stale data.** Values older than 3 s are labeled STALE or NO DATA, never
  shown as live.

## Request checks and status indicators

- **Host and Origin.** The HUD answers only the Host names `localhost` and
  `127.0.0.1` on its own `--port`, and refuses anything else with a 400 that
  names the right URL. `--host` must be a loopback address; another address
  needs `--allow-non-loopback`, which logs a warning and adds no
  authentication. uvicorn runs with `limit_concurrency=32` and a 4 KiB
  WebSocket frame limit.
- **Event provenance.** Events whose `source` is not `mission`, `guard`,
  `agent`, `presenter`, `operator` or (for its three record kinds) `facade` are shown with an UNVERIFIED SOURCE tag
  and change nothing on the page; an alarm from one still flashes, as
  UNVERIFIED ALARM. A presenter reset is refused (RESET REFUSED) while a gas
  alarm or a retreat is in progress.
- **Guard heartbeat.** The HUD subscribes to `payload/guard`. The header pill
  shows MONITORING or STANDBY while heartbeats arrive, GUARD STALLED (red) when
  the agent reports that the guard's gas checks have stopped completing (each
  heartbeat carries `checks` and `check_age_s`), GUARD DOWN (red) when Spot
  last reported inside and no heartbeat has come for 3 s, NOT ARMED when the
  guard is on standby with Spot inside, and NOT RUNNING when none has been
  seen.
- **Ingest limits.** Each topic is rate-limited before parsing, and
  `payload/events` again after parsing, per declared source (25 a second,
  bursts of 250), so a flood under one name does not crowd out the guard's and
  the mission's records; the names are self-declared, so a flood under the
  guard's own name, or above the topic's limit, still does. When anything is
  dropped, an INGEST LIMITED badge shows the count. Camera frames larger than
  1920x1080 or 8 MiB, in an unsupported encoding, or whose buffer does not cover
  their rows are refused before decoding and counted as rejected.
- **Simulator records.** The facade publishes its own `world_control`,
  `facade_service` and `facade_publishers` records on `payload/events` (source
  `facade`). The HUD shows them as SIMULATOR records, APPLIED or REFUSED, keeps
  them through a flood, and never lets them change the permit or the alarm.
- **Undisclosed simulator changes.** An applied gas inject, clear or ventilation
  change with no matching presenter disclosure within 15 s before or after it
  raises an UNDISCLOSED SIM CHANGE badge and a NO DISCLOSURE log line, and is
  counted in the snapshot's `world` key. Expect it during `host/e2e_ros_test.sh`
  and after a manual `ros2 topic pub`: neither sends a disclosure.

## Audit logs

The HUD export only covers what this HUD saw since it started, within the
buffer limits above. The authoritative audit trail is the mission's append-only
JSONL on the sim host, in `/opt/spot-demo/logs/`. To pull it to the Mac:

```bash
export SPOT_SIM_INSTANCE=<instance-id>
scripts/fetch-log.sh                       # every *.jsonl -> ./logs/
scripts/fetch-log.sh 'mission-2026*.jsonl' # a subset
```

The script uses SSM only, with no SSH and no S3. It's `deploy-to-host.sh` in
reverse: the host packs the logs into a tar.gz and base64-encodes it into
one line. The Mac then pulls 20000-character chunks, because
get-command-invocation returns at most 24000 characters. It checks the size
and sha256, unpacks, and deletes the remote temp file. The host logs are
append-only, so fetching again overwrites the older local copies.

## Tests

```bash
python3 -m unittest tests.test_hud -v
```

## Troubleshooting

| Symptom | Check |
|---|---|
| Header says "HUD OFFLINE · RECONNECTING" | The SSM tunnel or the HUD container is down: `docker ps`, `docker logs spot-hud`. |
| LIVE, but gauges show NO DATA | No DDS samples. Start the HUD with `host/run_hud.sh`, which gives it the same settings as the sim: `--network=host`, `ROS_DOMAIN_ID` (default 42), `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` and the loopback Fast DDS profile. A container started by hand without the profile loses samples. |
| "NO CAMERA SIGNAL" | The sim is running with `--cameras none`, or the topic name differs. Use `--camera-topic`, which is relative to `--namespace`. |
| The map says "Waiting for site layout" for more than 10 s | This shouldn't happen: the fallback loads after 10 s. Check the log line `no payload/site within 10 s; using ...`. |
