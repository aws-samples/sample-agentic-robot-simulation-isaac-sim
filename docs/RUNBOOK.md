# Spot confined-space demo: operator runbook

Exact steps to run the demo from the presenter's Mac. The sim host is
(a g6.4xlarge in us-west-2). It has no inbound ports; every
connection below goes over SSM. All numbers were measured on this host on
2026-09-27 (full rehearsal, headless, default cameras) unless marked
otherwise. The storyline runs end to end: unventilated test FAILS,
ventilation, re-test PASSES, entry, exit, COMPLETED (5/5), and the staged H2S
excursion aborts with a guard retreat (3/3 scripted, 1/1 agent-driven).
**Leadership rehearsal 2026-09-28** (GUI presentation mode, agent REPL as
ubuntu on a pty, spike from a second SSM shell): the cheat sheet below 5/5
times plus one clean run to COMPLETED, no fallback prompt needed; numbers in
[Leadership rehearsal](#leadership-rehearsal-2026-09-28-gui-presentation-mode)
and `docs/evidence/README.md` (v5). Presenter script:
[`docs/LEADERSHIP-BRIEF.md`](LEADERSHIP-BRIEF.md). See
[Presenter cheat sheet](#presenter-cheat-sheet) for what to say, and
[Known limitations](#known-limitations) before a live demo.

## 0. One-time setup on the Mac

- AWS CLI v2, the Session Manager plugin, and credentials for the sim account
  (presenters: port forwarding only; see [`docs/iam/`](iam/README.md)).
- `export SPOT_SIM_INSTANCE=<instance-id> AWS_REGION=us-west-2`

### Once per account: one OS account per operator (administrator)

Every operator connects as their own OS account, so the mission record names who ran
it, and every shell session is streamed to CloudWatch Logs, off the host (threat model
T38). The steps, with the commands and how to roll back, are in
[`docs/iam/README.md`](iam/README.md#session-manager-one-os-account-per-operator-and-session-logs-off-the-host):

1. deploy `infra/session-logging.yaml` (the encrypted log group and its KMS key);
2. add the session-log statements of `docs/iam/instance-role-policy.json` to the
   instance role;
3. tag each operator's IAM role or user with `SSMSessionRunAs=<their OS account>`;
4. on the host, create each account: `sudo bash /opt/spot-demo/host/add_operator.sh <os-account>`
   (no password; it may run only `sudo -iu ubuntu`);
5. apply `docs/iam/session-preferences.json` as the account's Session Manager
   preferences (Run As with no default user, so an untagged principal gets no shell).

The preferences apply to every Session Manager shell in the account and Region: use a
dedicated account.

### Once per account: the Bedrock guardrail (administrator)

The agent applies an Amazon Bedrock guardrail to the operator's instruction and the
model's reply, and it is on by default: with no guardrail configured it refuses to
start, unless the operator passes `--no-guardrail`, which warns and is recorded in the
session record (threat model T11). Review
[`infra/bedrock-guardrail.yaml`](../infra/bedrock-guardrail.yaml) and
[`infra/README.md`](../infra/README.md), then, from the Mac, as an administrator:

```bash
aws cloudformation deploy --region us-west-2 --stack-name spot-demo-bedrock-guardrail \
  --template-file infra/bedrock-guardrail.yaml
aws cloudformation describe-stacks --region us-west-2 \
  --stack-name spot-demo-bedrock-guardrail --query 'Stacks[0].Outputs'
```

Put the `GuardrailArn` output into the `ApplyTheOneGuardrail` statement of the instance
role policy, and the `GuardrailId` and `GuardrailVersion` outputs on the host as
`SPOT_GUARDRAIL_ID` and `SPOT_GUARDRAIL_VERSION` (`/etc/environment`, so every session
gets them; `host/run_agent.sh` passes them into the container). The guardrail is a
second, probabilistic layer: the facade's limits, the typed robot data, the
simulator-only check and the operator's confirmations are what stop the robot, and they
hold without it.

The stack also creates a CloudWatch dashboard, `<stack name>-guardrail`, and an alarm on
`InvocationsIntervened`; both are created whether or not you pass `AlarmTopicArn`, which
only decides whether the alarm notifies a topic as well. Read the dashboard after a
rehearsal: an intervention means the guardrail blocked an instruction or a reply, and the
session's mission log and the Bedrock trace say which policy fired.

### Once per host

Build the host to the baseline in [`docs/iam/README.md`](iam/README.md): no
inbound rules, no public IP, IMDSv2 with hop limit 1, the least-privilege
instance role. Then, in a host shell (step 3):

```bash
sudo systemctl disable --now ssh.socket ssh.service      # Session Manager needs no SSH (T44)
sudo systemctl mask --now avahi-daemon.socket avahi-daemon.service   # mDNS on every interface, installed with the desktop (T44)
sudo bash configure_dcv.sh apply --restart   # DCV on loopback only, QUIC off, one viewer (T1)
bash run_sim.sh --prepare-desktop            # opt-in: autologin + screen lock off for the GUI (prints each change)
sudo bash imds_guard.sh apply                # after every boot, and after installing DCV: only root, the agent and the DCV server reach IMDS; every launch refuses without it
```

The sim refuses to start while anything but the SSM agent, DCV or the DHCP client
listens off loopback: on Ubuntu 24.04 with the desktop installed that means SSH
(systemd holds its socket) and `avahi-daemon`, hence the first two lines. The DCV
server checks its licence on EC2 through the instance role, as the system account
`dcv`; `imds_guard.sh` lets that account through when it exists, so apply it after
installing DCV (measured on the host, 2026-11-01).

Do not add a livestream, WebRTC, Foxglove or any other remote viewer to the
host: each is an unauthenticated listener, and `run_sim.sh` (in either mode)
checks once the sim is ready and fails if one appears off loopback. Undo the host changes when the host stops
being a demo host: `bash run_sim.sh --restore-desktop` and
`sudo bash configure_dcv.sh restore --restart`.

**Cost guardrails** (T39; optional sample infrastructure, but do these unless your
account already has its own): a forgotten host then stops itself and spend is alarmed.
Review [`infra/README.md`](../infra/README.md) first. From the Mac, as an
administrator, check that an operating-system shutdown stops the instance rather than
terminating it (the idle stop below powers the host off), then deploy the guardrail
stack, and confirm the SNS subscription email it sends:

```bash
aws ec2 describe-instance-attribute --region us-west-2 --instance-id $SPOT_SIM_INSTANCE \
  --attribute instanceInitiatedShutdownBehavior     # must say "stop", the EC2 default
#   only if it says "terminate":
#   aws ec2 modify-instance-attribute --region us-west-2 --instance-id $SPOT_SIM_INSTANCE \
#     --instance-initiated-shutdown-behavior stop
aws cloudformation deploy --template-file infra/cost-guardrails.yaml --region us-west-2 \
  --stack-name spot-demo-guardrails --parameter-overrides InstanceId=$SPOT_SIM_INSTANCE \
  MonthlyBudgetUSD=<usd> NotificationEmail=<you@example.com>
```

It creates an account-wide AWS Budgets alert (actual and forecast spend, Bedrock
included), an alarm that stops the instance after 2 hours of low CPU, and status-check
alarms (reboot on an instance check; notify on a system check, because g6 supports no
automatic recovery). Then, in a host shell (step 3), the host-side idle stop:

```bash
sudo bash idle_stop.sh apply      # opt-in: power off after 60 idle minutes (--minutes N); prints each change
sudo bash idle_stop.sh status     # APPLIED / NOT APPLIED, and what is keeping the host in use
```

It powers the host off (which stops it) once there has been no Session Manager
session or Run Command, no `spot-sim`, `spot-agent`, `spot-mission`, `spot-hud` or
`spot-rec` container, no agent or mission process and no open mission in
`/opt/spot-demo/logs` for the whole idle time, and it never does when it cannot tell.
Each check is in `journalctl -u spot-idle-stop`; a power-off's reason is also in
`/var/log/spot-idle-stop.log`. A sim left running keeps the host up: stop it (step 9).
Undo with `sudo bash idle_stop.sh remove` and by deleting the stack (README, Cleanup).

## Capacity for a scheduled demo

**Required for any scheduled live demo.** A stopped instance holds no capacity: a start
fails with `InsufficientInstanceCapacity` when its Availability Zone has no free
g6.4xlarge at that moment, and retrying does not guarantee one (a restart of this host
once needed six attempts). An On-Demand Capacity Reservation for the host's instance
type, platform and Availability Zone holds the capacity while it is active.

**It bills while it is active**, at the On-Demand rate (about $1.32 an hour here; see the README's cost table),
whether the instance runs or not; while the instance runs in it, you pay for the
instance only. So create it only as early as you need certainty (the day before, for
example), give it an end time just after the demo window, and cancel it as soon as the
demo is over. Creating it can itself fail for lack of capacity, which is why you create
it ahead of time, not on the day. (A future-dated reservation needs at least 32 vCPUs,
two g6.4xlarge, and a 14-day commitment, so it does not suit one demo.) From the Mac,
as an administrator:

```bash
# The host's type, Availability Zone and reservation preference ("open" uses an open reservation)
aws ec2 describe-instances --region us-west-2 --instance-ids $SPOT_SIM_INSTANCE --output text \
  --query 'Reservations[0].Instances[0].[InstanceType,Placement.AvailabilityZone,CapacityReservationSpecification.CapacityReservationPreference]'
# Create it, ending (UTC) just after the demo window; note the cr-... id
aws ec2 create-capacity-reservation --region us-west-2 --instance-type g6.4xlarge \
  --instance-platform Linux/UNIX --availability-zone us-west-2a --instance-count 1 \
  --instance-match-criteria open --end-date-type limited --end-date <YYYY-MM-DDThh:mm:ssZ> \
  --query 'CapacityReservation.[CapacityReservationId,State]' --output text
# Check: State active; AvailableInstanceCount 0 once the host runs in it
aws ec2 describe-capacity-reservations --region us-west-2 --capacity-reservation-ids <cr-id> --output text \
  --query 'CapacityReservations[0].[State,TotalInstanceCount,AvailableInstanceCount,EndDate]'
# Cancel it as soon as the demo is over (the end date also ends it, within an hour)
aws ec2 cancel-capacity-reservation --region us-west-2 --capacity-reservation-id <cr-id>
```

With `open` matching the host needs no change: an instance whose preference is `open`
(the default) runs in a matching open reservation when it starts, or at once if it is
already running. Use a dedicated account, so no other instance takes it. If the
preference is `none`, set it while the instance is stopped:
`aws ec2 modify-instance-capacity-reservation-attributes --region us-west-2 --instance-id $SPOT_SIM_INSTANCE --capacity-reservation-specification CapacityReservationPreference=open`.
The idle stop and the idle alarm still stop the host inside the window; the
reservation keeps billing and keeps the capacity for the next start.

## 1. Start the instance (cost control: stop it again in step 9)

```bash
aws ec2 start-instances --region us-west-2 --instance-ids $SPOT_SIM_INSTANCE
aws ec2 wait instance-status-ok --region us-west-2 --instance-ids $SPOT_SIM_INSTANCE
```

Or `scripts/start-host.sh`, which does the same and retries an
`InsufficientInstanceCapacity` error with a growing wait (30 s, doubling, at most
300 s; 6 attempts by default), then prints the reservation commands above if there is
still no capacity. Before a scheduled demo, reserve the capacity instead of relying on
retries (above).

## 2. Deploy code (only if the repo changed)

```bash
scripts/deploy-to-host.sh    # ships sim/ host/ agent/ hud/ tests/ tools/ to /opt/spot-demo (~1 min, sha256-checked)
```

`agent/` and `hud/` code is baked into their images, so after a deploy that
touched them, rebuild on the host (layer cache: a few seconds):
`docker build -f /opt/spot-demo/agent/Dockerfile -t spot-agent:latest /opt/spot-demo`
and `bash /opt/spot-demo/host/run_hud.sh --build` (or pass `--build` to any
`run_agent.sh` call).

Optional facade check (no cameras, ~2.5 min, replaces any running sim, and
refuses while an agent or mission container is running; a HUD left running
shows UNDISCLOSED SIM CHANGE for its gas controls, which is expected):
`bash /opt/spot-demo/host/e2e_ros_test.sh`. Measured 2026-09-27 as ubuntu:
`E2E OK: 44/44` (2 runs), RTF 1.00, first stand-up moved 0.078 m / yaw
+0.004 rad, three sit -> stand -> walk cycles moved 0.060-0.063 m / yaw
+0.034-0.038 rad with body z >= 0.46 m; Spot lined up at the door mouth
(nose at x 5.99, 0.99 m from the 1.0 m-deep points): unventilated probe H2S
51.3 ppm at 0.2 m, the six storyline points (0.5 and 1.0 m inside x 0.2/0.6/
1.0 m) accepted, a point 1.31 m from the nose rejected (reach), a point behind
the shell beside the jamb rejected ("occluded by tank wall"); clean (2.5 ppm)
after 6 tau of ventilation, reset restores it. Logs: new per-run files
`/opt/spot-demo/logs/e2e-{sim,client}-<run_id>.log`.

## 3. Open a host shell

```bash
aws ssm start-session --region us-west-2 --target $SPOT_SIM_INSTANCE   # a shell as your own OS account (Run As)
sudo -iu ubuntu            # the only sudo your account has; ubuntu is in the docker group
cd /opt/spot-demo/host
```

Session Manager starts the shell as the OS account your IAM identity's
`SSMSessionRunAs` tag names (`whoami`); without the tag there is no session.
After `sudo -iu ubuntu`, `SUDO_USER` is your account, and `run_agent.sh` passes
it to the agent, so the mission record names you. The agent and `--mission`
refuse to start, and say why, when that login is a shared account (`ssm-user`,
`ubuntu`, `root`) or missing, for example in a shell opened with `sudo su` or
on the DCV desktop terminal (DCV logs in as `ubuntu`). Pass `--unattributed-run`
only when you mean to: the session and mission-start records then say that the
run names no operator. Every session is streamed to CloudWatch Logs as it
happens. A session ends after 20 minutes without a keystroke, which the agent
takes like a dropped session (step 6).

Open a second session the same way for the presenter console (step 7). The
DCV desktop terminal (step 4b) works for everything but the agent and the
scripted mission, which refuse it without `--unattributed-run`.

## 4. Start the simulator (only ONE sim at a time)

Every shell that starts the sim needs `export ACCEPT_EULA=Y` first: that is
your acceptance of the NVIDIA Isaac Sim licence, which you should read before
the first run. NVIDIA data collection stays off unless you also set
`PRIVACY_CONSENT=Y`.

**a. Headless** (HUD camera only):

```bash
bash run_sim.sh --background --headless true   # returns only once THIS launch is ready (35 s warm)
bash run_sim.sh --status                       # any time: READY (... run_id=...) / NOT READY / NOT RUNNING
docker logs spot-sim 2>&1 | grep ^PERF | tail -3   # expect rtf=0.98-1.02
```

`--background` starts the container detached (it survives closing the
shell), writes a NEW log `/opt/spot-demo/logs/sim-<run_id>.log`
(`sim-latest.log` points at it; `tail -f` it if you like) and waits for
`FACADE-READY ... run_id=<run_id>` in `docker logs spot-sim`, i.e. from the
container it just started. Exit 1 with the log tail if the container dies or
is not ready within 300 s (`SPOT_SIM_READY_TIMEOUT`). Do not redirect the sim
to `/tmp` yourself: `/tmp` is sticky and `fs.protected_regular=2`, so a
root-owned leftover (from an SSM run) cannot be reopened by ubuntu, the
redirect fails, no sim starts, and grepping the old file finds an OLD
FACADE-READY line. (The root-owned `/tmp/spot-*.log` files were removed on
2026-09-27; no script writes to `/tmp` any more.) Measured as ubuntu: READY
after 35 s, RTF 0.98-1.00.

The tank starts UNVENTILATED (`--gas-initial raw`, the default), so the first
pre-entry test fails. `--gas-initial ventilated` starts at the ventilation
residual (rehearsing the entry alone).

**b. GUI on the DCV desktop** (presentation mode: the Isaac viewport alone,
fullscreen, driven by the presenter camera):

```bash
# Mac, separate terminal: prints how to fetch the DCV password, then tunnels 8443
scripts/connect-dcv.sh            # browse https://localhost:8443, user ubuntu
# Host:
bash run_sim.sh --background --gui      # READY in ~40-65 s warm; log /opt/spot-demo/logs/sim-<run_id>.log
bash run_sim.sh --background --editor   # the same with the full Isaac Sim editor (debugging only)
bash set_view.sh auto                   # presenter camera: auto (default) | chase | interior | overhead | door
bash set_view.sh --ui editor            # editor panels back without a restart (--ui presentation: back again)
```

About 5 s after READY the Isaac window goes fullscreen with no panels, menus
or toolbars (`docker logs spot-sim | grep '^UI check'`: `mode=presentation
fullscreen=True hideUi=True window=1920x1080`). Nothing to click: a stray Esc
no longer leaves fullscreen, and with no viewer attached `run_sim.sh` raises
the 1024x768 console to 1920x1080 first (an attached viewer keeps its own
size; the window follows it). The `auto` camera is a low 3/4 chase outside,
one cut to a fixed interior corner camera at the wall crossing and one cut
back on the way out; `reset_spot.sh` puts it back on `auto`. Spot's payload
lights fade in near the tank and are off out in the sunlight (no glare in the
HUD camera tile at spawn or after the retreat), and an entry work light
lights the door face, which is on the shadow side of the tank. Inside, the
roofed tank is dark but legible (docs/evidence/v5/lr-*-sim.png). GNOME notification
banners, screen lock and crash popups were switched off once by `run_sim.sh
--prepare-desktop`; `--gui` refuses to start while DCV listens off loopback.

Details and troubleshooting: `host/gui_notes.md`. Both modes publish the same
topics, so everything below works with either. RTF in GUI presentation
mode (leadership rehearsal 2026-09-28, DCV server running, default cameras,
`--render-hz 10`, HUD up, final build): 394 five-second PERF samples over the
whole launch (5 reps, 2 clean runs, host checks, screenshot capture): min
0.95, median 1.00, max 1.06, none below 0.95; render 36 ms median, 50 ms max.
The previous launch (build before fixes 5-8, 5 reps + 2 clean runs): 624
samples, median 1.00, one 0.92 sample while two evidence screenshots (X grab +
headless Chromium) ran at once at the alarm, otherwise >= 0.95. Still not
measured: the extra load of a DCV viewer actually streaming (no viewer was
attached). Older numbers (editor window, 2026-09-27 dry run):
[`docs/evidence/README.md`](evidence/README.md).

Measured (headless, default cameras frontleft+frontright): RTF 0.96-1.04
over 338 five-second PERF samples during the whole rehearsal (none below
0.95; 95 % within 0.98-1.02), `camera/frontleft/image` 10.0 Hz at 640x480,
`payload/site` latched.

## 5. Start the HUD

```bash
bash run_hud.sh                    # container spot-hud, 127.0.0.1:8080 on the host
# Mac, separate terminal:
scripts/connect-hud.sh             # browse http://localhost:8080, F11 for full screen
```

Start the HUD before the mission or agent. The event history only survives
while its publisher lives, so a HUD started afterwards shows the permit as
NOT_STARTED until the next event. `run_hud.sh` prints `sim ready (...
run_id=...)`, or a WARN if no ready sim is running (the HUD then shows the
default site until the sim publishes it). Check it from the host with
`curl -s localhost:8080/api/state | head -c 400`. Audit export (what this HUD
saw, within its buffer limits): `curl -s localhost:8080/api/audit.csv` (or
`/api/audit.jsonl`); the authoritative log is the mission JSONL in
`/opt/spot-demo/logs/`, pulled to the Mac with `scripts/fetch-log.sh`.

## 6. Drive the robot: the storyline

Before a run: `bash reset_spot.sh --reset-ventilation` (Spot back at spawn,
standing; injected gas cleared; tank unventilated again). About 1 s.
`run_agent.sh` (agent or `--mission`, not `--dry-run`) refuses to start
unless the running spot-sim container has printed FACADE-READY, and says how
to start it. Name each log uniquely (the `$(date -u +%Y%m%dT%H%M%S)` below).

**Agent (the live demo)**, natural language on Bedrock
`us.anthropic.claude-sonnet-5` via the instance role (no keys):

```bash
bash run_agent.sh --log-path /workspace/logs/demo-$(date -u +%Y%m%dT%H%M%S).jsonl   # REPL, needs a TTY
operator> Prepare permit CSE-0427: the attendant is at the entry, isolation and lockout-tagout are in place, rescue plan: on-site standby team. Then run the pre-entry atmospheric test at the tank.
            # -> the console asks you to confirm the attestations: type y
operator> Start ventilation, wait 30 seconds, then re-test. Don't wait for the purge.
            # -> PURGE HOLD: door readings within limits, ~11 % remaining, target <= 8 %
operator> Wait for the purge target, then re-test.
operator> Proceed with the confined space entry.
            # -> the console asks you to confirm the entry: type y. Spot re-tests at the door
            #    before it crosses; a reading over the limits, ventilation stopping or a
            #    battery under 30 % withdraws the authorization.
            # once the HUD map shows Spot at waypoint 1: presenter shell -> bash inject_gas.sh   (step 7)
operator> quit                # refused while Spot is inside the permit space
```

**Console commands and confirmations.** The model cannot attest the permit,
enter the permit space, release an e-stop, self-right inside, or e-stop
inside (hard or gentle; both end with motor power off) on its own: the
console asks `y/N` for each, and anything but `y` refuses it. A model's
e-stop inside halts motion at once and waits for you only before power is
cut. Stopping from the console never asks. At any time, also while the model is
working, type `/stop` (halt motion), `/retreat` (bring Spot out), `/estop`
(gentle e-stop; `/estop hard` cuts power), `/status`, or `/help`. If a
model turn hangs, the console still answers these, and so it does while the
agent waits to exit for a retreat. Only lines that start with `agent> ` are the
model's. If you answer a question `n` (or let it expire), the model cannot ask
the same thing again for 30 s; at most two questions wait at once.

**Audit head lines.** At session start, at mission start, after every
confirmation, at mission end and at session end the console prints one
`[audit head] <log file> <records>:<sha256> after ...` line: how many records
the mission JSONL holds and the SHA-256 of the last one (each record carries
the SHA-256 of the one before). With session logging on, these lines reach
CloudWatch Logs as they are printed, so they are a copy of the log's state that
nobody on the host can take back; see "Check a log against the session log"
(step 9).

**BACKEND IDENTITY CONFLICT.** The agent commands only this simulator. If,
after it identified the simulator, something on the host publishes a
contradicting `payload/status` or a second publisher appears on it, the agent
records a `backend_conflict` event once (the HUD flashes it) and holds forward
motion and entry for the rest of the session. Stops, the guard's retreat and
`/retreat` still work, and `/status` shows the conflict. Bring Spot out, quit,
find what published on the bus (`ros2 topic info -v /demo_spot/payload/status`
in a ROS container on the same domain), and restart the agent only once it is
gone.

**Missing gas readings and failed retreats.** Inside the permit space, or
walking toward it, the mission holds Spot still as soon as no fresh gas
reading arrives and resumes when one does (a `gas_hold` record); moving away,
stops and the guard's retreat are never held. If a guard retreat cannot bring
Spot out while the alarm persists, it pauses 5 s and tries again, up to three
attempts, and each failure prints `[OPERATOR ATTENTION]` on the console. After
the last one Spot stays where it stopped: bring it out by hand (`/retreat`,
or the presenter's reset in the simulator). An exit (Ctrl-C, SIGTERM, a
hang-up) waits up to 900 s for these attempts.

**Purge estimates never authorize entry.** Without a current ventilation
reading from the simulator, a ventilated tank is not authorized: the HUD's
purge panel says ESTIMATE, and the agent says the purge is not confirmed.

**Run the agent so that a dropped session cannot kill it.** If the SSM
session drops, the shell should send SIGHUP, and the agent brings Spot out
before it exits (on the host, 2026-11-01: a session ended by the service while
Spot was inside brought it out in 12 s; a session whose client disappeared stayed
connected and the run went on until the session was ended). To avoid even that, run it
inside `tmux` on the host, on a tmux server of your own started from your
`sudo -iu ubuntu` shell (`tmux -L <your account>`): a server another operator
started keeps their `SUDO_USER`, and the record would name them. To stop an agent
from another shell, use `docker stop spot-agent` (SIGTERM; the container has
a 900 s stop timeout so the retreat, and its retries, can finish), never `docker rm -f`, which
kills it at once and leaves Spot wherever it is.

(Before 2026-09-28 the second prompt was "Start ventilation and tell me when
it's safe, then re-test." It still works, but it skips the purge-hold beat and
puts the word "safe" in the presenter's mouth; the agent never says it.)
Timings for these four prompts: [Leadership rehearsal](#leadership-rehearsal-2026-09-28-gui-presentation-mode).
The older one-session measurement below used the three-prompt form:

Measured (rehearsal 2026-09-27, one session, 3 turns, 122 s end to end):

| Turn | Wall | Tools the model called | Result |
|---|---|---|---|
| (a) permit + test | 32 s | prepare_permit, stand_up, run_atmospheric_test | FAIL: H2S 51.6 ppm at 0.2 m (O2 19.9 %, LEL 7.7 %, CO 0.7 ppm pass); entry denied, permit PRE_ENTRY_TESTING |
| (b) ventilate + re-test | 57 s | start_ventilation, ventilation_status(wait 45 s), run_atmospheric_test | purge 41.5 s to 8.7 % remaining; PASS: O2 20.8 %, LEL 0.8 %, H2S 4.2 ppm, CO 0.7 ppm; ENTRY_AUTHORIZED |
| (c) entry + incident | 31 s | enter_confined_space | presenter spike 13 s into the turn at (4.0, 0.4); guard alarm H2S 78.3 ppm 92 ms after the presenter publish; retreat 14.1 s; Spot sitting outside at (7.8, 0.1); permit ABORTED; spoken explanation 3.6 s after the retreat |

The model's final answer in (c), verbatim: "A gas alarm tripped the
SafetyGuard: hydrogen sulfide spiked to 78.3 ppm, well over the 10 ppm limit,
near waypoint 1 just past the door. This was a presenter-staged simulated
spike, not a real reading. The guard stopped Spot within a fraction of a
second and retreated it, and that retreat completed successfully, with Spot
now sitting outside at the entry point. Permit CSE-0427 is now aborted, so
re-entry will need a new permit and a fresh atmospheric test."

To end on COMPLETED instead, skip the injection: after (c) say "Walk back out
to the entry point." (walk_to_named entry exits through the door, closes the
permit and sits). Dry run 2026-09-27 (GUI): entry turn 38.8 s (5/5
waypoints), exit turn 43.9 s, permit COMPLETED, Spot sitting at (13.7, 0.2);
min door clearance 0.276 m.

Measured in GUI mode (presenter dry run 2026-09-27, clean session inc2, fix
below applied): turn (a) 33.3 s (FAIL, H2S 51.6 ppm), turn (b) 60.0 s (purge
45.5 s to 8.0 %, PASS H2S 3.6 ppm, ENTRY_AUTHORIZED), turn (c) 24.2 s: B's
Enter 1.0 s after interior waypoint 1 -> guard alarm (H2S 78.5 ppm) 1.23 s
later (34 ms after the presenter's disclosure event), retreat start +14 ms,
retreat 12.1 s, Spot sitting outside, permit ABORTED, explanation at the end
of the turn. The first session (inc1, before the fix) needed one extra
prompt at beat 4: see the cheat sheet.

Asking for an early re-test is refused with a reason: 2 s into ventilation
the test simply FAILS (H2S 38.9 ppm); 46 s in, with every gas within limits
at the door (H2S 4.6 ppm) but 8.5 % remaining, the mission logs
`purge_incomplete` "Purge incomplete — wait: ... keep ventilating about 2 s
more, then re-test" and the agent relays it; the permit stays
PRE_ENTRY_TESTING.
One-shot form: `bash run_agent.sh --once "..."` (a new session per call, so
the permit does not carry over; use the REPL for the storyline).

**Scripted storyline** (deterministic, no LLM; the fallback if Bedrock is
slow or unavailable):

```bash
bash run_agent.sh --mission --storyline --log /workspace/logs/story-$(date -u +%Y%m%dT%H%M%S).jsonl
```

It resets the simulated atmosphere itself (unless `--no-gas-reset`), records
a demo permit, tests (FAIL), ventilates (`--vent-tau`, default 15 s), waits
for the purge (3 tau), re-tests (PASS), enters, surveys, exits and sits.
Exit codes: 0 COMPLETED, 2 ABORTED as designed (Spot outside), 1 failure.
Real permit values: `--permit-id CSE-0427 --attendant "attendant 2"
--rescue-plan "On-site standby team"` (`--attendant` takes a role label or a
roster id, never a name; a value that looks like a name stops the run before
anything is recorded, and is not echoed). The run asks you at the console to confirm the
attestations and the entry; `--no-isolation` / `--no-loto` show the permit
gate (the run stops before Spot moves).

**Scripted excursion** (staged H2S spike, disclosed in the log):

```bash
bash reset_spot.sh && bash inject_gas.sh vent on 3 && sleep 20   # ventilated tank
bash run_agent.sh --mission --inject-h2s-at-waypoint 2 --log /workspace/logs/exc-$(date -u +%Y%m%dT%H%M%S).jsonl
# or: bash run_agent.sh --mission --storyline --inject-h2s-at-waypoint 2   (full story; not timed on the host)
```

## 7. Presenter console (second shell)

```bash
bash /opt/spot-demo/host/inject_gas.sh             # 80 ppm H2S, 1 m pocket, z 0.3 m, at Spot's position
bash /opt/spot-demo/host/inject_gas.sh lel 30 1.0  # other gases: h2s|lel|o2|co amount [radius] [z]
bash /opt/spot-demo/host/inject_gas.sh clear       # remove every injected pocket (no sim restart)
bash /opt/spot-demo/host/inject_gas.sh vent on [tau_s] | vent off | vent reset
```

Every action also publishes a disclosure event (`presenter` / `demo_inject`
etc.), so the HUD transcript and alert show that the spike was staged
("staged": true on the alert). The deterministic SafetyGuard (not the LLM)
stops Spot and backs it out through the door. Each call starts a ros:jazzy
container: allow about 1 s from Enter to the spike (measured 1.1 s; the
disclosure event follows the spike within ~10 ms). The script then stays up
1-3 s until an events subscriber has received the disclosure (2 x 5 presenter
events, 10/10 delivered, 2026-09-27). Presenter events carry `wall_t` (unix
time) and `t` (sim time when the script saw payload/status, else wall time;
`t_source` says which). As an alternative, start the
agent with `SPOT_DEMO_PRESENTER=1 bash run_agent.sh`: the model then gets
`inject_gas_demo_spike` / `clear_injected_gas`, but only between turns.

## 8. Between runs

- `bash reset_spot.sh --reset-ventilation`: clears injected gas, un-ventilates
  the tank and teleports Spot to spawn (14, 0), standing, in about 1 s.
  Without `--reset-ventilation` the tank keeps its current ventilation state.
- No sim restart is needed any more: injected pockets clear with
  `inject_gas.sh clear`, and a sat Spot can stand again (re-stand fixed; see
  the no-reset run below). The scripted storyline also clears and resets the
  atmosphere at its start.
- The HUD starts a fresh run on `reset_spot.sh` (its presenter `demo_reset`
  event): permit NOT STARTED with an empty checklist, no probe table, no path
  trail, no incident banner. The mission log and the audit export keep every
  event. (Until 2026-09-28 the next run started under the previous run's
  ABORTED permit and trigger banner.)

## 9. Stop (cost control)

Stop the agent before the sim: `run_sim.sh --stop` refuses while an agent or
mission container is running (it would lose gas and odometry mid-entry).

```bash
bash run_hud.sh --stop; docker stop spot-agent spot-mission 2>/dev/null; bash run_sim.sh --stop
exit                                                            # leave the host
aws ec2 stop-instances --region us-west-2 --instance-ids $SPOT_SIM_INSTANCE   # Mac
```

A stopped instance still bills for its 200 GB gp3 volume, but not for the
g6.4xlarge. A Capacity Reservation keeps billing until it ends or you cancel it
("Capacity for a scheduled demo"). With the guardrails in place a forgotten host
stops itself (after 60 idle minutes on the host, or 2 hours of low CPU), but stop
it here anyway.

Logs stay on the host in `/opt/spot-demo/logs/` (one file per run:
`sim-*`, `e2e-*`, `smoke-*`, plus the mission JSONL). Pull them with
`scripts/fetch-log.sh 'sim-2026*.log'` (patterns ending .jsonl/.log/.csv/
.json/.txt).

**Check a log against the session log** (threat model T38). The mission JSONL
can be deleted, cut short or rewritten on the host; the `[audit head]` lines in
the operator's Session Manager session log cannot. Pull the JSONL, take the
operator's session ID from Session Manager's own history (not from a log stream's
name: the instance role can create streams, so anyone with root on the host could
add one named like an operator's session), export that session's log stream from the
CloudWatch Logs log group `/spot-demo/session-manager`, and check the file against
every head line in it, not only the last. The export is the operator's whole console
transcript: keep it with the fetched logs, outside the checkout (T36):

```bash
scripts/fetch-log.sh 'demo-20261030T101500.jsonl'
aws ssm describe-sessions --region us-west-2 --state History \
  --filters key=Target,value=$SPOT_SIM_INSTANCE --query 'Sessions[].[SessionId,Owner,StartDate]'
aws logs filter-log-events --region us-west-2 --log-group-name /spot-demo/session-manager \
  --log-stream-names <session-id> --output json > ~/spot-demo-logs/<instance-id>/session-<session-id>.json
python3 agent/mission.py --verify ~/spot-demo-logs/<instance-id>/demo-20261030T101500.jsonl \
  --heads-from ~/spot-demo-logs/<instance-id>/session-<session-id>.json
# or a head copied from the session log by hand: --expect-head 57:9f71...169
```

It prints how many head lines it found for that file, one line per finding and
the file's own head, and exits 0 only with no finding: TRUNCATED (fewer records
than a head shows: records removed from the end, or the file replaced),
REWRITTEN (a record's `prev_sha256` does not match the line before it: a record
changed, inserted or removed) or MISMATCH (the record at a head's count has
another SHA-256: the chain rewritten from there on, or another file). A head
line cut in two by the session log's chunking is not found; copy it with
`--expect-head`. Records written after the last head line printed (a crash
before the session end) are not covered.

## Measured rehearsal numbers (2026-09-27, headless, default cameras)

Logs on the host: `/opt/spot-demo/logs/rehearsal-*.jsonl` (mission audit),
`odom-rehearsal-*.jsonl` (10 Hz odometry + gas), `rehearsal-agent-transcript.txt`,
`rehearsal-agent-events.jsonl`, `hud-{start,fail,vent,authorized,incident,incident-after,end}.json`
(`/api/state` at each moment), `hud-audit.csv` (75 lines), `rehearsal-sim.log`.
Door clearance = closest distance between Spot's 1.10 x 0.50 m body box and
a jamb face (jambs at |y| 0.65 m, 1.30 m clear width).

**Scripted storyline, 5 runs** (`reset_spot.sh --reset-ventilation` between):
5/5 COMPLETED, exit 0.

| | Min | Max |
|---|---|---|
| Duration (mission) | 145.95 s | 146.85 s (150-151 s wall incl. container start) |
| Test 1 (raw), worst of 3 heights | FAIL: H2S 51.34-51.72 ppm at 0.2 m | O2 19.87-19.93 %, LEL 7.70-7.84 %, CO 0.23-0.74 ppm (pass) |
| Ventilation wait (tau 15 s, to 3 tau) | 45.3 s | 45.3 s |
| Test 2 (ventilated) | PASS: H2S 4.04-4.39 ppm, LEL 0.66-0.90 % | O2 20.68-20.79 %, CO 0.31-0.88 ppm |
| Body sensor inside the tank | O2 min 20.69 % | LEL max 1.55-1.68 %, H2S max 1.51-1.70 ppm |
| Waypoints | 5/5 | |
| Path length | 36.1 m | 36.3 m |
| Min door clearance (body to jamb) | 0.209 m | 0.245 m (center within 0.109-0.133 m of the door axis) |
| Body z inside | >= 0.51 m | |
| End | sitting at (7.72-7.83, -0.01-0.03), 1.7-1.8 m outside the door | |

Extra run without any reset (standing up from the previous run's sit, 2 m
from the door): COMPLETED in 139.5 s, path 30.5 m, door clearance 0.266 m.

**Scripted excursion, 3 runs** (`--inject-h2s-at-waypoint 2`, ventilated
tank): 3/3 ABORTED, exit 2, 64.5-65.4 s. The spike fires on reaching interior
waypoint 3 of 5 (index 2, tank center). Injection -> guard alarm 33-117 ms
(H2S 78.1-78.5 ppm; reading age 13-44 ms); alarm -> zero twist 0.1 ms;
alarm -> retreat start 12 ms; retreat 23.8-24.4 s back through the door;
Spot outside and sitting at (7.78-7.82, 0.02-0.06); path 23.8-23.9 m, door
clearance 0.235-0.317 m.

**Agent-driven storyline, 1 session**: see the table in step 6. HUD
snapshots: fail = PRE_ENTRY_TESTING with the checklist complete (isolation,
LOTO, attendant, rescue plan; recorded before the hardening pass with a
fictional attendant name); vent = ventilation on, 55 %
remaining; authorized = ENTRY_AUTHORIZED, 7 % remaining; incident = ABORTED,
alert "SAFETY GUARD: RETREAT", reading "H2S 78.3 ppm (limit < 10 ppm)",
note "Spot is retreating automatically", staged true; incident-after = note
"Spot retreated automatically".

## Presenter cheat sheet

Two shells: A runs the agent REPL (or the scripted storyline), B is the
presenter console in `/opt/spot-demo/host`. The DCV window (Isaac, presentation
mode) and the HUD full screen side by side. Both shells are `sudo -iu ubuntu`
shells from `aws ssm start-session` (step 3); A needs a real terminal (the
REPL; under automation `script -qfc 'bash run_agent.sh ...' /path/transcript.txt`
gives it one and records the transcript; `tools/rehearse_rep.sh` does exactly
that). Timings (GUI presentation mode, leadership rehearsal 2026-09-28, final
build, 5 reps + clean runs): turn a 32-34 s, turn b1 44-46 s, turn b2 12-13 s,
turn c 24-29 s with the spike (37-38 s without; exit turn 43-44 s); one rep is
about 2.5 min of robot time. Presenter script with what to say: `docs/LEADERSHIP-BRIEF.md`.
Evidence capture: `bash /opt/spot-demo/tools/snap_screens.sh NAME` (Isaac Sim
window + HUD PNGs in `logs/screens/`), or `python3 /opt/spot-demo/tools/screen_watch.py
/opt/spot-demo/logs/<mission>.jsonl PREFIX &` for the storyline moments
(two moments at once cost one 0.92 RTF sample; not needed during a demo).

| # | Beat | Do | Say (short, honest) | If it fails |
|---|---|---|---|---|
| 0 | Reset | B: `bash reset_spot.sh --reset-ventilation` (Spot to spawn, standing; battery to 100 %; camera `auto`; HUD back to NOT STARTED). The simulated battery (`--battery-drain demo`, default) drains 25 %/h and never below 30 %, so it stays green for a 2 h session | (before the audience) | "no subscriber ... is the sim running?": `bash run_sim.sh --status`; NOT RUNNING / NOT READY: `bash run_sim.sh --background --gui` (returns when ready). Editor panels on screen: `bash set_view.sh --ui presentation` |
| 1 | Permit | A: prompt (a) | "Nobody goes near a confined space without a permit: isolation and lockout-tagout confirmed, an attendant assigned (by role), a rescue plan. The robot will not approach the entry until those fields are filled in." | Agent asks for a missing field: give it. Model slow or erroring: switch to the scripted storyline in A |
| 2 | First test | (part of turn a, ~20 s) | "Spot stops with its nose at the cleanout door, outside the space, and reaches in with a gas probe (simulated here; a real Spot would carry it on its arm): half a meter and one meter inside, at three heights each, because hydrogen sulfide is heavier than air. Oxygen first, then flammables, then toxics, in the OSHA order. H2S is about 52 ppm at floor level, five times the limit: entry denied." | Test passed: the sim was started `--gas-initial ventilated` or not reset; run `reset_spot.sh --reset-ventilation`, re-ask |
| 3 | Ventilate, early re-test | A: prompt (b1) "Start ventilation, wait 30 seconds, then re-test. Don't wait for the purge." | "Just like a real tank entry, we force-ventilate. I am asking for the re-test early on purpose: the readings at the door already look fine, but the tank is not purged yet." Then point at the amber PURGE HOLD: "Every gas is within permissible entry limits at the door, H2S about 6 ppm, but 11 percent of the contamination is still in the tank and the target is 8. The system holds the entry; the HUD and the agent quote the same number." | The early re-test FAILS instead (H2S still > 10 ppm: the model re-tested before ~25 s): carry on with (b2). The model waited and re-tested by itself and entry is already authorized (3 of 20 agent sessions): skip (b2) |
| 4 | Re-test, authorize | A: prompt (b2) "Wait for the purge target, then re-test." (~12 s) | "Target reached: H2S about 4 ppm. Only now is entry authorized." | "probe rejected ... repositioning" / "a test without the probe inside the opening cannot authorize entry": ask "Check the ventilation status and re-test." "Purge incomplete — wait": wait the seconds it names, then ask again |
| 5 | Entry | A: prompt (c) | "Spot goes through the 1.3 meter door and follows fixed waypoints around the pipes, reading the air continuously. No person is in the space." (the camera cuts once to the inside of the tank) | Refused as stale (> 10 min since the test): ask it to re-test first |
| 6 | Excursion | B: `bash inject_gas.sh` once the HUD map shows Spot at waypoint 1, ~4 s into turn c (rehearsal: Enter 1.0 s after the waypoint, guard alarm 1.15-1.33 s after Enter in 4/5 final reps, 3.0 s once when the inject container started slowly; retreat 11.3-14.9 s) | "I am now staging a hydrogen sulfide release, a simulated spike that the HUD marks as staged. The safety guard is deterministic code, not the language model: it stops the robot within milliseconds and backs it out." | No retreat banner: check the HUD gas panel alarms; the scripted fallback is `--mission --inject-h2s-at-waypoint 2` |
| 7 | Explanation | (end of turn c) | Let the agent answer (it says the spike was staged by the presenter, 5/5); then: "The permit is aborted and re-entry needs a new permit and a new test." | Agent did not explain: ask "What just happened?" (explain_last_incident) |
| 8 | Audit | HUD: audit export / `scripts/fetch-log.sh` | "Every reading, decision and the staged spike are in the audit log." | |
| 9 | Clean up | B: `bash inject_gas.sh clear` then `bash reset_spot.sh --reset-ventilation` | | |

To end on COMPLETED instead (no spike): after (c), prompt "Walk back out to the
entry point." (rehearsal: entry turn 37-38 s with 5/5 waypoints, exit turn 43-44 s,
permit COMPLETED, Spot sitting at (7.8, 0.0)).

Never say: that Spot or its payload is ATEX / IECEx / intrinsically safe
certified; that any named operator uses this; that simulated readings are
field measurements; that the tank or the entry "is safe" (say "the readings
are within permissible entry limits"; the agent's prompt and its speech
post-filter enforce the same).

## Leadership rehearsal 2026-09-28 (GUI presentation mode)

Sim host, GUI launch (`run_sim.sh --background --gui`,
presentation mode, DCV server running, no viewer attached), HUD from
`run_hud.sh`, agent REPL as ubuntu under `script -qfc` (`tools/rehearse_rep.sh`
types the cheat-sheet prompts one at a time, 3 s after each answer), the spike
fired from a second, concurrent SSM command as ubuntu (`inject_gas.sh`, 1.0 s
after the log showed interior waypoint 1). Final build = commit 70a2eaf +
working tree with fixes 1-8 below. Logs on the host: `/opt/spot-demo/logs/lr7-*`
and `lr8-clean*` (`LABEL.jsonl`, `-transcript.txt`, `-timing.txt`,
`-spike.txt`, `odom-LABEL.jsonl`); per-rep summary
`docs/evidence/v5/lr-reps-summary.jsonl`.

Final build, 5 reps + clean runs (turn = prompt sent to the next `operator>`):

| Rep | a | b1 (hold) | b2 | c | Enter -> alarm | disclosure -> alarm | H2S | alarm -> retreat | Retreat | Door clearance | Re-test seat | Anomalies |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 32.3 s | 45.9 s (11.2 %) | 11.9 s | 28.6 s | 2.96 s | 54 ms | 77.7 ppm | 23 ms | 14.9 s | 0.208 m | step 169 -> 177 deg | the presenter's inject container took ~2.9 s (screenshots running), so Spot was deeper in |
| 2 | 33.5 s | 45.3 s (11.0 %) | 11.7 s | 23.7 s | 1.15 s | 23 ms | 77.5 ppm | 12 ms | 11.3 s | 0.235 m | step 169 -> 177 deg | none |
| 3 | 33.3 s | 44.9 s (11.0 %) | 11.9 s | 24.5 s | 1.15 s | 46 ms | 78.2 ppm | 12 ms | 12.5 s | 0.288 m | step 167 -> 176 deg | none |
| 4 | 32.7 s | 44.3 s (11.4 %) | 12.6 s | 25.1 s | 1.33 s | 123 ms | 77.4 ppm | 13 ms | 12.7 s | 0.267 m | skip (in envelope) | none |
| 5 | 32.5 s | 45.1 s (11.0 %) | 11.9 s | 24.1 s | 1.16 s | 53 ms | 78.8 ppm | 13 ms | 12.2 s | 0.186 m | skip (in envelope) | none |
| clean `lr7` | 32.8 s | 55.4 s (no hold) | 15.5 s | 38.3 s + exit 43.3 s | - | - | - | - | - | 0.251 m | step 167 -> 184 deg, 12 s | the step hunted +-4 deg until its timeout, the early re-test ran at 46 s and PASSED (no hold beat): fix 7 |
| clean `lr8` (after fix 7) | 33.0 s | 44.7 s (11.2 %) | 13.0 s | 36.9 s + exit 44.1 s | - | - | - | - | - | 0.257 m | step 169 -> 175 deg | none; COMPLETED, 5/5 waypoints, path 35.9 m |

Every incident rep: first test FAIL on H2S 51.5-51.6 ppm; the early re-test a
PURGE HOLD with every gas within limits (H2S about 6 ppm) and 11.0-11.4 %
remaining, the agent quoting the same percent as the HUD; the re-test PASS;
guard stop command 0.1 ms after detection (reading age 2-38 ms); retreat
complete, Spot sitting outside at x 7.73-7.78; permit ABORTED; the agent's
answer says the spike was presenter-staged (5/5). Spot faced the tank
(179-180 deg) at prompt (a) in all 7 sessions (fix 6). 0 probe rejections,
0 turn refusals. The word "safe" appears in none of the agent's spoken lines
(only "SafetyGuard"). RTF: step 4b.

An earlier series on the build before fixes 5-8 (`lr4-*`: 5 reps + clean)
gave the same storyline 6/6 (turns a 32-38 s, b1 44-46 s, b2 11-13 s, c
24-25 s; Enter -> alarm 1.13-1.42 s; retreat 12.1-12.8 s; door clearance
0.218-0.282 m); in its rep 4 the model waited out the hold and re-tested by
itself inside b1 (also seen in `lr2-rep3` and `lr5-clean`: 3 of 20 agent
sessions with this prompt), so b2 was a no-op.

Found and fixed during the rehearsal (runs before the fixes: `lr2-*` .. `lr6-*`):

1. **Re-seat hop turned Spot away from the door** (`lr3-rep1`): before the
   early re-test Spot stood 0.152 m from the seat target at yaw 169 deg (just
   outside the 10 deg envelope), the 1 cm unicycle hop swung it to 94 deg
   (body box 6 cm into the jamb zone by the odometry model), the facade
   rejected the probe, `face()` may not turn 69 deg that close to the door,
   and both later re-seats ended facing away (yaw -6 / -9 deg): entry stayed
   denied for the rest of the session. Fix: `agent/mission.py` `_seat_step`,
   short seat corrections (<= 0.5 m) strafe in the body frame (cmd_vel
   linear.y) while squaring the heading to the door. Host: 5/5 displaced
   re-tests stepped and probed, and 5 of the 7 final sessions used it live.
   Tests: `TestSeatedInProbeEnvelope` (the old hop fails them).
2. **HUD kept the previous run's ABORTED permit and trigger banner** at the
   next run's start: `hud/ros_bridge.py` `_new_session` on the presenter's
   `demo_reset` (step 8). Tests: `TestPresenterReset`.
3. **Headlight glare outside**: the HUD camera tile was a blown-out white
   floor at spawn and after the retreat, and the GUI showed a white hot spot on
   the sunlit ground. `sim/sensors.py` `PayloadLights` fades the payload lights
   in within 0.8-1.6 m of the shell (full at the door and inside). Tests:
   `TestPayloadLightFade`.
4. Rehearsal tooling: `tools/rehearse_rep.sh` (shell A), seat-decision audit
   events (`probe_seat`, hidden on the HUD), `tools/screen_watch.py` moments
   `authorized`, `mid`, `alarm`.
5. **The door beat was near-black**: the door face is on the shadow side of
   the tank. `sim/build_scene.py` entry work light (hidden emitter on a mast
   beside the door). Render cost within noise.
6. **After an exit or a retreat, the next run started with Spot turning its
   back to the tank**: the reset teleported it to spawn facing the tank, then
   the stance hold, which had latched the PRE-teleport heading from a pose
   not yet refreshed, turned it back (180 -> ~10 deg in 5 s; the camera then
   framed the pipe rack, not the tank). `sim/isaac_app.py` drops the stale
   pose on a policy restart. Host: 2/2 resets held 179-180 deg, then 7/7
   sessions.
7. **Seat step hunting** (`lr7-clean`): the 4 deg squaring tolerance was
   tighter than the policy's minimum turn nudge; the step hunted for its whole
   12 s timeout and the early re-test missed the hold window. Tolerance 6 deg
   (the envelope allows 10), timeout 8 s; `lr8-clean` then held at 11.2 % as
   designed.
8. HUD polish: the path trail restarts on a teleport (the reset drew a line
   door -> spawn on the next run's ready screen), and a normal exit logs
   "Entry complete: Spot exited the confined space and the permit is closed."
   instead of "Entry authorization withdrawn" under COMPLETED.
9. Not a code issue: `lr2-rep3` and final rep 1 had Enter -> alarm 3-4 s
   because the presenter's `inject_gas.sh` ros:jazzy container started slowly;
   the guard's own latency (disclosure -> alarm 23-123 ms) was unchanged.

## Known limitations

1. Nothing here has been checked against real `spot_ros2` or real hardware;
   the gas probe, blower and gas field are simulated.
2. The six probe points alone would pass at about 19 % contamination
   remaining (27 s into ventilation at tau 15 s): the simulated headspace
   varies with height, not depth, so the 1.0 m points read like the 0.5 m
   ones, and the route at z 0.1 m still has up to ~21 ppm H2S (GasField
   analysis, not measured on the host). Entry therefore also needs the
   purge target: after a failed test (or once the tank is ventilated) a
   passing re-test authorizes only at fraction_remaining <= 8 % (~48 s at
   tau 15 s; route <= 8.7 ppm); earlier it reports "Purge incomplete —
   wait" and the permit stays in pre-entry testing.
3. The gas probe is an extendable gas probe (simulated; arm-mounted on the
   real robot) with no arm model: no kinematics, collisions or motion time.
   The facade refuses a probe request without a body pose, with any point
   more than 1.1 m (horizontal) from the body's front, or behind the tank
   shell outside the 1.3 x 1.3 m doorway ("occluded by tank wall"); the rules
   are advertised in the `probe` block of payload/site. The 1.0 m-deep points
   need the nose within about 0.1 m of the door plane (body center at most
   0.65 m outside it, facing the tank). The real Spot Arm reaches ~0.98 m.
   After a walk the simulated policy, left on a zero command, used to creep
   ~40 deg in place within 12 s (the storyline then lost the probe during
   the ventilation wait and ABORTED); the facade now holds the standing
   heading (`sim/stance_hold.py`: 50 s idle within 0.1 deg) and the mission
   seats Spot ~3 cm nearer the door. Host 2026-09-27 after both: storyline
   COMPLETED (tests FAIL then PASS, 0 probe rejections, 151 s), excursion
   ABORTED on the staged H2S as designed, e2e 44/44, RTF 1.00.
4. Event `t` stamps are sim time for mission/agent/guard events; presenter
   events (host scripts) use sim time only when the script saw payload/status
   (`t_source`: sim / wall). Every presenter event also carries `wall_t`;
   order across sources by wall time (`wall_t` / `wall` / `rx_wall_utc`).
5. GUI (DCV) mode was rehearsed on 2026-09-28 in presentation mode (5 reps +
   1 clean run, see above) as ubuntu with the REPL on a real pty, but still
   without a DCV viewer attached (no streaming load in the RTF numbers) and
   with each instruction delivered as one line (no typing pauses). The Isaac
   window was checked by screenshots of the X display, not in a DCV client.
6. Presenter dry run 2026-09-27, found and fixed: after the ~45 s purge wait
   Spot stood 0.147 m from the probe-seat target (seat tolerance 0.14 m; the
   standing drift after the first seat), so the re-test hopped 1 cm; the
   policy's stop transient carried it 7 cm outward, the 1.0 m probe points
   went out of reach (1.10 m), the re-seat from 0.2 m away swung the heading
   175 -> 114 deg (body within 0.09 m of a jamb) and entry stayed denied until
   a second "re-test" prompt. The mission now skips the pre-test re-seat when
   the probe already works from where Spot stands (`_probe_seated`: every
   point within reach - 4 cm of the nose, facing the door within 10 deg,
   within 0.15 m of the door axis, nose outside the door plane); the
   re-seat after a facade rejection is unchanged, and still hops from close
   range (not reproduced after the fix: 0 rejections in 4 later re-tests).
7. `docker ps` no longer reports the sim "(unhealthy)": the Isaac image's
   HEALTHCHECK waits for a Kit "AppReady" line this app never prints, so
   `run_sim.sh` runs with `--no-healthcheck` (readiness is FACADE-READY).
8. The battery is a fake linear discharge of sim time (`sim/battery.py`,
   `--battery-drain demo|real|off`): `demo` (default) 25 %/h with a 30 % floor,
   `real` 100 % in 90 min (floor 5 %), `off`. Nothing consumes it; it only
   drives the HUD chip and what a model asked for status may say.
   `reset_spot.sh` recharges it.
9. The HUD at 1920x1080 used to cover its "Gas readings simulated" footer
   whenever the permit was PRE-ENTRY TESTING (the badge wrapped). Fixed in
   `hud/static/style.css` and checked on the host at 1080p
   (docs/evidence/14-hud-layout-fix-1080p.png). Other viewport sizes were not
   checked.
10. The agent and mission containers run as uid 10001 and write their JSONL
   logs to `/opt/spot-demo/logs/`, which `ubuntu` owns, through that
   directory's group. The directory is sticky, so a file in it can be renamed or
   deleted only by the file's owner, the directory's owner or root: for a
   mission log, any agent or mission container (they all run as uid 10001),
   `ubuntu` (and so every operator, through `sudo -iu ubuntu`) and root. The
   one-shot containers that write there through the group too (uid 10003: the
   rehearsal recorder and the screenshot capture) cannot. Each record carries
   the SHA-256 of the one before, so an edit inside a file shows, but a whole
   file can still be removed, cut short or rewritten on the host: that shows
   only against the `[audit head]` lines in the session log (step 9, "Check a
   log against the session log"). The session and mission-start records say
   that the file is a demonstration record of a simulated entry, not a
   regulatory permit record, name the account, the login that ran `sudo` and
   an SSM session id when there is one, and say whether the run was started
   with `--unattributed-run`.
11. The seat step (fix 1 of the leadership rehearsal) ran live in 5 of the 7
   final sessions (squared 166-169 -> 175-184 deg, 0 probe rejections) and
   in a scripted displacement check (5/5). The unrecoverable case (Spot turned
   more than 45 deg away within 0.9 m of the door) still has no automatic
   recovery: back it out with "Walk to the entry point." and re-test.
12. The model does not always stop at the purge hold: in 3 of 20 agent
   sessions with the (b1) prompt it waited the remaining seconds and
   re-tested by itself inside turn b1 (entry then authorized one prompt
   early). Harmless, but the presenter should skip (b2) when the HUD already
   shows ENTRY AUTHORIZED. The hold window is narrow by design (the early
   re-test must end 27-48 s after ventilation starts): a slow re-seat before
   it (fix 7) or a model that re-tests at once (a FAIL instead) misses it.
