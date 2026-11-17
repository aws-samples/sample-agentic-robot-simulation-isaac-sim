"""Threat model data-flow diagram: trust boundaries, flows and threat status.

Regenerate from the repository root:

    uv run --no-project --with diagrams==0.25.1 \
        python docs/diagrams/src/threat_dataflow.py

Writes docs/diagrams/exports/threat-dataflow.{svg,png} (PNG at 2x).

Drawn against the current state, after the hardening pass on the 63c05ef baseline
(threat model 0.5): one hand-built EC2 GPU host with no inbound security-group rules
(documented, no IaC), operators on SSM only, every container on the host network
with its own non-root uid, ROS 2 DDS on domain 42 bound to loopback with no SROS2,
and the LLM agent on Amazon Bedrock. A real Spot is not supported (the agent refuses
any backend but the simulator) and is drawn dashed: nothing there is built or tested.

Blue numbers are the system model's flow IDs (F1-F41). REGISTER mirrors the threat
register (severity band and status after the hardening pass); when the register
changes, change REGISTER in the same commit. Every High and Medium entry is badged on
the component it sits on; the Low entries and the flows that are not drawn are listed
under the legend.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from svgkit import (  # noqa: E402
    DATA,
    FIXED,
    IDENTITY,
    PARTIAL,
    PLATFORM,
    THREAT,
    Canvas,
)

OUT = Path(__file__).resolve().parents[1] / "exports" / "threat-dataflow"

# Threat register: ID -> (severity band, status after the hardening pass). Status:
# fixed (an effective, tested control closes the path), partial (part of the path
# is closed), open (nothing closes it).
REGISTER = {
    "T1": ("Medium", "partial"),
    "T2": ("Medium", "partial"),
    "T3": ("Medium", "partial"),
    "T4": ("Low", "fixed"),
    "T5": ("Low", "fixed"),
    "T6": ("Low", "fixed"),
    "T7": ("Low", "fixed"),
    "T8": ("Low", "fixed"),
    "T9": ("Medium", "partial"),
    "T10": ("Medium", "partial"),
    "T11": ("Medium", "partial"),
    "T12": ("Low", "fixed"),
    "T13": ("Low", "partial"),
    "T14": ("Low", "partial"),
    "T15": ("Low", "partial"),
    "T16": ("Medium", "partial"),
    "T17": ("Medium", "partial"),
    "T18": ("Low", "partial"),
    "T19": ("Low", "partial"),
    "T20": ("Medium", "partial"),
    "T21": ("Medium", "partial"),
    "T22": ("Medium", "partial"),
    "T23": ("Medium", "partial"),
    "T24": ("Medium", "partial"),
    "T25": ("Medium", "partial"),
    "T26": ("Medium", "partial"),
    "T27": ("Medium", "partial"),
    "T28": ("Medium", "partial"),
    "T29": ("Medium", "partial"),
    "T30": ("Low", "partial"),
    "T31": ("Medium", "partial"),
    "T32": ("Medium", "partial"),
    "T33": ("Medium", "partial"),
    "T34": ("Medium", "partial"),
    "T35": ("Low", "fixed"),
    "T36": ("Medium", "partial"),
    "T37": ("Medium", "partial"),
    "T38": ("Medium", "partial"),
    "T39": ("Medium", "partial"),
    "T40": ("Low", "fixed"),
    "T41": ("Low", "partial"),
    "T42": ("Medium", "partial"),
    "T43": ("Low", "partial"),
    "T44": ("Medium", "partial"),
}
COLOR = {"fixed": FIXED, "partial": PARTIAL, "open": THREAT}

# One-line titles for the badged IDs (every High and Medium entry).
BADGED = {
    "T1": "DCV: root-equivalent desktop behind one shared password",
    "T2": "HUD: no auth, Host allowlist or connection caps",
    "T3": "HUD trusts any publisher's event source, gas flags, frames",
    "T9": "Sim framing, presenter tools, overrides not tied to the sim",
    "T10": "Narration unchecked: denylist filter only, escapes pass",
    "T11": "Bedrock model choice unvalidated, untested, data use undisclosed",
    "T16": "SafetyGuard dies with the agent process (Ctrl-C, rm -f)",
    "T17": "Retreat fragile: one failed leg or a pose gap abandons it",
    "T20": "Any DDS participant can drive, claim, power, un-e-stop",
    "T21": "Spoofed payload/gas or probe_result hides a gas hazard",
    "T22": "Spoofed odometry or site redirects routes, disarms the guard",
    "T23": "World controls (inject, clear, vent, battery) accept anyone",
    "T24": "Unencrypted DDS broadcasts transcript, permit, camera, pose",
    "T25": "No DDS isolation: domain 0, subnet discovery, no SROS2",
    "T26": "Every host-network container can take the role credentials",
    "T27": "No IaC; the role can read an NGC key nothing needs",
    "T28": "Containers are the only boundary: X11, root, rw mounts",
    "T29": "Every operator can become root; DCV and Run Command not session-logged",
    "T31": "Images, packages, assets by mutable tag; no digest or hash",
    "T32": "Known-vulnerable components; no scan or patch cadence",
    "T33": "Public PRs pass no CI, scanners or CODEOWNERS",
    "T34": "Internal AWS identifiers in docs, defaults and history",
    "T36": "fetch-log writes to an un-ignored logs/; captures unredacted",
    "T37": "Attendant names spread everywhere; no minimisation, retention",
    "T38": "Audit trail: editable on the host; anchored in the session log",
    "T39": "Hand-built GPU host; optional budget and idle stop; no g6 recovery",
    "T42": "DDS flood slows the sim, evicts alarms, starves telemetry",
    "T44": "Optional livestream / Foxglove listeners unauthenticated",
}
SEVERITY_ORDER = {"High": 0, "Medium": 1, "Low": 2}

c = Canvas(1440, 1520)


def zone(x1, y1, x2, y2, label, where="top-left"):
    c.box(
        x1, y1, x2, y2, label, stroke=THREAT, dash="8 5", width=2, where=where, size=14
    )


def badge(x, y, tid):
    _, status = REGISTER[tid]
    c.badge(x, y, tid, COLOR[status], pill=True)


def threat(x, y, *ids, step=46):
    for i, tid in enumerate(ids):
        badge(x + i * step, y, tid)


def flow(x, y, n):
    c.badge(x, y, n, DATA)


# ------------------------------------------------------------ boundaries ----
zone(15, 70, 225, 640, "Operator Mac · TZ1")
zone(15, 660, 225, 900, "Public GitHub · TZ0", where="bottom-right")
zone(245, 70, 455, 900, "AWS control plane · TZ2")
zone(
    475,
    70,
    1095,
    900,
    "EC2 host g6.4xlarge · TZ3 · SG with no inbound rules (documented, no IaC)",
    where="bottom-left",
)
zone(680, 160, 1080, 875, "Host network · ROS 2 domain 42, loopback · TZ4")
zone(1115, 70, 1425, 300, "Public internet · TZ0")
zone(1115, 320, 1425, 560, "Model provider · TZ5")
c.box(
    1115,
    580,
    1425,
    900,
    "Future real-robot site · TZ6\nnot supported, not built",
    stroke=PLATFORM,
    dash="8 5",
    width=2,
    size=14,
)
# SSM: every operator path into the host passes through it.
c.box(265, 105, 435, 470, "", stroke="#e7157b", fill="#fdf2f8")
# DDS: one flat bus on the host loopback, no authentication between participants.
c.box(700, 370, 1060, 412, "", stroke=THREAT, fill="#fdecea", width=2)
c.text(
    880,
    396,
    "DDS bus /demo_spot · loopback · no auth or encryption",
    size=12,
    color=THREAT,
    weight="bold",
)
c.box(
    695,
    470,
    1065,
    865,
    "spot-agent container · uid 10001",
    stroke="#8c4fff",
    fill="#f7f3ff",
    size=13,
)

# ----------------------------------------------------------------- nodes ----
c.node(
    120,
    130,
    "aws/general/user",
    "Operator · AWS CLI\nssmrun · deploy · fetch-log",
    size=44,
    font=13,
)
c.node(
    120, 290, "aws/general/client", "Browser 1 · DCV\nlocalhost:8443", size=44, font=13
)
c.node(
    120, 420, "aws/general/client", "Browser 2 · HUD\nlocalhost:8080", size=44, font=13
)
c.node(
    120,
    550,
    "aws/general/disk",
    "Repo checkout\nlogs/ · docs/evidence",
    size=44,
    font=13,
)
c.node(
    60, 735, "onprem/vcs/github", "aws-samples\nMIT-0", where="right", size=40, font=13
)
c.node(
    60,
    850,
    "aws/general/users",
    "Adopters copy\ndefaults, RUNBOOK",
    where="right",
    size=40,
    font=12,
)

c.node(
    350,
    200,
    "aws/management/systems-manager",
    "Systems Manager\nport-fwd · shell\nRun Command = root",
    size=38,
    font=12,
)
c.node(
    350,
    580,
    "aws/security/identity-and-access-management-iam-role",
    "IAM instance role",
    size=42,
    font=12,
)
c.node(
    350,
    760,
    "aws/security/secrets-manager",
    "Secrets Manager\nDCV password",
    size=42,
    font=12,
)

c.node(
    565,
    130,
    "generic/os/ubuntu",
    "Ubuntu 24.04 · Docker\nubuntu in docker group",
    size=44,
    font=12,
)
c.node(
    565,
    290,
    "aws/enduser/desktop-and-app-streaming",
    "NICE DCV :8443\nloopback once configured",
    size=44,
    font=12,
)
c.node(
    565,
    580,
    "aws/security/identity-and-access-management-iam-temporary-security-credential",
    "IMDSv2\nhop limit 1 (doc)\nguard: root, 10001",
    size=44,
    font=12,
)
c.node(
    565,
    760,
    "aws/general/disk",
    "/opt/spot-demo\ncode · logs · audit",
    size=44,
    font=12,
)

c.node(
    745,
    250,
    "aws/robotics/robomaker-simulator",
    "spot-sim · uid 1234\nIsaac Sim · facade",
    size=44,
    font=12,
)
c.node(
    880,
    250,
    "onprem/container/docker",
    "Presenter one-shots\ninject_gas · reset",
    size=40,
    font=12,
)
c.node(
    1015,
    250,
    "programming/framework/fastapi",
    "spot-hud · uid 10002\n127.0.0.1:8080",
    size=44,
    font=12,
)

c.node(
    760,
    560,
    "programming/language/python",
    "LLM agent · 18 tools\nconsole confirmations",
    size=46,
    font=12,
)
c.node(
    760,
    760,
    "aws/robotics/robomaker",
    "mission · permit gate\nSafetyGuard 20 Hz",
    size=46,
    font=12,
)
c.node(
    1000,
    680,
    "aws/general/generic-sdk",
    "RobotClient\ntyped data\nsim gate · census",
    size=44,
    font=12,
)

c.node(
    1150,
    115,
    "onprem/container/docker",
    "nvcr.io · ECR Public · MCR\nby digest · PyPI · npm · apt",
    where="right",
    size=40,
    font=13,
)
c.node(
    1150,
    215,
    "aws/storage/simple-storage-service-s3-bucket",
    "NVIDIA asset root\n+ telemetry (opt-in)",
    where="right",
    size=40,
    font=13,
)
c.node(
    1150,
    430,
    "aws/ml/bedrock",
    "Amazon Bedrock\nClaude Sonnet 5 (us.)\nguardrail opt-in · session cap",
    where="right",
    size=44,
    font=13,
)
c.node(
    1150,
    680,
    "aws/robotics/robomaker-cloud-extension-ros",
    "spot_ros2 driver\n/<spot_name>",
    where="right",
    size=44,
    font=13,
)
c.node(
    1150,
    805,
    "aws/robotics/robotics",
    "Physical Spot · not ATEX\nreal detector · people",
    where="right",
    size=44,
    font=13,
)

# ----------------------------------------------------------------- flows ----
# 3-6: operator shell, deploy, fetch-log and ssmrun, all through SSM, all root.
c.edge([(142, 130), (543, 130)], tail=True)
flow(172, 130, "3")
flow(200, 130, "4")
flow(455, 130, "5")
flow(505, 130, "6")
c.text(350, 123, "shell · Run Command", size=12, color=DATA, bg=True)

# 2: the operator reads the DCV password; 20: the shipped role reads no secret.
c.edge([(142, 145), (236, 145), (236, 760), (327, 760)], color=IDENTITY)
flow(236, 660, "2")
c.edge([(543, 580), (373, 580)], color=IDENTITY)
c.text(458, 572, "role creds", size=12, color=IDENTITY, bg=True)
c.edge([(350, 645), (350, 737)], color=IDENTITY, dash="6 4")
flow(350, 690, "20")
c.text(337, 690, "no secret read in\nthe shipped policy", size=11, color=IDENTITY, anchor="end")

# 7 and 9: SSM port forwards to DCV and to the HUD on the host loopback.
c.edge([(142, 290), (543, 290)])
flow(455, 290, "7")
c.edge([(142, 420), (465, 420), (465, 95), (1015, 95), (1015, 226)])
flow(190, 420, "9")
c.text(745, 90, "HTTP + WS · Host and Origin allowlist, no auth", size=12, color=DATA, bg=True)

# 23-25: image, package and asset pulls; telemetry only if the operator opts in.
c.edge([(587, 120), (1128, 120)], color=THREAT)
flow(640, 120, "23")
c.edge([(745, 226), (745, 205), (1128, 205)], color=THREAT)
flow(860, 205, "24")
flow(890, 205, "25")

# 10 + 30: docker run on the host network, then the operator> REPL over -it.
c.edge([(587, 145), (668, 145), (668, 552), (735, 552)])
flow(668, 380, "10")
flow(668, 420, "30")

# 26: Isaac draws on the autologin desktop through the X socket.
c.edge([(721, 262), (640, 262), (640, 300), (588, 300)])
flow(700, 262, "26")

# 12, 14, 15: sim, presenter tools and HUD on the bus.
c.edge([(745, 312), (745, 368)], tail=True)
flow(745, 340, "12")
c.edge([(880, 312), (880, 368)])
flow(880, 340, "14")
c.edge([(1015, 370), (1015, 312)])
flow(1015, 340, "15")

# 11, 13, 16: RobotClient is the only code that talks to the robot.
c.edge([(1000, 414), (1000, 656)], tail=True)
flow(1000, 445, "11")
flow(1000, 590, "13")
flow(1000, 622, "16")

# 17-18: prompts and tool results out, tool-use proposals back.
c.edge([(785, 545), (1105, 545), (1105, 430), (1126, 430)], tail=True, color=THREAT)
flow(870, 545, "17")
flow(900, 545, "18")
c.edge([(760, 632), (760, 735)], color=PLATFORM)
c.text(772, 690, "proposals; gates decide", size=11, color=PLATFORM, anchor="start")
c.edge([(784, 770), (945, 770), (945, 680), (977, 680)], color=PLATFORM)

# 19: the IMDS guard admits only root and the agent's uid on the host network.
c.edge([(695, 600), (588, 600)], color=THREAT)
flow(612, 600, "19")
c.text(662, 590, "agent, root", size=11, color=THREAT, bg=True)

# 21: the hash-chained audit JSONL, written to the host's logs directory.
c.edge([(735, 760), (588, 760)])
flow(655, 760, "21")

# 28-29: publication, then adopters copying defaults.
c.edge([(98, 550), (30, 550), (30, 735), (38, 735)])
flow(30, 640, "28")
c.edge([(60, 757), (60, 828)])
flow(60, 792, "29")

# 32-33: the agent against a real driver: refused by the simulator gate (dashed).
c.edge([(1022, 680), (1128, 680)], color=PLATFORM, dash="6 4")
flow(1105, 680, "32")
c.edge([(1150, 704), (1150, 781)], color=PLATFORM, dash="6 4")
flow(1150, 742, "33")

# ---------------------------------------------------------------- badges ----
badge(190, 550, "T36")
badge(195, 715, "T33")
badge(195, 743, "T34")
badge(395, 200, "T29")
badge(290, 580, "T27")
badge(520, 250, "T1")
threat(520, 212, "T28", "T32", "T39")
badge(520, 540, "T26")
threat(520, 720, "T37", "T38")
badge(795, 338, "T44")
badge(930, 338, "T23")
badge(950, 238, "T2")
badge(950, 266, "T3")
threat(718, 440, "T20", "T21", "T22", "T24", "T25", "T42")
badge(820, 518, "T9")
badge(1040, 518, "T10")
threat(720, 845, "T16", "T17")
badge(1180, 510, "T11")
badge(1390, 115, "T31")

c.text(
    15,
    42,
    "Spot confined-space demo · data flows, trust boundaries and threat status, "
    "after the hardening pass (baseline 63c05ef)",
    size=18,
    weight="bold",
    anchor="start",
)

# ---------------------------------------------------------------- legend ----
y = 940
c.text(
    15,
    y,
    "Threat register, after the hardening pass",
    size=15,
    weight="bold",
    anchor="start",
)
key = [("fixed", "Fixed"), ("partial", "Partially fixed"), ("open", "Open")]
x = 420
for status, label in key:
    c.badge(x, y - 5, "T", COLOR[status])
    c.text(x + 18, y, label, size=14, anchor="start")
    x += 42 + len(label) * 7.6
flow(x + 8, y - 5, "1")
c.text(x + 26, y, "flow ID (system model F1-F41)", size=14, anchor="start")

rows = sorted(BADGED, key=lambda t: (SEVERITY_ORDER[REGISTER[t][0]], int(t[1:])))
half = (len(rows) + 1) // 2
for i, tid in enumerate(rows):
    col, row = divmod(i, half)
    bx = 32 + col * 710
    by = y + 34 + row * 24
    badge(bx, by - 5, tid)
    c.text(bx + 27, by, REGISTER[tid][0], size=13, anchor="start", color=PLATFORM)
    c.text(bx + 85, by, BADGED[tid], size=14, anchor="start")

y2 = y + 34 + half * 24 + 8
notes = [
    "Low, not badged: T4, T5, T6, T7, T8, T12, T35 and T40 (fixed); T13, T14, T15, T18, "
    "T19, T30, T41 and T43 (partially fixed). The register gives each one's rating.",
    "Not drawn: F1 EC2 start/stop, F8 DCV console session, F22 sim log, F27 tools/ "
    "screenshots, F31 laptop dry run (inset), F34 real gas detector,",
    "F35 account cleanup automation stop, F36 GDM and desktop settings, F37 account "
    "logs, F38 SSM command history, F39-F41 session logs, their export, the idle-stop check;",
    "nor the spot-rec and e2e containers (C39), the idle-stop timer (C40) or the optional "
    "guardrail and session-log stacks (C41, D13).",
    "Red edges carry untrusted or unauthenticated input; purple carry credentials; grey "
    "are in-process or future. The bus stops at the host's loopback (TB5) but has no "
    "boundary between participants.",
]
for i, note in enumerate(notes):
    c.text(15, y2 + i * 20, note, size=13, anchor="start")

# Laptop mode inset.
iy = y2 + 20 * len(notes) + 10
c.box(15, iy, 1425, iy + 60, "", stroke=PLATFORM, dash="5 4", fill="#fafafa")
c.text(
    28,
    iy + 23,
    "Laptop mode (M3, no AWS host):",
    size=14,
    weight="bold",
    anchor="start",
)
c.text(
    252,
    iy + 23,
    "unit tests with fakes; HUD --fake or --replay on 127.0.0.1; agent --dry-run, "
    "which still calls Bedrock (F31)",
    size=14,
    anchor="start",
)
c.text(
    28,
    iy + 45,
    "with the laptop's default AWS credential chain. The same prompt content, operator "
    "text and tool results leave the laptop as leave the host (T11, T37).",
    size=14,
    anchor="start",
)

c.save(OUT)
