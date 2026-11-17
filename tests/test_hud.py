"""Unit tests for the presenter HUD (hud/server.py + hud/ros_bridge.py).

Runs on a Mac WITHOUT rclpy: every test drives the app through FakeBridge
(realtime=False, so scenario time is advanced explicitly with tick(t)) and
FastAPI's in-process TestClient -- no sockets, no sleeps on the happy path.

Run from the repo root:

    python3 -m unittest tests.test_hud -v

Needs: fastapi, httpx (TestClient), pillow.
"""

import array
import io
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

from fastapi.testclient import TestClient
from PIL import Image
from starlette.websockets import WebSocketDisconnect

from hud import server

rb = server.ros_bridge

SNAPSHOT_KEYS = {"server", "permit", "status", "gas", "ventilation", "probe", "purge_hold", "pose",
                 "site", "camera", "link", "alert", "thresholds", "events_seq", "trail_epoch",
                 "ingest", "guard", "world", "events", "trail"}


# The HUD answers only Host localhost/127.0.0.1:<port> and /ws needs the
# page's own Origin (T2), so every test talks to it the way the browser does.
HUD_URL = "http://localhost:8080"
WS_URL = "ws://localhost:8080/ws"


def hud_client(app):
    """TestClient whose requests carry the browser's Host (localhost:8080)."""
    return TestClient(app, base_url=HUD_URL)


def ws_headers(origin=HUD_URL):
    return {"origin": origin}   # a fresh dict: websocket_connect adds to it


def make_client(t0=0.0):
    """TestClient around a manually-ticked FakeBridge (lifespan runs on enter)."""
    bridge = rb.FakeBridge(realtime=False)
    bridge.tick(t0)
    return bridge, hud_client(server.create_app(bridge))


def receive_until(ws, predicate, max_messages=60):
    """Read WS messages until predicate(msg) is true; bounded so a bug fails
    the test instead of hanging it (the server sends a diff every 0.25 s)."""
    seen = []
    for _ in range(max_messages):
        msg = ws.receive_json()
        seen.append(msg)
        if predicate(msg):
            return msg, seen
    raise AssertionError("predicate never matched; types seen: %s" % [m["type"] for m in seen])


def _fake_ros_modules():
    """Minimal stand-ins for rclpy + message packages (Mac has no ROS).

    Only the calls RosBridge makes; the node records every subscription so
    a test can check topics/QoS and push messages through the callbacks.
    """
    class Enum:
        def __init__(self, name):
            self.name = name

        def __repr__(self):
            return self.name

    class QoSProfile:
        def __init__(self, **kw):
            self.__dict__.update(kw)

    class Node:
        def __init__(self, name, namespace):
            self.name, self.namespace, self.subs, self.destroyed = name, namespace, {}, False

        def create_subscription(self, msg_type, topic, callback, qos):
            self.subs[topic] = (msg_type, callback, qos)

        def destroy_node(self):
            self.destroyed = True

    class Context:
        def __init__(self):
            self._ok = False

        def ok(self):
            return self._ok

        def try_shutdown(self):
            self._ok = False

    class Executor:
        def __init__(self, context):
            self.context, self.nodes = context, []

        def add_node(self, node):
            self.nodes.append(node)

        def spin_once(self, timeout_sec):
            time.sleep(timeout_sec)

        def shutdown(self, timeout_sec):
            pass

    rclpy = types.ModuleType("rclpy")
    rclpy.nodes = []
    rclpy.init_kwargs = []

    def init(context=None, **kw):
        rclpy.init_kwargs.append(kw)
        context._ok = True

    def create_node(name, namespace=None, context=None):
        node = Node(name, namespace)
        rclpy.nodes.append(node)
        return node
    rclpy.init, rclpy.create_node = init, create_node
    mods = {"rclpy": rclpy}
    for sub, attrs in {
        "context": {"Context": Context},
        "executors": {"SingleThreadedExecutor": Executor},
        "qos": {"QoSProfile": QoSProfile, "qos_profile_sensor_data": QoSProfile(name="sensor"),
                "QoSDurabilityPolicy": types.SimpleNamespace(
                    TRANSIENT_LOCAL=Enum("TRANSIENT_LOCAL"), VOLATILE=Enum("VOLATILE")),
                "QoSHistoryPolicy": types.SimpleNamespace(KEEP_LAST=Enum("KEEP_LAST")),
                "QoSReliabilityPolicy": types.SimpleNamespace(RELIABLE=Enum("RELIABLE"))},
        "signals": {"SignalHandlerOptions": types.SimpleNamespace(NO=Enum("NO"))},
    }.items():
        module = types.ModuleType("rclpy." + sub)
        module.__dict__.update(attrs)
        mods["rclpy." + sub] = module
        setattr(rclpy, sub, module)
    for pkg, cls in (("nav_msgs", "Odometry"), ("sensor_msgs", "Image"), ("std_msgs", "String")):
        top, msg = types.ModuleType(pkg), types.ModuleType(pkg + ".msg")
        setattr(msg, cls, type(cls, (), {}))
        top.msg = msg
        mods[pkg], mods[pkg + ".msg"] = top, msg
    return mods


class TestApiState(unittest.TestCase):
    def test_state_shape(self):
        bridge, client = make_client(0.0)
        for i in range(1, 25):
            bridge.tick(0.5 * i)  # walk spawn -> entry so the trail fills in
        with client:
            resp = client.get("/api/state")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.headers["cache-control"], "no-store")
        snap = resp.json()
        self.assertEqual(set(snap), SNAPSHOT_KEYS)
        # Strict JSON (browsers reject NaN/Infinity).
        json.dumps(snap, allow_nan=False)
        self.assertEqual(snap["server"]["mode"], "fake")
        self.assertEqual(snap["permit"]["state"], "PRE_ENTRY_TESTING")
        self.assertEqual(set(snap["status"]), {"claimed", "powered", "standing", "standing_up",
                                               "estopped", "battery_pct", "sim_time"})
        self.assertEqual(set(snap["gas"]), {"o2_pct", "lel_pct", "h2s_ppm", "co_ppm", "alarms", "alarm",
                                            "ventilation", "injected_pockets"})
        self.assertEqual(set(snap["gas"]["alarms"]), {"o2", "lel", "h2s", "co", "any"})
        self.assertEqual(set(snap["pose"]), {"x", "y", "z", "yaw", "speed"})
        self.assertAlmostEqual(snap["pose"]["x"], 7.0, places=2)   # waiting at the entry point
        self.assertEqual(snap["site"]["source"], "topic")
        for key in ("spawn_xy", "entry_xy", "door_xy", "door_normal_yaw", "interior_waypoints",
                    "tank_center_xy", "tank_radius"):
            self.assertIn(key, snap["site"])
        self.assertEqual(snap["thresholds"]["h2s"]["alarm_at"], 10.0)
        seqs = [ev["seq"] for ev in snap["events"]]
        self.assertEqual(seqs, sorted(seqs))
        self.assertEqual(snap["events_seq"], seqs[-1])
        for ev in snap["events"]:
            self.assertTrue({"t", "source", "kind", "message", "data", "seq", "rx_wall"} <= set(ev))
        self.assertGreater(len(snap["trail"]), 10)

    def test_alarm_after_20s_and_abort(self):
        bridge, client = make_client(0.0)
        for t in (5.0, 12.0, 16.0, 19.9):
            bridge.tick(t)
        snap = bridge.state.snapshot()
        self.assertFalse(snap["gas"]["alarm"], snap["gas"])
        self.assertEqual(snap["permit"]["state"], "ENTRY_IN_PROGRESS")
        self.assertEqual(snap["alert"]["seq"], 0)
        t = 20.0
        while t < 20.6:
            t += 0.05
            bridge.tick(t)
        snap = bridge.state.snapshot(include_events=True)
        self.assertTrue(snap["gas"]["alarms"]["h2s"], snap["gas"])
        self.assertEqual(snap["permit"]["state"], "ABORTED")
        # Gas edge, guard event and ABORTED are one incident -> one flash.
        self.assertEqual(snap["alert"]["seq"], 1)
        self.assertIn("H2S", snap["alert"]["reason"])
        guard = [ev for ev in snap["events"] if ev["source"] == "guard"]
        self.assertEqual(len(guard), 1)
        self.assertTrue(guard[0]["data"]["alarm"])

    def test_retreat_leaves_the_tank(self):
        bridge, _client = make_client(0.0)
        for t in range(0, 31):
            bridge.tick(float(t))
        pose = bridge.state.snapshot()["pose"]
        self.assertGreater(pose["x"], 6.0)  # back outside the doorway
        self.assertFalse(bridge.state.snapshot()["status"]["standing"])


class TestStaticAndCamera(unittest.TestCase):
    def test_index_and_assets_served(self):
        _bridge, client = make_client()
        with client:
            index = client.get("/")
            js = client.get("/static/app.js")
            css = client.get("/static/style.css")
            traversal = client.get("/static/../server.py")
        self.assertEqual(index.status_code, 200)
        self.assertIn("text/html", index.headers["content-type"])
        self.assertIn("/static/app.js", index.text)
        self.assertIn("Simulation", index.text)
        self.assertIn("Gas readings simulated", index.text)
        self.assertIn("default-src 'self'", index.headers["content-security-policy"])
        self.assertEqual(js.status_code, 200)
        self.assertEqual(css.status_code, 200)
        self.assertNotEqual(traversal.status_code, 200)

    def test_left_column_cannot_cover_the_footer_disclosures(self):
        # Presenter dry run 2026-09-27 at 1920x1080: "PRE-ENTRY TESTING"
        # wrapped to two lines and the gauges ran over the footer's
        # "Gas readings simulated". Static guard (the pixel check was a
        # headless-Chromium screenshot on the host): one-line badge, and the
        # left column clips instead of overflowing.
        import re
        css = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "hud", "static", "style.css")).read()
        badge = re.search(r"\.permit-state\s*\{([^}]*)\}", css).group(1)
        self.assertIn("white-space: nowrap", badge)
        size = float(re.search(r"font-size:\s*([0-9.]+)rem", badge).group(1))
        self.assertLessEqual(size, 1.6)
        left = re.search(r"\.col-left\s*\{([^}]*)\}", css).group(1)
        self.assertIn("overflow: hidden", left)

    def test_camera_endpoint_returns_jpeg(self):
        _bridge, client = make_client(0.0)
        with client:
            resp = client.get("/api/camera.jpg")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.headers["content-type"], "image/jpeg")
        self.assertEqual(resp.content[:2], b"\xff\xd8")
        img = Image.open(io.BytesIO(resp.content))
        self.assertEqual(img.format, "JPEG")
        self.assertLessEqual(img.width, 800)

    def test_camera_503_before_first_frame(self):
        state = rb.HudState(mode="fake")
        bridge = rb.FakeBridge(state=state, realtime=False)  # never ticked -> no frame
        with hud_client(server.create_app(bridge)) as client:
            resp = client.get("/api/camera.jpg")
        self.assertEqual(resp.status_code, 503)


class TestWebSocket(unittest.TestCase):
    def test_hello_then_events_pushed(self):
        bridge, client = make_client(0.0)
        with client, client.websocket_connect(WS_URL, headers=ws_headers()) as ws:
            hello = ws.receive_json()
            self.assertEqual(hello["type"], "hello")
            self.assertTrue(SNAPSHOT_KEYS <= set(hello["state"]))
            first_seq = hello["state"]["events_seq"]
            bridge.tick(1.5)  # emits the agent 'say' + tool_call events
            msg, _ = receive_until(ws, lambda m: m["type"] == "events")
            kinds = [ev["kind"] for ev in msg["events"]]
            self.assertIn("say", kinds)
            self.assertTrue(all(ev["seq"] > first_seq for ev in msg["events"]))

    def test_diff_carries_alert_and_permit(self):
        bridge, client = make_client(0.0)
        for t in (5.0, 12.0, 16.0, 19.9):
            bridge.tick(t)
        with client, client.websocket_connect(WS_URL, headers=ws_headers()) as ws:
            self.assertEqual(ws.receive_json()["type"], "hello")
            for i in range(1, 15):
                bridge.tick(19.9 + 0.05 * i)
            msg, _ = receive_until(
                ws, lambda m: m["type"] == "diff" and m["changes"].get("alert", {}).get("seq", 0) >= 1)
            self.assertNotIn("events", msg["changes"])  # streamed separately
            self.assertNotIn("trail", msg["changes"])
            msg, _ = receive_until(
                ws, lambda m: m["type"] == "diff"
                and m["changes"].get("permit", {}).get("state") == "ABORTED")

    def test_foreign_origin_rejected(self):
        _bridge, client = make_client()
        with client:
            with self.assertRaises(WebSocketDisconnect):
                with client.websocket_connect(WS_URL, headers=ws_headers("http://evil.example")) as ws:
                    ws.receive_json()


class TestIngest(unittest.TestCase):
    def test_bad_payloads_do_not_raise(self):
        state = rb.HudState()
        state.ingest_gas_json("{not json")
        state.ingest_status_json("[1, 2]")
        state.ingest_site_json(json.dumps({"tank_radius": -1}))
        state.ingest_event_json("\"just a string\"")
        snap = state.snapshot(include_events=True)
        self.assertIsNone(snap["gas"])
        self.assertIsNone(snap["status"])
        self.assertIsNone(snap["site"])
        self.assertEqual(snap["events"][0]["kind"], "raw")


    def test_nan_sanitized(self):
        state = rb.HudState()
        state.ingest_gas_json('{"o2_pct": NaN, "lel_pct": Infinity, "h2s_ppm": 1, "co_ppm": 0,'
                              ' "alarms": {"any": false}, "alarm": false}')
        json.dumps(state.snapshot(), allow_nan=False)
        self.assertIsNone(state.snapshot()["gas"]["o2_pct"])

    def test_permit_and_alert_rules(self):
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit", "message": "go",
                         "data": {"permit_state": "entry_in_progress", "permit_id": "P-1"}})
        snap = state.snapshot()
        self.assertEqual(snap["permit"]["state"], "ENTRY_IN_PROGRESS")
        self.assertEqual(snap["permit"]["permit_id"], "P-1")
        state.add_event({"source": "guard", "kind": "armed", "message": "watching at 4 Hz"})
        self.assertEqual(state.alert_seq, 0)   # benign guard chatter never flashes
        state.add_event({"source": "mission", "kind": "retreat_complete", "message": "out"})
        self.assertEqual(state.alert_seq, 0)
        # Kinds emitted by agent/mission.py: permit via data.state, passed tag.
        state.add_event({"source": "mission", "kind": "permit", "message": "p",
                         "data": {"state": "PRE_ENTRY_TESTING", "permit_id": "P-2"}})
        self.assertEqual(state.snapshot()["permit"]["state"], "PRE_ENTRY_TESTING")
        self.assertEqual(state.alert_seq, 0)
        state.add_event({"source": "mission", "kind": "permit", "message": "aborted",
                         "data": {"permit_state": "ABORTED"}})
        self.assertEqual(state.alert_seq, 1)
        self.assertEqual(state.snapshot()["alert"]["title"], "ENTRY ABORTED")

    def test_retreat_incomplete_flashes(self):
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "retreat_incomplete", "message": "stuck"})
        self.assertEqual(state.alert_seq, 1)

    @staticmethod
    def _clocked_state():
        """HudState on a fake clock: returns (state, advance(seconds))."""
        now = [1000.0]
        state = rb.HudState(wall_clock=lambda: now[0], monotonic=lambda: now[0])

        def advance(dt):
            now[0] += dt
        return state, advance

    @staticmethod
    def _gas(h2s):
        alarm = h2s >= 10.0
        return json.dumps({"o2_pct": 20.9, "lel_pct": 1.0, "h2s_ppm": h2s, "co_ppm": 1.0,
                           "alarms": {"o2": False, "lel": False, "h2s": alarm, "co": False,
                                      "any": alarm}, "alarm": alarm})

    def test_guard_monitor_echoes_during_retreat_do_not_reflash(self):
        # Regression: agent/mission.py logs a "monitor" event (source guard,
        # data = the payload/gas reading, so data.alarm is true) every 5 s
        # while retreating through the pocket. Each one used to open a NEW
        # incident once 5 s had passed since the first flash, re-covering
        # the camera and map with the full-screen overlay during the retreat.
        state, advance = self._clocked_state()
        state.ingest_gas_json(self._gas(1.0))
        state.ingest_gas_json(self._gas(25.0))                  # rising edge -> flash 1
        state.add_event({"source": "guard", "kind": "alarm", "message": "GAS ALARM: H2S 25 ppm",
                         "data": {"reasons": ["H2S"], "auto_retreat": True}})
        state.add_event({"source": "guard", "kind": "permit", "message": "aborted",
                         "data": {"state": "ABORTED"}})
        state.add_event({"source": "guard", "kind": "retreat_start", "message": "RETREAT"})
        self.assertEqual(state.alert_seq, 1)
        for _ in range(6):                                     # ~31 s still in alarm
            for _ in range(21):
                advance(0.25)
                state.ingest_gas_json(self._gas(25.0))
            state.add_event({"source": "guard", "kind": "monitor", "message": "H2S 25 ppm",
                             "data": json.loads(self._gas(25.0))})
        self.assertEqual(state.alert_seq, 1)
        self.assertEqual(state.snapshot()["alert"]["title"], "SAFETY GUARD: RETREAT")
        # Out of the pocket, quiet for > 5 s: a new exposure is a new incident.
        state.ingest_gas_json(self._gas(1.0))
        advance(6.0)
        state.ingest_gas_json(self._gas(30.0))
        self.assertEqual(state.alert_seq, 2)

    def test_failed_retreat_and_estop_flash_inside_open_incident(self):
        state, advance = self._clocked_state()
        state.ingest_gas_json(self._gas(1.0))
        state.ingest_gas_json(self._gas(25.0))
        advance(1.0)
        state.ingest_gas_json(self._gas(25.0))
        state.add_event({"source": "guard", "kind": "retreat_incomplete",
                         "message": "Retreat INCOMPLETE - operator attention required"})
        self.assertEqual(state.alert_seq, 2)
        self.assertEqual(state.snapshot()["alert"]["title"], rb.RETREAT_FAILED_TITLE)
        status = {"claimed": True, "powered": True, "standing": True, "standing_up": False,
                  "battery_pct": 90.0, "sim_time": 1.0}
        state.ingest_status_json(json.dumps(dict(status, estopped=False)))
        state.ingest_status_json(json.dumps(dict(status, estopped=True)))   # rising edge
        self.assertEqual(state.alert_seq, 3)
        self.assertEqual(state.snapshot()["alert"]["title"], rb.ESTOP_TITLE)
        state.ingest_status_json(json.dumps(dict(status, estopped=True)))
        state.ingest_status_json(json.dumps(dict(status, estopped=False)))  # release: no flash
        self.assertEqual(state.alert_seq, 3)

    def test_estop_status_edge_only_after_seen_clear(self):
        state = rb.HudState()
        state.ingest_status_json(json.dumps({"estopped": True}))  # HUD started mid-E-stop
        self.assertEqual(state.alert_seq, 0)

    def test_release_kinds_and_rehearsal_title(self):
        state = rb.HudState()
        for kind in ("estop_release", "estop_released", "alarm_reset", "retreat_complete"):
            state.add_event({"source": "mission", "kind": kind, "message": kind})
        self.assertEqual(state.alert_seq, 0)
        state.add_event({"source": "guard", "kind": "alarm", "message": "GAS ALARM",
                         "data": {"auto_retreat": False}})
        self.assertEqual(state.snapshot()["alert"]["title"], "ALARM: AUTO-RETREAT OFF")

    def test_replayed_history_sets_permit_without_flashing(self):
        # Regression: payload/events is TRANSIENT_LOCAL; history delivered to
        # a restarted HUD must restore the permit but not flash old alarms.
        state = rb.HudState()
        state.ingest_event_json(json.dumps({"source": "mission", "kind": "permit", "message": "in",
                                            "data": {"state": "ENTRY_IN_PROGRESS"}}), replay=True)
        state.ingest_event_json(json.dumps({"source": "guard", "kind": "alarm", "message": "old",
                                            "data": {"alarm": True}}), replay=True)
        state.ingest_event_json(json.dumps({"source": "mission", "kind": "permit", "message": "x",
                                            "data": {"state": "ABORTED"}}), replay=True)
        snap = state.snapshot(include_events=True)
        self.assertEqual(snap["permit"]["state"], "ABORTED")
        self.assertEqual(len(snap["events"]), 3)
        self.assertEqual(state.alert_seq, 0)

    def test_truncated_event_data_keeps_permit_state(self):
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit", "message": "authorized",
                         "data": {"state": "ENTRY_AUTHORIZED", "permit_id": "P-9",
                                  "results": ["x" * 100] * 80}})
        snap = state.snapshot(include_events=True)
        self.assertEqual(snap["permit"]["state"], "ENTRY_AUTHORIZED")
        self.assertEqual(snap["permit"]["permit_id"], "P-9")
        self.assertTrue(snap["events"][-1]["data"]["_truncated"])
        self.assertLessEqual(len(json.dumps(snap["events"][-1]["data"])), rb.MAX_DATA_CHARS + 200)

    def test_ring_buffer_and_truncation(self):
        state = rb.HudState()
        for i in range(250):
            state.add_event({"source": "agent", "kind": "say", "message": "m%d" % i})
        snap = state.snapshot(include_events=True)
        self.assertEqual(len(snap["events"]), rb.EVENT_BUFFER)
        self.assertEqual(snap["events"][0]["message"], "m50")
        self.assertEqual([e["seq"] for e in state.events_since(248)], [249, 250])
        state.add_event({"source": "agent", "kind": "say", "message": "x" * 10000})
        self.assertEqual(len(state.events_since(250)[0]["message"]), rb.MAX_MESSAGE_CHARS)

    def test_listener_notified_and_errors_contained(self):
        state = rb.HudState()
        calls = []
        state.add_listener(lambda: calls.append(1))
        state.add_listener(lambda: 1 / 0)  # a dead client must not break ingest
        state.add_event({"source": "agent", "kind": "say", "message": "hi"})
        self.assertEqual(calls, [1])


def run_fake(bridge, until, step=0.05, start=0.0):
    """Tick the FakeBridge from ``start`` to ``until`` like the realtime loop."""
    t = start
    while t < until:
        t = round(t + step, 4)
        bridge.tick(t)


def mission_alarm_sequence(state, advance, h2s=25.3):
    """The event order agent/mission.py produces for a guard retreat, with
    the payload/gas edge the HUD sees on its own subscription."""
    reading = json.loads(TestIngest._gas(h2s))
    state.ingest_gas_json(TestIngest._gas(1.0))
    state.ingest_gas_json(TestIngest._gas(h2s - 5.0))          # edge: first sample in alarm
    advance(0.05)
    state.add_event({"source": "guard", "kind": "alarm",
                     "message": "GAS ALARM: H2S %.1f ppm >= 10 ppm" % h2s,
                     "data": {"reasons": ["H2S %.1f ppm >= 10 ppm" % h2s], "reading": reading,
                              "pose": [2.0, 1.0, 0.3], "auto_retreat": True}})
    state.add_event({"source": "guard", "kind": "permit", "message": "aborted",
                     "data": {"state": "ABORTED", "previous": "ENTRY_IN_PROGRESS",
                              "cause": "atmosphere", "reading": reading}})
    state.add_event({"source": "guard", "kind": "retreat_start", "message": "RETREAT: ...",
                     "data": {"cause": "atmosphere", "reading": reading}})
    return reading


class TestPresenterReset(unittest.TestCase):
    """Rehearsal 2026-09-28: after reset_spot.sh the HUD still showed the
    previous run's ABORTED permit, trigger banner, probe table and path at
    the next run's start. A live presenter demo_reset starts a fresh run."""

    def aborted_state(self):
        state, advance = TestIngest._clocked_state()
        state.add_event({"source": "agent", "kind": "permit_prepared", "message": "complete",
                         "data": {"permit": {"permit_id": "CSE-0427", "isolation": True,
                                             "loto": True, "attendant": "Alex Rivera",
                                             "rescue_plan": "standby team"}}})
        state.add_event({"source": "mission", "kind": "atmos_probe", "message": "rows",
                         "data": {"samples": [{"z": 0.2, "inside_m": 0.5, "o2_pct": 20.8,
                                              "lel_pct": 0.6, "h2s_ppm": 3.4, "co_ppm": 0.4}],
                                  "passed": True, "at": "door"}})
        for x in (14.0, 10.0, 6.5, 4.6):
            state.ingest_pose(x, 0.0)
        mission_alarm_sequence(state, advance)
        return state, advance

    def test_presenter_reset_starts_a_fresh_run(self):
        state, advance = self.aborted_state()
        # reset_spot.sh clears the gas first, and the retreat is over: a
        # reset during either is refused (tests/test_hardening_hud.py, T3).
        state.ingest_gas_json(TestIngest._gas(1.0))
        state.add_event({"source": "guard", "kind": "retreat_complete", "message": "out",
                         "data": {"cause": "atmosphere"}})
        before = state.snapshot(include_trail=True)
        self.assertEqual(before["permit"]["state"], "ABORTED")
        self.assertIsNotNone(before["probe"])
        self.assertTrue(before["trail"])
        events = len(state.snapshot(include_events=True)["events"])
        advance(5.0)
        state.add_event({"source": "presenter", "kind": "demo_reset", "message": "reset to spawn",
                         "data": {"reset_ventilation": True}})
        after = state.snapshot(include_trail=True, include_events=True)
        self.assertEqual(after["permit"]["state"], "NOT_STARTED")
        self.assertIsNone(after["permit"]["permit_id"])
        self.assertEqual(after["permit"]["checklist_summary"]["confirmed"], 0)
        self.assertIsNone(after["probe"])
        self.assertEqual(after["trail"], [])
        self.assertEqual(after["trail_epoch"], before["trail_epoch"] + 1)
        self.assertEqual(after["alert"]["seq"], before["alert"]["seq"])   # no new flash
        self.assertEqual(len(after["events"]), events + 1)                 # the log keeps all

    def test_teleport_to_spawn_restarts_the_path_trail(self):
        # The reset's odometry jump (door -> spawn) is not a walked path.
        state = rb.HudState()
        for x in (7.8, 7.7, 7.6):
            state.ingest_pose(x, 0.0)
        epoch = state.snapshot()["trail_epoch"]
        state.ingest_pose(14.0, 0.0)
        snap = state.snapshot(include_trail=True)
        self.assertEqual(snap["trail"], [[14.0, 0.0]])
        self.assertEqual(snap["trail_epoch"], epoch + 1)
        state.ingest_pose(13.8, 0.0)                     # walking again: no restart
        state.ingest_pose(13.6, 0.0)
        self.assertEqual(len(state.snapshot(include_trail=True)["trail"]), 3)

    def test_only_a_live_presenter_reset_clears_the_screen(self):
        for event, replay in (({"source": "mission", "kind": "demo_reset", "message": "x"}, False),
                              ({"source": "presenter", "kind": "demo_reset", "message": "x"}, True),
                              ({"source": "presenter", "kind": "demo_clear", "message": "x"}, False)):
            state, _advance = self.aborted_state()
            state.add_event(event, replay=replay)
            self.assertEqual(state.snapshot()["permit"]["state"], "ABORTED", (event, replay))


class TestPermitChecklist(unittest.TestCase):
    def test_fake_checklist_goes_from_not_set_to_confirmed(self):
        bridge, client = make_client(0.0)
        with client:
            before = client.get("/api/state").json()["permit"]
            bridge.tick(0.7)
            after = client.get("/api/state").json()["permit"]
        self.assertEqual([r["key"] for r in before["checklist"]],
                         ["isolation", "loto", "attendant", "rescue_plan"])
        self.assertEqual([r["ok"] for r in before["checklist"]], [False] * 4)
        self.assertEqual(before["checklist_summary"], {"confirmed": 0, "total": 4, "complete": False})
        self.assertEqual([r["ok"] for r in after["checklist"]], [True] * 4)
        self.assertTrue(after["checklist_summary"]["complete"])
        self.assertIn("attendant on station", after["checklist"][2]["text"])
        self.assertEqual(after["permit_id"], rb.FakeBridge.PERMIT_ID)
        self.assertEqual(after["state"], "NOT_STARTED")   # checklist events don't move the permit

    def test_absent_fields_are_unknown_not_failed(self):
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit", "message": "p",
                         "data": {"state": "PRE_ENTRY_TESTING", "permit_id": "P-1"}})
        permit = state.snapshot()["permit"]
        self.assertEqual([r["ok"] for r in permit["checklist"]], [None] * 4)
        self.assertEqual([r["text"] for r in permit["checklist"]], [None] * 4)
        self.assertFalse(permit["checklist_summary"]["complete"])

    def test_top_level_aliases_on_permit_kinds(self):
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit", "message": "prepared",
                         "data": {"state": "NOT_STARTED", "permit_id": "P-7",
                                  "isolation_confirmed": True, "loto_confirmed": "tag 12",
                                  "attendant": "J. Doe", "rescue_plan": ""}})
        rows = {r["key"]: r for r in state.snapshot()["permit"]["checklist"]}
        self.assertEqual((rows["isolation"]["ok"], rows["isolation"]["text"]), (True, "Confirmed"))
        self.assertEqual((rows["loto"]["ok"], rows["loto"]["text"]), (True, "tag 12"))
        self.assertEqual((rows["attendant"]["ok"], rows["attendant"]["text"]), (True, "J. Doe"))
        self.assertEqual(rows["rescue_plan"]["ok"], False)
        self.assertEqual(state.snapshot()["permit"]["checklist_summary"]["confirmed"], 3)

    def test_non_permit_kind_top_level_keys_ignored_but_nested_dict_read(self):
        state = rb.HudState()
        state.add_event({"source": "agent", "kind": "say", "message": "x",
                         "data": {"attendant": "Nobody"}})
        self.assertIsNone(state.snapshot()["permit"]["checklist"][2]["ok"])
        state.add_event({"source": "mission", "kind": "summary", "message": "x",
                         "data": {"permit": {"id": "P-3", "attendant": "R. Roe"}}})
        permit = state.snapshot()["permit"]
        self.assertTrue(permit["checklist"][2]["ok"])
        self.assertEqual(permit["permit_id"], "P-3")

    def test_fields_persist_across_transitions_and_reset_for_new_permit(self):
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit", "message": "prep",
                         "data": {"permit": {"permit_id": "P-7", "isolation": True, "loto": True,
                                             "attendant": "J. Doe", "rescue_plan": "Team B"}}})
        state.add_event({"source": "mission", "kind": "permit", "message": "testing",
                         "data": {"state": "PRE_ENTRY_TESTING", "permit_id": "P-7"}})
        permit = state.snapshot()["permit"]
        self.assertEqual(permit["state"], "PRE_ENTRY_TESTING")
        self.assertTrue(permit["checklist_summary"]["complete"])
        state.add_event({"source": "mission", "kind": "permit", "message": "new permit",
                         "data": {"state": "NOT_STARTED", "permit_id": "P-8"}})
        permit = state.snapshot()["permit"]
        self.assertEqual(permit["permit_id"], "P-8")
        self.assertEqual([r["ok"] for r in permit["checklist"]], [None] * 4)

    def test_nested_permit_state_used_when_top_level_absent(self):
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit", "message": "m",
                         "data": {"permit": {"state": "ENTRY_AUTHORIZED", "permit_id": "P-2"}}})
        self.assertEqual(state.snapshot()["permit"]["state"], "ENTRY_AUTHORIZED")

    def test_truncated_event_keeps_permit_dict(self):
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit", "message": "m",
                         "data": {"state": "ABORTED", "results": ["x" * 100] * 80,
                                  "permit": {"permit_id": "P-9", "attendant": None}}})
        permit = state.snapshot()["permit"]
        self.assertEqual(permit["permit_id"], "P-9")
        self.assertEqual(permit["checklist"][2]["ok"], False)

    def test_checklist_item_forms(self):
        item = rb.checklist_item
        self.assertEqual(item(None, present=False), {"ok": None, "text": None})
        self.assertEqual(item(None), {"ok": False, "text": "Not set"})
        self.assertEqual(item("pending")["ok"], False)
        self.assertEqual(item({"confirmed": True, "by": "Ops lead"}), {"ok": True, "text": "Ops lead"})
        self.assertEqual(item({"confirmed": False})["ok"], False)
        self.assertEqual(len(item("y" * 500)["text"]), 80)


class TestVentilation(unittest.TestCase):
    def test_normalize_variants(self):
        nv = rb.normalize_ventilation
        self.assertIsNone(nv(None))
        self.assertIsNone(nv([1, 2]))
        v = nv({"active": True, "fraction_remaining": 0.42, "tau_s": 15, "residual": 0.05})
        self.assertEqual((v["state"], v["fraction_remaining"], v["tau_s"], v["residual"]),
                         ("on", 0.42, 15.0, 0.05))
        self.assertEqual(nv("raw")["state"], "off")
        self.assertEqual(nv("unventilated")["state"], "off")
        self.assertEqual(nv("ventilating")["state"], "on")
        self.assertEqual(nv({"mode": "raw", "fraction_remaining": 1.0})["state"], "off")
        self.assertEqual(nv(False)["state"], "off")
        self.assertEqual(nv(0.3), dict(nv(0.3), state="on", fraction_remaining=0.3))
        self.assertEqual(nv({"remaining_pct": 40})["fraction_remaining"], 0.4)
        self.assertEqual(nv({"fraction": 42})["fraction_remaining"], 0.42)   # percent heuristic
        self.assertEqual(nv({"fraction_remaining": -3})["fraction_remaining"], 0.0)
        self.assertEqual(nv({"mode": "mystery"})["state"], "unknown")

    def test_fake_ventilation_decays_toward_residual(self):
        bridge, client = make_client(0.0)
        with client:
            off = client.get("/api/state").json()["ventilation"]
            run_fake(bridge, 10.0)
            on = client.get("/api/state").json()["ventilation"]
        self.assertEqual((off["state"], off["fraction_remaining"]), ("off", 1.0))
        self.assertEqual(on["state"], "on")
        self.assertLess(on["fraction_remaining"], 0.1)
        self.assertGreaterEqual(on["fraction_remaining"], on["residual"])

    def test_bad_ventilation_field_does_not_break_gas(self):
        state = rb.HudState()
        state.ingest_gas_json(json.dumps({"o2_pct": 20.9, "alarm": False, "ventilation": {"x": []}}))
        snap = state.snapshot()
        self.assertEqual(snap["gas"]["o2_pct"], 20.9)
        self.assertEqual(snap["ventilation"]["state"], "unknown")
        state.ingest_gas_json(json.dumps({"o2_pct": 20.9, "alarm": False}))
        self.assertIsNone(state.snapshot()["ventilation"])

    def test_ventilation_streamed_in_ws_diff(self):
        bridge, client = make_client(0.0)
        with client, client.websocket_connect(WS_URL, headers=ws_headers()) as ws:
            hello = ws.receive_json()
            self.assertEqual(hello["state"]["ventilation"]["state"], "off")
            run_fake(bridge, 3.5, start=0.0)
            receive_until(ws, lambda m: m["type"] == "diff"
                          and (m["changes"].get("ventilation") or {}).get("state") == "on")


class TestProbe(unittest.TestCase):
    def test_fake_probe_result_on_topic_path(self):
        bridge, client = make_client(0.0)
        with client:
            run_fake(bridge, 10.4)
            self.assertIsNone(client.get("/api/state").json()["probe"])
            bridge.tick(10.5)
            probe = client.get("/api/state").json()["probe"]
        self.assertEqual(probe["source"], "topic")
        # The mission's six points: 0.5 and 1.0 m inside the door x three heights.
        self.assertEqual([(r["z"], r["inside_m"]) for r in probe["rows"]],
                         [(0.2, 0.5), (0.2, 1.0), (0.6, 0.5), (0.6, 1.0), (1.0, 0.5), (1.0, 1.0)])
        self.assertEqual(probe["depths_m"], [0.5, 1.0])
        self.assertEqual(probe["points_xy"], [[5.5, 0.0], [5.0, 0.0]])
        self.assertEqual(probe["worst"]["h2s_ppm"], 1.8)     # pooled low
        self.assertFalse(probe["any_alarm"])
        self.assertTrue(probe["complete"])
        self.assertIsNone(probe["verdict"])
        self.assertEqual(probe["xy"], [5.5, 0.0])
        self.assertEqual(probe["inside_m"], 0.5)

    def test_threshold_colouring_and_worst(self):
        probe = rb.normalize_probe({"samples": [
            {"z": 1.0, "o2_pct": 20.9, "lel_pct": 2.0, "h2s_ppm": 3.0, "co_ppm": 4},
            {"z": 0.2, "o2_pct": 19.6, "lel_pct": 5.0, "h2s_ppm": 14.0, "co_ppm": 2,
             "alarms": {"lel": True}},
            {"z": 0.6, "o2_pct": 23.2, "lel_pct": 9.99, "h2s_ppm": 9.9, "co_ppm": 35},
        ]})
        rows = {r["z"]: r for r in probe["rows"]}
        self.assertEqual([r["z"] for r in probe["rows"]], [0.2, 0.6, 1.0])
        self.assertTrue(rows[0.2]["alarms"]["h2s"])
        self.assertTrue(rows[0.2]["alarms"]["lel"])          # payload flag OR'ed in
        self.assertFalse(rows[0.6]["alarms"]["h2s"])
        self.assertFalse(rows[0.6]["alarms"]["lel"])
        self.assertTrue(rows[0.6]["alarms"]["co"])           # >= 35 ppm
        self.assertFalse(rows[1.0]["alarms"]["o2"])
        self.assertEqual(probe["worst"]["o2_pct"], 19.6)     # closest to a limit
        self.assertEqual(probe["worst"]["h2s_ppm"], 14.0)
        self.assertEqual(probe["worst"]["alarms"],
                         {"o2": False, "lel": True, "h2s": True, "co": True})
        self.assertTrue(probe["any_alarm"])

    def test_accepted_layouts(self):
        rows_dict = rb.normalize_probe({"heights": {"0.2m": {"o2": 20.9, "lel": 1, "h2s": 2, "co": 3},
                                                    "1.0m": {"o2": 20.9, "lel": 1, "h2s": 1, "co": 3}}})
        self.assertEqual([r["z"] for r in rows_dict["rows"]], [0.2, 1.0])
        parallel = rb.normalize_probe({"heights": [0.2, 0.6], "readings": [
            {"reading": {"o2_pct": 20.9, "lel_pct": 1, "h2s_ppm": 11, "co_ppm": 1}},
            {"reading": {"o2_pct": 20.9, "lel_pct": 1, "h2s_ppm": 2, "co_ppm": 1}}],
            "passed": False, "at_xy": {"x": 5.4, "y": 0.1}})
        self.assertEqual([r["z"] for r in parallel["rows"]], [0.2, 0.6])
        self.assertTrue(parallel["rows"][0]["alarms"]["h2s"])
        self.assertIs(parallel["verdict"], False)
        self.assertEqual(parallel["xy"], [5.4, 0.1])
        single = rb.normalize_probe({"z": 0.6, "o2_pct": 20.9, "lel_pct": 1, "h2s_ppm": 1})
        self.assertEqual(len(single["rows"]), 1)
        self.assertFalse(single["complete"])                  # CO missing

    def test_probe_from_event_and_bad_payloads(self):
        state = rb.HudState()
        state.ingest_probe_json("{bad")
        state.ingest_probe_json(json.dumps({"samples": []}))
        state.ingest_probe_json(json.dumps({"samples": [{"z": 0.2}]}))
        state.ingest_probe_json(json.dumps([1, 2]))
        self.assertIsNone(state.snapshot()["probe"])
        state.add_event({"source": "mission", "kind": "probe_result", "message": "probe",
                         "data": {"samples": [{"z": 0.2, "o2_pct": 20.9, "lel_pct": 1,
                                               "h2s_ppm": 1, "co_ppm": 1}]}})
        self.assertEqual(state.snapshot()["probe"]["source"], "event")
        json.dumps(state.snapshot(), allow_nan=False)


class TestIncidentBanner(unittest.TestCase):
    def test_fake_alert_has_reading_and_retreat_note(self):
        bridge, _client = make_client(0.0)
        run_fake(bridge, 20.6)
        alert = bridge.state.snapshot()["alert"]
        self.assertEqual(alert["seq"], 1)
        self.assertIn("H2S", alert["reading"])
        self.assertIn("ppm", alert["reading"])
        # Spot is still inside at 20.6 s: the note must not claim it is out yet.
        self.assertEqual(alert["note"],
                         "Spot is retreating automatically — no human entered the space")
        run_fake(bridge, bridge._retreat_end_t + 0.5)   # the fake's retreat_complete
        alert = bridge.state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["note"]), (1, rb.RETREAT_NOTE))
        self.assertEqual(alert["note"],
                         "Spot retreated automatically — no human entered the space")

    def test_reaction_time_and_presenter_staging_are_disclosed(self):
        bridge, _client = make_client(0.0)
        run_fake(bridge, 20.6)
        alert = bridge.state.snapshot()["alert"]
        self.assertEqual(alert["reaction_s"], 0.02)
        self.assertTrue(alert["staged"])     # fake's demo_inject disclosure + the pocket count
        # Mission-shaped: the guard's own flag stands in for the disclosure;
        # the facade's pocket count is still needed (interface contract).
        state, advance = TestIngest._clocked_state()
        state.ingest_gas_json(json.dumps(dict(json.loads(TestIngest._gas(1.0)), injected_pockets=1)))
        state.add_event({"source": "guard", "kind": "alarm", "message": "GAS ALARM",
                         "data": {"reading": json.loads(TestIngest._gas(30.0)), "auto_retreat": True,
                                  "stop_command_s": 0.004, "staged_by_presenter": [{"gas": "h2s"}]}})
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["reaction_s"], alert["staged"]), (0.004, True))
        # An unstaged incident says nothing about staging.
        advance(600.0)
        state.add_event({"source": "guard", "kind": "alarm", "message": "GAS ALARM",
                         "data": {"reading": json.loads(TestIngest._gas(30.0)), "auto_retreat": True}})
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["staged"], alert["reaction_s"]), (2, False, None))

    def test_mission_shaped_retreat_uses_the_guards_reading(self):
        state, advance = TestIngest._clocked_state()
        mission_alarm_sequence(state, advance, h2s=25.3)
        alert = state.snapshot()["alert"]
        self.assertEqual(alert["seq"], 1)                     # one flash per incident
        self.assertEqual(alert["reading"], "H2S 25.3 ppm (limit < 10 ppm)")
        self.assertEqual(alert["note"], rb.RETREATING_NOTE)   # under way, not yet done
        # Monitor echoes at other values during the retreat don't rewrite it.
        advance(1.0)
        state.add_event({"source": "guard", "kind": "monitor", "message": "H2S 40 ppm",
                         "data": json.loads(TestIngest._gas(40.0))})
        self.assertEqual(state.snapshot()["alert"]["reading"], "H2S 25.3 ppm (limit < 10 ppm)")
        # retreat_complete long after the incident window: annotates, no re-flash.
        state.ingest_gas_json(TestIngest._gas(1.0))
        advance(30.0)
        state.add_event({"source": "guard", "kind": "retreat_complete", "message": "out",
                         "data": {"cause": "atmosphere"}})
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["note"]), (1, rb.RETREAT_NOTE))

    def test_gas_edge_alone_has_reading_but_no_retreat_claim(self):
        state, _advance = TestIngest._clocked_state()
        state.ingest_gas_json(TestIngest._gas(1.0))
        state.ingest_gas_json(TestIngest._gas(12.0))
        alert = state.snapshot()["alert"]
        self.assertEqual(alert["reading"], "H2S 12.0 ppm (limit < 10 ppm)")
        self.assertIsNone(alert["note"])

    def test_rehearsal_mode_never_claims_a_retreat(self):
        state, _advance = TestIngest._clocked_state()
        state.add_event({"source": "guard", "kind": "alarm", "message": "GAS ALARM",
                         "data": {"reading": json.loads(TestIngest._gas(20.0)),
                                  "auto_retreat": False}})
        state.add_event({"source": "guard", "kind": "alarm_ignored", "message": "rehearsal"})
        alert = state.snapshot()["alert"]
        self.assertEqual(alert["title"], "ALARM: AUTO-RETREAT OFF")
        self.assertIsNone(alert["note"])
        self.assertIn("H2S 20.0 ppm", alert["reading"])

    def test_failed_retreat_drops_the_note_but_keeps_the_reading(self):
        state, advance = TestIngest._clocked_state()
        mission_alarm_sequence(state, advance)
        advance(2.0)
        state.add_event({"source": "guard", "kind": "retreat_incomplete",
                         "message": "Retreat INCOMPLETE", "data": {"cause": "atmosphere"}})
        alert = state.snapshot()["alert"]
        self.assertEqual(alert["seq"], 2)
        self.assertEqual(alert["title"], rb.RETREAT_FAILED_TITLE)
        self.assertIsNone(alert["note"])
        self.assertIn("H2S", alert["reading"])

    def test_commanded_retreat_is_labelled_as_commanded(self):
        state, _advance = TestIngest._clocked_state()
        state.add_event({"source": "mission", "kind": "permit", "message": "aborted",
                         "data": {"state": "ABORTED", "cause": "operator"}})
        state.add_event({"source": "mission", "kind": "retreat_start", "message": "RETREAT",
                         "data": {"cause": "operator"}})
        self.assertEqual(state.snapshot()["alert"]["note"], rb.COMMAND_RETREATING_NOTE)
        state.add_event({"source": "mission", "kind": "retreat_complete", "message": "out",
                         "data": {"cause": "operator"}})
        self.assertEqual(state.snapshot()["alert"]["note"], rb.COMMAND_RETREAT_NOTE)

    def test_replayed_retreat_does_not_annotate(self):
        state = rb.HudState()
        state.ingest_event_json(json.dumps({"source": "guard", "kind": "retreat_start",
                                            "message": "old", "data": {"cause": "atmosphere"}}),
                                replay=True)
        self.assertIsNone(state.snapshot()["alert"]["note"])


class TestAuditExport(unittest.TestCase):
    def test_jsonl_download_is_the_ring_buffer(self):
        bridge, client = make_client(0.0)
        run_fake(bridge, 21.0)
        with client:
            resp = client.get("/api/audit.jsonl")
        self.assertEqual(resp.status_code, 200)
        self.assertTrue(resp.headers["content-type"].startswith("application/x-ndjson"))
        self.assertRegex(resp.headers["content-disposition"],
                         r'^attachment; filename="spot-hud-audit-\d{8}T\d{6}Z\.jsonl"$')
        self.assertEqual(resp.headers["cache-control"], "no-store")
        self.assertIn("hud-event-buffers", resp.headers["x-audit-scope"])
        lines = resp.text.splitlines()
        records = [json.loads(line) for line in lines]
        self.assertEqual(len(records), int(resp.headers["x-audit-events"]))
        self.assertEqual([r["seq"] for r in records], [e["seq"] for e in bridge.state.events()])
        self.assertTrue(all("permit_state" in r["hud"] for r in records))
        kinds = [r["kind"] for r in records]
        self.assertIn("gas_alarm", kinds)
        self.assertIn("permit_prepared", kinds)
        self.assertNotIn("NaN", resp.text)

    def test_csv_download_flattens_events_with_gas_provenance(self):
        import csv
        bridge, client = make_client(0.0)
        run_fake(bridge, 21.0)
        with client:
            resp = client.get("/api/audit.csv")
        self.assertEqual(resp.status_code, 200)
        self.assertTrue(resp.headers["content-type"].startswith("text/csv"))
        self.assertIn(".csv", resp.headers["content-disposition"])
        text = resp.content.decode("utf-8")
        self.assertTrue(text.startswith("﻿"))
        rows = list(csv.DictReader(io.StringIO(text[1:])))
        self.assertEqual(tuple(rows[0].keys()), server.audit.CSV_COLUMNS)
        self.assertEqual(len(rows), len(bridge.state.events()))
        alarm = next(r for r in rows if r["kind"] == "gas_alarm")
        self.assertEqual(alarm["source"], "guard")
        self.assertEqual(alarm["gas_values_from"], "event")
        self.assertGreater(float(alarm["h2s_ppm"]), 10.0)
        self.assertEqual(alarm["gas_alarm"], "true")
        self.assertEqual(alarm["permit_state"], "ENTRY_IN_PROGRESS")
        test = next(r for r in rows if r["kind"] == "atmospheric_test" and "H2S" in r["message"])
        self.assertEqual((test["gas_values_from"], test["h2s_ppm"], test["o2_pct"]),
                         ("event", "1.8", ""))
        say = [r for r in rows if r["kind"] == "say"][-1]    # 21 s: gas flowing
        self.assertEqual(say["gas_values_from"], "hud_latest")
        self.assertNotEqual(say["o2_pct"], "")
        first = rows[0]                                      # before any gas sample
        self.assertEqual((first["gas_values_from"], first["o2_pct"]), ("", ""))
        self.assertTrue(first["rx_wall_utc"].endswith("Z"))

    def test_csv_neutralizes_formula_cells(self):
        state = rb.HudState()
        state.add_event({"source": "agent", "kind": "say", "message": "=HYPERLINK(\"http://x\")"})
        state.add_event({"source": "@evil", "kind": "-cmd", "message": "+1 plain"})
        rows = server.audit.csv_text(state.events()).splitlines()
        self.assertIn("'=HYPERLINK", rows[1])
        self.assertIn("'@evil", rows[2])
        self.assertIn("'-cmd", rows[2])
        self.assertIn("'+1 plain", rows[2])

    def test_replayed_events_get_no_gas_stamp(self):
        state = rb.HudState()
        state.ingest_gas_json(TestIngest._gas(1.0))
        state.ingest_event_json(json.dumps({"source": "mission", "kind": "info", "message": "old"}),
                                replay=True)
        state.add_event({"source": "mission", "kind": "info", "message": "new"})
        old, new = state.events()
        self.assertEqual((old["hud"]["replay"], old["hud"]["gas"]), (True, None))
        self.assertEqual(new["hud"]["gas"]["h2s_ppm"], 1.0)
        rows = [server.audit.flatten(e) for e in (old, new)]
        self.assertEqual([r["gas_values_from"] for r in rows], ["", "hud_latest"])

    def test_empty_buffer_exports_cleanly(self):
        state = rb.HudState()
        with hud_client(server.create_app(rb.FakeBridge(state=state, realtime=False))) as client:
            jsonl = client.get("/api/audit.jsonl")
            csv_resp = client.get("/api/audit.csv")
        self.assertEqual((jsonl.status_code, jsonl.text), (200, ""))
        self.assertEqual(jsonl.headers["x-audit-events"], "0")
        self.assertEqual(len(csv_resp.content.decode("utf-8").splitlines()), 1)   # header only


class TestModuleContracts(unittest.TestCase):
    """The exact shapes sim/isaac_app.py and agent/mission.py publish (read
    from their source, 2026-09-27). The real classes are used when they
    import; the literal shapes pin the contract either way."""

    FACADE_PROBE = {"id": "hud-1", "sim_time": 42.5, "simulated": True,
                    "ventilation": {"on": False, "t_on": None, "tau_s": 15.0,
                                    "fraction_remaining": 1.0, "residual_fraction": 0.04},
                    "samples": [
                        {"point": [5.5, 0.0, z], "o2_pct": 20.4, "lel_pct": lel, "h2s_ppm": h2s,
                         "co_ppm": 4.0, "alarms": {"o2": False, "lel": lel >= 10, "h2s": h2s >= 10,
                                                   "co": False, "any": lel >= 10 or h2s >= 10}}
                        for z, lel, h2s in ((0.2, 12.0, 14.0), (0.6, 8.0, 6.0), (1.0, 4.0, 2.0))]}

    def test_facade_probe_result(self):
        state = rb.HudState()
        state.ingest_site_json(json.dumps(rb.BUILTIN_SITE))
        state.ingest_probe_json(json.dumps(self.FACADE_PROBE))
        probe = state.snapshot()["probe"]
        self.assertEqual([r["z"] for r in probe["rows"]], [0.2, 0.6, 1.0])
        self.assertEqual(probe["xy"], [5.5, 0.0])                 # centroid of the points
        self.assertEqual(probe["inside_m"], 0.5)                  # from the site's door plane
        self.assertEqual(probe["t"], 42.5)
        self.assertEqual(probe["worst"]["lel_pct"], 12.0)
        self.assertEqual(probe["worst"]["alarms"], {"o2": False, "lel": True, "h2s": True, "co": False})
        self.assertIsNone(probe["error"])

    def test_facade_rejected_probe_is_shown_not_dropped(self):
        state = rb.HudState()
        state.ingest_probe_json(json.dumps({"id": "hud-2", "error": "point (9.00, 0.00) is beyond "
                                            "the 1.2 m probe reach", "samples": [], "sim_time": 3.0}))
        probe = state.snapshot()["probe"]
        self.assertEqual(probe["rows"], [])
        self.assertIn("beyond", probe["error"])
        json.dumps(state.snapshot(), allow_nan=False)

    def test_mission_atmos_probe_not_overwritten_by_late_facade_reply(self):
        state, advance = TestIngest._clocked_state()
        state.add_event({"source": "mission", "kind": "atmos_probe", "message": "rows",
                         "data": {"samples": [{"z": 0.2, "o2_pct": 20.9, "lel_pct": 1.0,
                                               "h2s_ppm": 14.0, "co_ppm": 1.0}],
                                  "passed": False, "xy": [5.5, 0.0], "inside_m": 0.5, "at": "door",
                                  "simulated": True}})
        advance(0.4)
        state.ingest_probe_json(json.dumps(self.FACADE_PROBE))   # late per-request reply
        probe = state.snapshot()["probe"]
        self.assertEqual((probe["source"], probe["verdict"]), ("event", False))
        advance(5.0)
        state.ingest_probe_json(json.dumps(self.FACADE_PROBE))   # a new probe, later
        self.assertEqual(state.snapshot()["probe"]["source"], "topic")

    def test_facade_ventilation_block(self):
        nv = rb.normalize_ventilation
        on = nv({"on": True, "t_on": 5.0, "tau_s": 15.0, "fraction_remaining": 0.3,
                 "residual_fraction": 0.04})
        self.assertEqual((on["state"], on["fraction_remaining"], on["residual"], on["tau_s"]),
                         ("on", 0.3, 0.04, 15.0))
        self.assertEqual(nv(self.FACADE_PROBE["ventilation"])["state"], "off")
        try:
            from sim.gas_field import GasField
        except Exception as error:  # the sim module may be mid-edit
            self.skipTest("sim.gas_field not importable: %r" % (error,))
        field = GasField()
        self.assertEqual(nv(field.ventilation_state(0.0))["state"], "off")

    def test_mission_permit_dict(self):
        fields = {"permit_id": "CSE-1", "isolation_confirmed": True, "loto_confirmed": False,
                  "attendant": "", "rescue_plan": "", "issued_at": None,
                  "missing": ["loto_confirmed", "attendant", "rescue_plan"], "complete": False}
        try:
            from agent.mission import Permit
            fields = Permit("CSE-1", isolation_confirmed=True).as_dict()
        except Exception:  # the mission module may be mid-edit: literal shape above
            pass
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit_prepared", "message": "incomplete",
                         "data": {"permit": fields, "permit_id": "CSE-1",
                                  "permit_state": "NOT_STARTED", "complete": False}})
        rows = state.snapshot()["permit"]["checklist"]
        self.assertEqual([(r["ok"], r["text"]) for r in rows],
                         [(True, "Confirmed"), (False, "Not confirmed"), (False, "Not set"),
                          (False, "Not set")])
        done = dict(fields, loto_confirmed=True, attendant="J. Doe", rescue_plan="Team B",
                    missing=[], complete=True)
        state.add_event({"source": "mission", "kind": "permit", "message": "testing",
                         "data": {"state": "PRE_ENTRY_TESTING", "previous": "NOT_STARTED",
                                  "permit_id": "CSE-1", "reason": "r", "permit": done}})
        permit = state.snapshot()["permit"]
        self.assertEqual(permit["state"], "PRE_ENTRY_TESTING")
        self.assertTrue(permit["checklist_summary"]["complete"])


class TestReviewRegressions(unittest.TestCase):
    """One test per bug found in the adversarial review (2026-09-27)."""

    GAS = staticmethod(lambda h2s, pockets=None: json.dumps(dict(
        json.loads(TestIngest._gas(h2s)),
        **({} if pockets is None else {"injected_pockets": pockets}))))

    def _alarm(self, state, h2s=25.0, pockets=None, staged=None):
        data = {"reading": json.loads(self.GAS(h2s, pockets)), "auto_retreat": True}
        if staged is not None:
            data["staged_by_presenter"] = staged
        state.add_event({"source": "guard", "kind": "alarm", "message": "GAS ALARM", "data": data})

    def test_agent_tool_and_harness_injections_are_disclosed_as_staged(self):
        # spot_agent.inject_gas logs source "agent" kind gas_injected;
        # run_mission --inject-h2s-at-waypoint logs source "mission" demo_inject.
        # Neither reaches the mission's staged_by_presenter (presenter-only).
        for source, kind in (("agent", "gas_injected"), ("mission", "demo_inject")):
            state, _advance = TestIngest._clocked_state()
            state.ingest_gas_json(self.GAS(1.0, 0))
            state.add_event({"source": source, "kind": kind, "message": "PRESENTER DEMO",
                             "data": {"gas": "h2s", "ppm": 30.0}})
            state.ingest_gas_json(self.GAS(25.0, 1))
            self._alarm(state, staged=[])
            self.assertTrue(state.snapshot()["alert"]["staged"], (source, kind))

    def test_injection_disclosed_just_after_the_gas_edge_still_stages(self):
        # inject_gas.sh publishes the inject, THEN the disclosure: the sim's
        # immediate payload/gas can beat the event to the HUD.
        state, advance = TestIngest._clocked_state()
        state.ingest_gas_json(self.GAS(1.0, 0))
        state.ingest_gas_json(self.GAS(25.0, 1))          # edge first: a pocket, no disclosure yet
        self.assertFalse(state.snapshot()["alert"]["staged"])
        advance(0.05)
        state.add_event({"source": "presenter", "kind": "demo_inject", "message": "PRESENTER DEMO",
                         "data": {"gas": "h2s"}})
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["staged"]), (1, True))

    def test_staging_needs_both_a_disclosure_and_the_facade_count(self):
        # Interface contract: a pocket count alone is a genuine alarm...
        state, advance = TestIngest._clocked_state()
        state.ingest_gas_json(self.GAS(1.0, 0))
        state.ingest_gas_json(self.GAS(25.0, 1))          # the edge sample carries the count
        self.assertFalse(state.snapshot()["alert"]["staged"])
        # ...a disclosure plus the count stages it...
        state.ingest_gas_json(self.GAS(1.0, 1))
        advance(30.0)
        state.add_event({"source": "presenter", "kind": "demo_inject", "message": "x", "data": {}})
        state.ingest_gas_json(self.GAS(25.0, 1))
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["staged"]), (2, True))
        # ...and once cleared (count 0) a later alarm is not staged, even
        # inside the disclosure window.
        state.ingest_gas_json(self.GAS(1.0, 0))
        advance(30.0)
        state.ingest_gas_json(self.GAS(25.0, 0))
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["staged"]), (3, False))

    def test_clear_event_ends_the_staging_window_without_a_count(self):
        state, advance = TestIngest._clocked_state()
        state.add_event({"source": "presenter", "kind": "demo_inject", "message": "x", "data": {}})
        advance(10.0)
        state.add_event({"source": "presenter", "kind": "demo_clear", "message": "x", "data": {}})
        advance(30.0)
        state.ingest_gas_json(self.GAS(1.0))
        state.ingest_gas_json(self.GAS(25.0))
        self.assertFalse(state.snapshot()["alert"]["staged"])

    def test_retreat_software_fault_is_a_failed_retreat(self):
        # mission._retreat_worker's except path emits kind "error" only.
        state, advance = TestIngest._clocked_state()
        mission_alarm_sequence(state, advance)
        advance(3.0)
        state.add_event({"source": "guard", "kind": "error",
                         "message": "Retreat software fault: RuntimeError('boom')"})
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["title"], alert["note"]),
                         (2, rb.RETREAT_FAILED_TITLE, None))
        self.assertIn("H2S", alert["reading"])
        # An unrelated agent error is not a retreat failure.
        state.add_event({"source": "agent", "kind": "error", "message": "Agent error: retreat?"})
        self.assertEqual(state.snapshot()["alert"]["seq"], 2)

    def test_long_retreat_completion_still_annotates_and_is_not_downgraded(self):
        state, advance = TestIngest._clocked_state()
        mission_alarm_sequence(state, advance)
        state.ingest_gas_json(TestIngest._gas(1.0))
        advance(200.0)                                    # < mission RETREAT_TIMEOUT_S (240)
        state.add_event({"source": "guard", "kind": "retreat_complete", "message": "out",
                         "data": {"cause": "atmosphere"}})
        self.assertEqual(state.snapshot()["alert"]["note"], rb.RETREAT_NOTE)
        # Inside an open incident a late in-progress signal never downgrades it.
        state, advance = TestIngest._clocked_state()
        mission_alarm_sequence(state, advance)
        state.add_event({"source": "guard", "kind": "retreat_complete", "message": "out",
                         "data": {"cause": "atmosphere"}})
        state.add_event({"source": "guard", "kind": "retreat_start", "message": "late echo",
                         "data": {"cause": "atmosphere"}})
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["note"]), (1, rb.RETREAT_NOTE))

    def test_checklist_follows_the_missions_gate(self):
        # Permit.prepare accepts any non-blank text, so "TBD" completes the
        # permit and Spot may approach; the HUD must not show a cross.
        permit = {"permit_id": "CSE-9", "isolation_confirmed": True, "loto_confirmed": True,
                  "attendant": "attendant", "rescue_plan": "TBD", "issued_at": "2026-09-27",
                  "missing": [], "complete": True}
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit_prepared", "message": "complete",
                         "data": {"permit": permit, "permit_id": "CSE-9",
                                  "permit_state": "NOT_STARTED", "complete": True, "missing": []}})
        p = state.snapshot()["permit"]
        self.assertEqual([r["ok"] for r in p["checklist"]], [True] * 4)
        self.assertEqual(p["checklist"][3]["text"], "TBD")
        self.assertEqual(p["checklist_summary"], {"confirmed": 4, "total": 4, "complete": True})
        # The mission's missing list wins the other way too.
        state.add_event({"source": "mission", "kind": "permit_prepared", "message": "incomplete",
                         "data": {"permit": dict(permit, missing=["attendant"], complete=False)}})
        p = state.snapshot()["permit"]
        self.assertEqual([r["ok"] for r in p["checklist"]], [True, True, False, True])
        self.assertFalse(p["checklist_summary"]["complete"])


class TestFrontendSource(unittest.TestCase):
    """Static checks on hud/static/app.js (no browser needed)."""

    @classmethod
    def setUpClass(cls):
        import os
        with open(os.path.join(server.STATIC_DIR, "app.js"), encoding="utf-8") as handle:
            cls.js = handle.read()

    def test_no_regex_lookbehind(self):
        # Regression: a /(?<!in).../ literal is a SyntaxError on Safari < 16.4,
        # which aborts the whole script and leaves a blank HUD.
        self.assertNotIn("(?<", self.js)

    def test_untrusted_text_never_parsed_as_html(self):
        # Event, permit (attendant / rescue plan) and LLM text reach the page.
        self.assertNotRegex(self.js, r"\.(inner|outer)HTML\b")
        self.assertNotIn("insertAdjacentHTML(", self.js)
        self.assertNotIn("document.write", self.js)

    def test_new_panels_present_and_wired(self):
        import os
        with open(os.path.join(server.STATIC_DIR, "index.html"), encoding="utf-8") as handle:
            html = handle.read()
        for element_id in ("checklist", "checklist-summary", "incident", "vent-chip", "vent-fill",
                           "probe-body", "probe-verdict", "flash-reading", "flash-note",
                           "flash-detail"):
            self.assertIn('id="%s"' % element_id, html)
            self.assertIn('$("%s")' % element_id, self.js)   # rendered by app.js
        for element_id in ("probe-table", "audit-dl"):         # static markup only
            self.assertIn('id="%s"' % element_id, html)
        self.assertIn('href="/api/audit.jsonl" download', html)
        self.assertIn('href="/api/audit.csv" download', html)
        self.assertIn("Download audit log", html)
        self.assertIn("stand-in for an arm-mounted gas probe", html)   # honest labeling
        for key in ("model.ventilation", "model.probe", "a.reading", "alert.note"):
            self.assertIn(key, self.js)

    def test_routine_guard_lines_not_styled_as_alarms(self):
        # Regression: every source=guard line (incl. benign 5 s monitor
        # readings) got the red alarm style.
        self.assertNotIn('source === "guard" ||', self.js)


class TestHelpers(unittest.TestCase):
    def test_encode_ros_image_stride_and_bgr(self):
        w, h, step = 4, 2, 4 * 3 + 4  # 4 bytes of row padding
        rows = []
        for _ in range(h):
            rows.append(bytes([10, 20, 250] * w) + b"\x00" * 4)  # BGR: pure-ish red
        jpeg, ow, oh = rb.encode_ros_image(b"".join(rows), w, h, "bgr8", step)
        img = Image.open(io.BytesIO(jpeg)).convert("RGB")
        self.assertEqual((ow, oh), (4, 2))
        r, g, b = img.getpixel((1, 1))
        self.assertGreater(r, 200)
        self.assertLess(b, 60)

    def test_encode_ros_image_downsamples(self):
        w, h = 1280, 720
        jpeg, ow, oh = rb.encode_ros_image(bytes(w * h * 3), w, h, "rgb8", 0, max_width=640)
        self.assertEqual((ow, oh), (640, 360))
        self.assertEqual(jpeg[:2], b"\xff\xd8")

    def test_encode_ros_image_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            rb.encode_ros_image(bytes(12), 2, 2, "32FC1")
        with self.assertRaises(ValueError):
            rb.encode_ros_image(bytes(5), 2, 2, "rgb8")

    def test_site_defaults_fallback(self):
        site, source = rb.load_site_defaults("/nonexistent/site_defaults.json")
        self.assertEqual(source, "builtin")
        self.assertEqual(site["door_xy"], [6.0, 0.0])
        self.assertEqual(len(site["interior_waypoints"]), 5)
        alias = rb.normalize_site(dict(rb.BUILTIN_SITE, door_xy=None, doorway_xy=[6.0, 0.0]))
        self.assertEqual(alias["door_xy"], [6.0, 0.0])
        with self.assertRaises(ValueError):
            rb.normalize_site({"tank_center_xy": [0, 0], "tank_radius": 6})

    def test_topic_site_wins_over_defaults(self):
        state = rb.HudState()
        state.ingest_site_json(json.dumps(rb.BUILTIN_SITE))
        site, source = rb.load_site_defaults("/nonexistent.json")
        state.set_site(dict(site, tank_radius=99.0), source)
        self.assertEqual(state.snapshot()["site"]["tank_radius"], 6.0)
        self.assertEqual(state.snapshot()["site"]["source"], "topic")


class TestBridges(unittest.TestCase):
    def test_realtime_fake_bridge_starts_and_stops_quickly(self):
        bridge = rb.FakeBridge(realtime=True, speed=10.0)
        bridge.start()
        try:
            deadline = time.monotonic() + 3.0
            while bridge.state.snapshot()["pose"] is None and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertIsNotNone(bridge.state.snapshot()["pose"])
        finally:
            t0 = time.monotonic()
            bridge.stop()
            self.assertLess(time.monotonic() - t0, 2.5)

    def test_ros_bridge_wiring_with_fake_rclpy(self):
        """RosBridge end to end on fakes: QoS, callbacks, camera worker, teardown."""
        mods = _fake_ros_modules()
        with mock.patch.dict(sys.modules, mods):
            state = rb.HudState(mode="ros")
            bridge = rb.RosBridge(state, site_defaults_path="/nonexistent.json", site_timeout_s=0.3)
            bridge.start()
            try:
                self.assertEqual(mods["rclpy"].init_kwargs[-1].get("signal_handler_options").name, "NO")
                node = mods["rclpy"].nodes[-1]
                self.assertEqual(node.namespace, "/demo_spot")
                self.assertEqual(set(node.subs), {"payload/status", "payload/gas", "payload/events",
                                                  "payload/site", "odometry", "camera/frontleft/image",
                                                  "payload/gas/probe_result", "payload/guard"})
                # Reliable + volatile: matches volatile AND transient-local publishers.
                self.assertEqual(node.subs["payload/gas/probe_result"][2], 10)
                # Regression: must match robot_client's TRANSIENT_LOCAL depth-100
                # publisher so a restarted HUD gets the current permit back.
                ev_qos = node.subs["payload/events"][2]
                self.assertEqual(ev_qos.durability.name, "TRANSIENT_LOCAL")
                self.assertEqual(ev_qos.reliability.name, "RELIABLE")
                self.assertEqual(ev_qos.depth, 100)
                self.assertEqual(node.subs["payload/site"][2].durability.name, "TRANSIENT_LOCAL")

                def push(topic, msg):
                    node.subs[topic][1](msg)
                ns = types.SimpleNamespace
                push("payload/status", ns(data=json.dumps({"estopped": False, "claimed": True})))
                push("payload/gas", ns(data='{"o2_pct": 20.9, "alarm": false}'))
                push("payload/events", ns(data=json.dumps(
                    {"source": "guard", "kind": "alarm", "message": "old", "data": {"state": "ABORTED"}})))
                push("payload/events", ns(data="{broken"))       # must not raise into the executor
                push("payload/gas/probe_result", ns(data="{broken"))
                push("payload/gas/probe_result", ns(data=json.dumps(
                    {"samples": [{"z": 0.2, "o2_pct": 20.9, "lel_pct": 1, "h2s_ppm": 12, "co_ppm": 2}]})))
                push("odometry", ns(pose=ns(pose=ns(position=ns(x=3.0, y=-1.0, z=0.5),
                                                    orientation=ns(x=0.0, y=0.0, z=math.sin(0.5),
                                                                   w=math.cos(0.5)))),
                                    twist=ns(twist=ns(linear=ns(x=0.3, y=0.4, z=0.0)))))
                w, h = 32, 16
                push("camera/frontleft/image", ns(data=array.array("B", bytes(w * h * 3)), width=w,
                                                  height=h, encoding="rgb8", step=w * 3))
                deadline = time.monotonic() + 3.0
                while (state.camera_jpeg()[0] is None or not state.has_site()) \
                        and time.monotonic() < deadline:
                    time.sleep(0.05)
                snap = state.snapshot(include_events=True)
                self.assertEqual(snap["permit"]["state"], "ABORTED")
                self.assertEqual(snap["alert"]["seq"], 0)       # replay window: no stale flash
                self.assertAlmostEqual(snap["pose"]["yaw"], 1.0, places=3)
                self.assertAlmostEqual(snap["pose"]["speed"], 0.5, places=3)
                self.assertEqual(snap["camera"]["width"], w)
                self.assertEqual(snap["site"]["source"], "builtin")   # no topic within 0.3 s
                self.assertEqual(snap["probe"]["source"], "topic")
                self.assertTrue(snap["probe"]["rows"][0]["alarms"]["h2s"])
                self.assertTrue(snap["server"]["bridge_ok"])
            finally:
                t0 = time.monotonic()
                bridge.stop()
                self.assertLess(time.monotonic() - t0, 2.5)
            self.assertTrue(node.destroyed)
            self.assertFalse(state.snapshot()["server"]["bridge_ok"])
            bridge.stop()   # idempotent

    def test_ros_bridge_reports_missing_rclpy(self):
        try:
            import rclpy  # noqa: F401
        except ImportError:
            with self.assertRaises(rb.BridgeUnavailable):
                rb.check_ros_available()
        else:
            self.skipTest("rclpy is importable here; the missing-ROS path is not reachable")


FAKE_AWS = r'''#!/usr/bin/env python3
"""Stand-in for the AWS CLI (tests only): 'ssm send-command' runs the
AWS-RunShellScript commands locally with sh; 'ssm get-command-invocation'
returns Status / StandardOutputContent truncated to 24000 chars like SSM."""
import json, os, subprocess, sys, uuid
state = os.environ["FAKE_AWS_STATE"]
args = sys.argv[1:]
def opt(name):
    return args[args.index(name) + 1] if name in args else None
with open(os.path.join(state, "calls.log"), "a") as log:
    log.write(" ".join(args[:2]) + "\n")
if args[:2] == ["ssm", "send-command"]:
    req = json.load(open(opt("--cli-input-json")[len("file://"):]))
    assert req["DocumentName"] == "AWS-RunShellScript"
    script = "\n".join(req["Parameters"]["commands"])
    proc = subprocess.run(["sh", "-c", script], capture_output=True, text=True)
    cid = uuid.uuid4().hex
    json.dump({"rc": proc.returncode, "out": proc.stdout[:24000], "err": proc.stderr[:8000]},
              open(os.path.join(state, cid), "w"))
    print(cid)
elif args[:2] == ["ssm", "get-command-invocation"]:
    res = json.load(open(os.path.join(state, opt("--command-id"))))
    query = opt("--query")
    if query == "Status":
        print("Success" if res["rc"] == 0 else "Failed")
    elif query == "StandardOutputContent":
        print(res["out"])
    else:
        print(res["out"], res["err"])
else:
    sys.exit("fake aws: unsupported call %r" % args)
'''


@unittest.skipUnless(shutil.which("bash") and shutil.which("shasum") and shutil.which("sha256sum")
                     and shutil.which("python3"), "needs bash, shasum, sha256sum, python3")
class TestFetchLogScript(unittest.TestCase):
    """scripts/fetch-log.sh end to end against a local fake of the AWS CLI:
    no AWS call is made and the sim host is never touched."""

    SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "scripts", "fetch-log.sh")

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="fetchlog-test-")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.bin = os.path.join(self.tmp, "bin")
        self.state = os.path.join(self.tmp, "state")
        self.remote = os.path.join(self.tmp, "remote")
        self.dest = os.path.join(self.tmp, "dest")
        for d in (self.bin, self.state, self.remote):
            os.makedirs(d)
        aws = os.path.join(self.bin, "aws")
        with open(aws, "w") as handle:
            handle.write(FAKE_AWS)
        os.chmod(aws, 0o755)

    @staticmethod
    def remote_temp_files():
        return {f for f in os.listdir("/tmp") if f.startswith("spot-logs-")}

    def run_script(self, *args):
        env = dict(os.environ, PATH=self.bin + os.pathsep + os.environ.get("PATH", ""),
                   FAKE_AWS_STATE=self.state, SPOT_DEMO_DIR=self.remote, FETCH_DEST=self.dest,
                   SPOT_SIM_INSTANCE="i-0123456789abcdef0", AWS_REGION="us-west-2")
        # An argument list, no shell: bash, the repository's scripts/fetch-log.sh
        # and the test's own literal patterns (some hostile on purpose: the
        # script must refuse them), with a fake aws first on PATH.
        return subprocess.run(["bash", self.SCRIPT, *(str(arg) for arg in args)], env=env,
                              capture_output=True, text=True, timeout=120)

    def test_multi_chunk_fetch_is_byte_identical_and_cleans_up(self):
        logs = os.path.join(self.remote, "logs")
        os.makedirs(logs)
        # Random hex compresses only ~2x: ~80 KB of b64 -> several 20000-char chunks.
        files = {"mission-a.jsonl": "".join(json.dumps({"seq": i, "x": os.urandom(40).hex()}) + "\n"
                                            for i in range(1000)),
                 "mission-b.jsonl": '{"seq": 1, "kind": "summary"}\n'}
        for name, text in files.items():
            with open(os.path.join(logs, name), "w") as handle:
                handle.write(text)
        with open(os.path.join(logs, "notes.txt"), "w") as handle:
            handle.write("not a log")
        self.before = self.remote_temp_files()
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertRegex(result.stdout, r"in ([2-9]|\d\d+) chunk\(s\)")   # really multi-chunk
        self.assertEqual(sorted(os.listdir(self.dest)), sorted(files))
        for name, text in files.items():
            with open(os.path.join(self.dest, name)) as handle:
                self.assertEqual(handle.read(), text)
        with open(os.path.join(self.state, "calls.log")) as handle:
            calls = handle.read()
        self.assertNotIn("s3", calls)
        # The remote temp file this run created must be gone (cleanup command).
        self.assertEqual(self.remote_temp_files(), self.before)

    def test_no_logs_is_not_an_error(self):
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Nothing fetched", result.stdout)
        os.makedirs(os.path.join(self.remote, "logs"))
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("No files matching", result.stdout)
        self.assertFalse(os.path.exists(self.dest))

    def test_rejects_unsafe_pattern_before_any_aws_call(self):
        result = self.run_script("*.jsonl; rm -rf /")
        self.assertEqual(result.returncode, 2)
        self.assertFalse(os.path.exists(os.path.join(self.state, "calls.log")))

    def test_subdir_png_pattern_fetches_screenshots_flattened(self):
        # Presenter dry run: screenshots live in logs/screens/*.png.
        screens = os.path.join(self.remote, "logs", "screens")
        os.makedirs(screens)
        pngs = {"sim-ready.png": b"\x89PNG\r\n\x1a\n" + os.urandom(3000),
                "hud-ready.png": b"\x89PNG\r\n\x1a\n" + os.urandom(50)}
        for name, data in pngs.items():
            with open(os.path.join(screens, name), "wb") as handle:
                handle.write(data)
        with open(os.path.join(self.remote, "logs", "top.png"), "wb") as handle:
            handle.write(b"not in the subdir")
        result = self.run_script("screens/*.png")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(sorted(os.listdir(self.dest)), sorted(pngs))   # flattened, top.png not taken
        for name, data in pngs.items():
            with open(os.path.join(self.dest, name), "rb") as handle:
                self.assertEqual(handle.read(), data)

    def test_rejects_parent_or_hidden_dir_patterns_before_any_aws_call(self):
        for pattern in ("../*.log", "a/../*.log", ".x/*.png", "a/b/*.png", "/etc/*.txt", "screens/*.sh"):
            result = self.run_script(pattern)
            self.assertEqual(result.returncode, 2, pattern)
        self.assertFalse(os.path.exists(os.path.join(self.state, "calls.log")))


class TestSixPointProbeAndWallTime(unittest.TestCase):
    """Critic round: the mission's 2-depth x 3-height probe (P1-3) and
    wall_t on every event (P2-9)."""

    SIX = [(x, z) for x in (5.5, 5.0) for z in (0.2, 0.6, 1.0)]

    def facade_reply(self):
        return {"id": "m-7", "sim_time": 88.0, "simulated": True, "samples": [
            {"point": [x, 0.0, z], "o2_pct": 20.6, "lel_pct": 1.0,
             "h2s_ppm": 12.0 if (x, z) == (5.0, 0.2) else 3.0, "co_ppm": 1.0}
            for x, z in self.SIX]}

    def test_facade_six_point_reply_gets_depths_from_the_site(self):
        state = rb.HudState()
        state.ingest_site_json(json.dumps(rb.BUILTIN_SITE))
        state.ingest_probe_json(json.dumps(self.facade_reply()))
        probe = state.snapshot()["probe"]
        self.assertEqual([(r["z"], r["inside_m"]) for r in probe["rows"]],
                         [(0.2, 0.5), (0.2, 1.0), (0.6, 0.5), (0.6, 1.0), (1.0, 0.5), (1.0, 1.0)])
        self.assertEqual(probe["depths_m"], [0.5, 1.0])
        self.assertEqual(probe["inside_m"], 0.5)                  # nearest depth, not the centroid
        self.assertEqual(probe["points_xy"], [[5.5, 0.0], [5.0, 0.0]])
        deep_floor = next(r for r in probe["rows"] if (r["z"], r["inside_m"]) == (0.2, 1.0))
        self.assertTrue(deep_floor["alarms"]["h2s"])
        self.assertEqual(probe["worst"]["h2s_ppm"], 12.0)
        self.assertTrue(probe["any_alarm"])
        json.dumps(state.snapshot(), allow_nan=False)

    def test_mission_atmos_probe_event_with_six_rows(self):
        state = rb.HudState()
        rows = [{"inside_m": d, "z": z, "x": 6.0 - d, "y": 0.0, "o2_pct": 20.9, "lel_pct": 1.0,
                 "h2s_ppm": 2.0, "co_ppm": 1.0} for d in (0.5, 1.0) for z in (0.2, 0.6, 1.0)]
        state.add_event({"source": "mission", "kind": "atmos_probe", "message": "rows",
                         "t": 50.0, "wall_t": 1790000000.5,
                         "data": {"samples": rows, "passed": True, "xy": [5.5, 0.0],
                                  "inside_m": 0.5, "depths_m": [0.5, 1.0], "at": "door",
                                  "points_xy": [[5.5, 0.0], [5.0, 0.0]], "simulated": True}})
        probe = state.snapshot()["probe"]
        self.assertEqual(len(probe["rows"]), 6)
        self.assertEqual(probe["source"], "event")
        self.assertIs(probe["verdict"], True)
        self.assertEqual(probe["depths_m"], [0.5, 1.0])
        self.assertEqual([r["inside_m"] for r in probe["rows"][:2]], [0.5, 1.0])

    def test_events_keep_wall_t_and_the_csv_exports_both_times(self):
        import csv
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "atmos_test", "message": "H2S 3 PASS",
                         "t": 42.25, "wall_t": 1790000000.125, "data": {"gas": "h2s", "value": 3.0}})
        state.add_event({"source": "presenter", "kind": "demo_inject", "message": "old publisher",
                         "t": 1790000001.0})
        state.add_event({"source": "agent", "kind": "say", "message": "x", "t": 1.0,
                         "wall_t": float("nan")})
        events = state.events()
        self.assertEqual(events[0]["wall_t"], 1790000000.125)
        self.assertIsNone(events[1]["wall_t"])
        self.assertIsNone(events[2]["wall_t"])                     # non-finite is dropped
        self.assertIn("wall_t", server.audit.CSV_COLUMNS)
        self.assertLess(server.audit.CSV_COLUMNS.index("t"), server.audit.CSV_COLUMNS.index("wall_t"))
        text = server.audit.csv_text(events)
        rows = list(csv.DictReader(io.StringIO(text[1:])))
        self.assertEqual((rows[0]["t"], rows[0]["wall_t"]), ("42.25", "1790000000.125"))
        self.assertTrue(rows[0]["wall_utc"].startswith("2026-09-21T"))
        self.assertTrue(rows[0]["wall_utc"].endswith("Z"))
        self.assertEqual((rows[1]["wall_t"], rows[1]["wall_utc"]), ("", ""))
        self.assertNotIn("NaN", text)

    def test_out_of_range_wall_t_does_not_break_the_csv_export(self):
        # Regression: wall_t is publisher-supplied; 1e20 made fromtimestamp
        # raise OverflowError and the whole /api/audit.csv download failed.
        import csv
        state = rb.HudState()
        for wall_t in (1e20, -1e18, 1790000000.0):
            state.add_event({"source": "presenter", "kind": "demo_inject", "message": "m",
                             "t": 1.0, "wall_t": wall_t})
        text = server.audit.csv_text(state.events())
        rows = list(csv.DictReader(io.StringIO(text[1:])))
        self.assertEqual([r["wall_utc"] for r in rows[:2]], ["", ""])
        self.assertTrue(rows[2]["wall_utc"].endswith("Z"))

    def test_fake_bridge_events_carry_wall_time(self):
        bridge, client = make_client(0.0)
        with client:
            run_fake(bridge, 12.0)
            events = client.get("/api/state").json()["events"]
        self.assertTrue(events)
        self.assertTrue(all(isinstance(e["wall_t"], float) for e in events))
        test = next(e for e in events if e["kind"] == "atmospheric_test")
        self.assertIn("worst of 6 probe points", test["message"])

    def test_page_shows_six_rows_and_the_honest_probe_label(self):
        with open(os.path.join(server.STATIC_DIR, "index.html"), encoding="utf-8") as handle:
            html = handle.read()
        with open(os.path.join(server.STATIC_DIR, "app.js"), encoding="utf-8") as handle:
            js = handle.read()
        self.assertIn("extendable gas probe (simulated; arm-mounted on the real robot)", html.lower())
        self.assertIn("2 depths &times; 3 heights", html)
        self.assertIn('["—", "—", "—", "—", "—", "—", "Worst"]', js)   # six placeholder rows
        self.assertIn("m inside the door", js)
        self.assertIn("probe.points_xy", js)


EVIDENCE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs",
                        "evidence")


def _probe_rows(h2s=4.6):
    return [{"inside_m": d, "z": z, "x": 6.0 - d, "y": 0.0, "o2_pct": 20.8, "lel_pct": 0.9,
             "h2s_ppm": h2s if z == 0.2 else 1.0, "co_ppm": 0.7}
            for d in (0.5, 1.0) for z in (0.2, 0.6, 1.0)]


class TestPurgeHold(unittest.TestCase):
    """Dry-run critic V3: the HUD showed a red FAIL over an all-green probe
    table and 7 % while the agent said 8.5 %. The hold is amber and shows the
    mission's purge_hold numbers (the agent quotes the same event)."""

    # agent/mission.py _purge_hold() for docs/evidence/events-ok1.jsonl t=444.6.
    HOLD = {"fraction_remaining": 0.0847, "purge_target": 0.08, "eta_s": 1.7,
            "percent_remaining": 8.5, "fraction_source": "sim", "ventilation_on": True,
            "target_reachable": True, "prior_failure": True, "simulated": True}

    def held_state(self):
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "permit", "message": "testing",
                         "data": {"state": "PRE_ENTRY_TESTING", "permit_id": "CSE-0428"}})
        state.add_event({"source": "mission", "kind": "purge_incomplete", "t": 444.6,
                         "message": "Purge incomplete — wait: ... 8.5 % ... (entry needs <= 8 %)",
                         "data": {"purge_target_fraction": 0.08, "fraction_remaining": 0.0847,
                                  "wait_s": 1.7, "ventilation": {"on": True}}})
        state.add_event({"source": "mission", "kind": "purge_hold", "t": 444.6,
                         "message": "PURGE HOLD - waiting for ventilation to reach target",
                         "data": dict(self.HOLD)})
        state.add_event({"source": "mission", "kind": "atmos_probe", "message": "rows", "t": 444.6,
                         "data": {"samples": _probe_rows(), "passed": False, "at": "door",
                                  "xy": [5.5, 0.0], "depths_m": [0.5, 1.0]}})
        return state

    def test_hold_carries_exactly_the_missions_numbers(self):
        state = self.held_state()
        snap = state.snapshot()
        hold = snap["purge_hold"]
        self.assertTrue(hold["active"])
        self.assertEqual(hold["kind"], "purge_hold")
        self.assertEqual((hold["fraction_remaining"], hold["purge_target"], hold["eta_s"]),
                         (0.0847, 0.08, 1.7))
        self.assertEqual(hold["percent_remaining"], 8.5)
        self.assertFalse(hold["reached"])
        self.assertEqual(snap["permit"]["state"], "PRE_ENTRY_TESTING")   # hold is not an abort
        self.assertEqual(state.alert_seq, 0)                                # and never flashes
        # The all-green probe table is marked as held (amber), not failed.
        self.assertIs(snap["probe"]["verdict"], False)
        self.assertFalse(snap["probe"]["any_alarm"])
        self.assertTrue(snap["probe"]["held"])
        json.dumps(snap, allow_nan=False)

    def test_live_ventilation_reaching_the_target_marks_it_reached(self):
        state = self.held_state()
        sample = {"o2_pct": 20.9, "lel_pct": 0.0, "h2s_ppm": 0.0, "co_ppm": 0.0,
                  "alarms": {"any": False}, "alarm": False,
                  "ventilation": {"on": True, "tau_s": 15.0, "fraction_remaining": 0.083}}
        state.ingest_gas_json(json.dumps(sample))
        self.assertFalse(state.snapshot()["purge_hold"]["reached"])
        sample["ventilation"]["fraction_remaining"] = 0.0712
        state.ingest_gas_json(json.dumps(sample))
        hold = state.snapshot()["purge_hold"]
        self.assertTrue(hold["active"])          # still held until the re-test
        self.assertTrue(hold["reached"])
        self.assertEqual(hold["reached_fraction"], 0.0712)
        self.assertEqual(hold["fraction_remaining"], 0.0847)   # the quoted value never drifts

    def test_hold_ends_on_retest_authorization_new_permit_or_reset(self):
        for event in ({"kind": "atmos_test_start", "data": {"at_xy": [6.58, 0.0]}},
                      {"kind": "permit", "data": {"state": "ENTRY_AUTHORIZED",
                                                  "permit_id": "CSE-0428"}},
                      {"kind": "permit", "data": {"state": "PRE_ENTRY_TESTING",
                                                  "permit_id": "CSE-0429"}},
                      {"kind": "demo_reset", "source": "presenter", "data": {}}):
            state = self.held_state()
            state.add_event(dict({"source": "mission", "message": "x"}, **event))
            hold = state.snapshot()["purge_hold"]
            self.assertFalse(hold["active"], event)
            self.assertTrue(hold["released"], event)

    def test_older_mission_purge_incomplete_alone_opens_the_same_hold(self):
        # docs/evidence/events-ok1.jsonl predates the purge_hold event.
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "purge_incomplete", "message": "wait",
                         "data": {"purge_target_fraction": 0.08, "fraction_remaining": 0.0847,
                                  "wait_s": 1.7}})
        hold = state.snapshot()["purge_hold"]
        self.assertTrue(hold["active"])
        self.assertEqual((hold["fraction_remaining"], hold["purge_target"], hold["eta_s"]),
                         (0.0847, 0.08, 1.7))
        # A later structured event wins; a later fallback never overrides it.
        state.add_event({"source": "mission", "kind": "purge_hold", "message": "hold",
                         "data": dict(self.HOLD, fraction_remaining=0.0846)})
        state.add_event({"source": "mission", "kind": "purge_incomplete", "message": "wait",
                         "data": {"purge_target_fraction": 0.08, "fraction_remaining": 0.0999}})
        self.assertEqual(state.snapshot()["purge_hold"]["fraction_remaining"], 0.0846)

    def test_a_real_gas_failure_is_never_marked_held(self):
        state = self.held_state()
        state.add_event({"source": "mission", "kind": "atmos_probe", "message": "rows",
                         "data": {"samples": _probe_rows(h2s=38.9), "passed": False}})
        probe = state.snapshot()["probe"]
        self.assertTrue(probe["any_alarm"])
        self.assertFalse(probe["held"])

    def test_malformed_hold_data_is_ignored(self):
        state = rb.HudState()
        for data in (None, {"fraction_remaining": "lots"}, {"purge_target": 7},
                     {"fraction_remaining": float("nan")}):
            state.add_event({"source": "mission", "kind": "purge_hold", "message": "x",
                             "data": data})
        self.assertIsNone(state.snapshot()["purge_hold"])
        self.assertIsNone(rb.purge_hold_from("atmos_result", {"fraction_remaining": 0.1}))

    def test_mission_hud_and_agent_quote_one_percent_at_a_rounding_edge(self):
        # Review of V3: the gate judged 0.08251 (mission message "8.3 %"),
        # the event stores fraction 0.0825, and re-rounding that gave 8.2 in
        # the HUD bridge and the agent's next_step (JS toFixed: 8.3). Every
        # consumer now quotes the mission's percent_remaining.
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if repo not in sys.path:
            sys.path.insert(0, repo)
        from agent import mission as mission_mod, spot_agent
        purge = {"fraction_remaining": 0.08251, "wait_s": 1.2, "prior_failure": True,
                 "ventilation": {"on": True, "target_reachable": True}}
        hold = mission_mod.ConfinedSpaceMission._purge_hold(
            types.SimpleNamespace(purge_target=0.08), purge)
        self.assertEqual((hold["fraction_remaining"], hold["percent_remaining"]), (0.0825, 8.3))
        self.assertEqual("%.1f" % (100.0 * purge["fraction_remaining"]), "8.3")  # purge message
        self.assertIn("8.3 %", mission_mod.ConfinedSpaceMission._purge_hold_message(hold))
        self.assertEqual(rb.purge_hold_from("purge_hold", hold)["percent_remaining"], 8.3)
        state = rb.HudState()
        state.add_event({"source": "mission", "kind": "purge_hold", "message": "h",
                         "data": dict(hold)})
        self.assertEqual(state.snapshot()["purge_hold"]["percent_remaining"], 8.3)
        self.assertIn("8.3 percent of the unventilated", spot_agent._purge_hold_text(hold))
        with open(os.path.join(server.STATIC_DIR, "app.js"), encoding="utf-8") as handle:
            js = handle.read()
        self.assertIn("const pct0 = num(hold.percent_remaining);", js)
        # A bad percent falls back to the fraction (never NaN on screen).
        self.assertEqual(rb.purge_hold_from("purge_hold", dict(hold, percent_remaining=float("nan")))
                         ["percent_remaining"], 8.2)

    def test_purge_hold_streamed_in_ws_diff(self):
        bridge, client = make_client(1.0)
        with client, client.websocket_connect(WS_URL, headers=ws_headers()) as ws:
            ws.receive_json()   # hello
            bridge.state.add_event({"source": "mission", "kind": "purge_hold", "message": "h",
                                    "data": dict(self.HOLD)})
            msg, _ = receive_until(ws, lambda m: m["type"] == "diff"
                                   and (m["changes"].get("purge_hold") or {}).get("active"))
        self.assertEqual(msg["changes"]["purge_hold"]["fraction_remaining"], 0.0847)


class TestReplayMode(unittest.TestCase):
    """FakeBridge replay mode: the REAL dry-run event sequences from
    docs/evidence/ drive the HUD (no simulator)."""

    def replay(self, name, csv=False, **kw):
        paths = [os.path.join(EVIDENCE, "events-%s.jsonl" % name)]
        if csv:
            paths.append(os.path.join(EVIDENCE, "hud-audit-%s.csv" % name))
        return rb.ReplayBridge(paths, realtime=False, **kw)

    def run_to(self, bridge, sim_t, step=0.25):
        end = bridge.tau_of_sim(sim_t)
        tau = 0.0
        while tau < end:
            bridge.tick(tau)
            tau += step
        bridge.tick(end)
        return bridge.state.snapshot(include_events=True)

    def test_ok1_purge_hold_shows_the_recorded_numbers(self):
        bridge = self.replay("ok1")
        snap = self.run_to(bridge, 450.0)
        self.assertEqual(snap["server"]["mode"], "replay")
        self.assertEqual(snap["permit"]["state"], "PRE_ENTRY_TESTING")
        self.assertEqual(snap["permit"]["permit_id"], "CSE-0428")
        hold = snap["purge_hold"]
        self.assertTrue(hold["active"])
        self.assertEqual(hold["fraction_remaining"], 0.0847)   # what the mission logged
        self.assertEqual(hold["purge_target"], 0.08)
        self.assertTrue(hold["reached"])                       # 5 s later the purge is done
        self.assertLess(snap["ventilation"]["fraction_remaining"], 0.08)
        self.assertEqual(snap["ventilation"]["state"], "on")
        self.assertTrue(snap["probe"]["held"])
        self.assertFalse(snap["probe"]["any_alarm"])
        self.assertLess(abs(snap["pose"]["x"] - 6.58), 0.05)   # at the door mouth
        self.assertEqual(snap["alert"]["seq"], 0)
        self.assertLess(snap["link"]["gas"], 1.0)

    def test_ok1_completes_with_the_full_path_walked(self):
        bridge = self.replay("ok1")
        bridge.tick(bridge.tau_of_sim(590.0))    # one jump, like --replay-at: trail still filled
        snap = bridge.state.snapshot(include_events=True)
        self.assertEqual(snap["permit"]["state"], "COMPLETED")
        self.assertFalse(snap["purge_hold"]["active"])
        xs = [p[0] for p in bridge.state.snapshot(include_trail=True)["trail"]]
        self.assertLess(min(xs), -4.0)                          # reached the far wall
        self.assertGreater(snap["pose"]["x"], 6.5)              # and came back out
        self.assertFalse(snap["gas"]["alarm"])

    def test_inc2_alarm_flash_carries_the_recorded_trigger_and_reaction(self):
        bridge = self.replay("inc2")
        snap = self.run_to(bridge, 205.0)
        alert = snap["alert"]
        self.assertEqual(alert["title"], "SAFETY GUARD: RETREAT")
        self.assertIn("H2S 78.5 ppm", alert["reading"])
        self.assertEqual(alert["reaction_s"], 0.0002)
        self.assertTrue(alert["staged"])
        self.assertEqual(snap["permit"]["state"], "ABORTED")
        # The HUD clock runs on the recording's timeline: at the alarm the
        # flash is fresh; at the end of the recording (retreat done) the
        # alarm is older than app.js FLASH_MAX_AGE_S (10 s), so a page opened
        # then shows the header summary, not a stale full-screen flash.
        self.assertLess(snap["server"]["wall_time"] - alert["t_wall"], 2.0)
        snap = self.run_to(bridge, 230.0)                       # clamped to the recording
        self.assertEqual(snap["alert"]["seq"], 1)               # one incident, one flash
        self.assertGreater(snap["server"]["wall_time"] - snap["alert"]["t_wall"], 10.0)
        self.assertIn("retreated automatically", snap["alert"]["note"])
        self.assertFalse(snap["gas"]["alarm"])
        self.assertGreater(snap["pose"]["x"], 6.0)             # outside the tank
        self.assertFalse(snap["status"]["standing"])

    def test_csv_adds_the_agent_transcript_without_duplicating_mission_events(self):
        plain, merged = self.replay("ok1"), self.replay("ok1", csv=True)
        kinds = [r["kind"] for r in merged.records]
        self.assertIn("say", kinds)
        self.assertIn("tool_call", kinds)
        self.assertEqual(sum(1 for r in plain.records if r["kind"] == "atmos_result"),
                         sum(1 for k in kinds if k == "atmos_result"))
        snap = self.run_to(merged, 450.0)
        says = [e for e in snap["events"] if e["kind"] == "say"]
        self.assertTrue(any("8.5 percent" in e["message"] for e in says))

    def test_say_filter_and_hold_keep_the_scene_fresh(self):
        bridge = self.replay("inc2", csv=True, say_filter=lambda text: text.upper())
        snap = self.run_to(bridge, 192.5)
        says = [e["message"] for e in snap["events"] if e["kind"] == "say"]
        self.assertTrue(says and all(m == m.upper() for m in says))
        tau = bridge.tau_of_sim(192.5)
        bridge.tick(tau)
        time.sleep(0.3)
        bridge.tick(tau)                         # held: same scene time, fresh telemetry
        self.assertLess(bridge.state.snapshot()["link"]["gas"], 0.3)

    def test_bad_recording_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "empty.jsonl")
            with open(path, "w") as handle:
                handle.write("not json\n{}\n")
            with self.assertRaises(ValueError):
                rb.ReplayBridge([path], realtime=False)

    def test_server_cli_wires_replay(self):
        args = server.parse_args(["--replay", "a.jsonl", "--replay", "b.csv", "--replay-at", "450"])
        self.assertEqual(args.replay, ["a.jsonl", "b.csv"])
        self.assertEqual(args.replay_at, 450.0)
        self.assertIn("purge_hold", server.DIFF_KEYS)
        with mock.patch.object(server.logging, "basicConfig"), \
                self.assertLogs("hud.server", level="CRITICAL"):
            self.assertEqual(server.main(["--replay", "/nonexistent.jsonl"]), 2)


class TestPresenterPolish(unittest.TestCase):
    """Dry-run critic P2 items and V3 wording, checked in the page source."""

    @classmethod
    def setUpClass(cls):
        def read(name):
            with open(os.path.join(server.STATIC_DIR, name), encoding="utf-8") as handle:
                return handle.read()
        cls.js, cls.css, cls.html = read("app.js"), read("style.css"), read("index.html")

    def test_sub_millisecond_stop_reads_less_than_one_ms(self):
        self.assertIn('"< 1 ms"', self.js)
        self.assertIn("fmtReaction(r)", self.js)
        self.assertNotIn('Math.round(r * 1000) + " ms" : r.toFixed(1)', self.js)

    def test_trigger_and_now_are_labelled(self):
        self.assertIn('class="incident-tag">Trigger<', self.html)
        self.assertIn('id="incident-now"', self.html)
        self.assertIn('"now " + parts.join', self.js)
        self.assertIn('Gas at robot <span class="panel-now">Now</span>', self.html)

    def test_every_agent_tool_has_a_plain_language_label(self):
        sys.path.insert(0, os.path.dirname(server.STATIC_DIR.rsplit(os.sep, 1)[0]))
        from agent.spot_agent import TOOL_NAMES
        for name in TOOL_NAMES:
            self.assertRegex(self.js, r"\n\s+%s: \"[A-Z]" % name, name)
        self.assertIn("renderToolEvent", self.js)
        self.assertNotIn('"TOOL"', self.js)

    def test_purge_hold_is_amber_with_a_progress_bar_not_a_red_fail(self):
        for needle in ('"PURGE HOLD"', "Waiting for ventilation to reach target", "activeHold()",
                       '" left at test"', '"TARGET REACHED"', '"Ready to re-test"',
                       'cls = "vent-hold"', "fmtPct1(f0)", '"NOT AUTHORIZED"'):
            self.assertIn(needle, self.js)
        self.assertIn(".vent-hold", self.css)
        self.assertNotIn("Math.round(frac * 100)", self.js)   # one rounding rule: 0.1 %
        self.assertIn('id="vent-sub"', self.html)

    def test_replay_page_never_says_live_and_shows_no_permit_codes(self):
        # Review of the v5 screenshots: a green LIVE pill sat beside "REPLAY ·
        # RECORDED EVENTS"; the permit card and log printed "PRE_ENTRY_TESTING
        # -> ENTRY_AUTHORIZED"; the probe table said CO "1" under an agent
        # line quoting "carbon monoxide 0.8 ppm" (mission text uses 0.1 ppm).
        self.assertIn('server.mode === "replay" || server.mode === "fake" ? "CONNECTED" : "LIVE"',
                      self.js)
        self.assertIn("plainText(p.message", self.js)
        self.assertIn("plainText(ev.message)", self.js)
        self.assertIn('Object.keys(PERMIT).join("|")', self.js)
        self.assertRegex(self.js, r'id: "co", key: "co_ppm",[^}]*digits: 1 \}')

    def test_staging_disclosure_is_not_the_part_the_header_truncates(self):
        # v5 retreat-complete screenshot: the one-line header strip cut
        # "simulated excursion staged by the presenter" off at 1920 and 1280.
        body = self.js[self.js.index("function alertDetail("):]
        body = body[:body.index("\n  }\n")]
        self.assertLess(body.index("staged by the presenter"), body.index("fmtReaction(r)"))

    def test_log_top_edge_fades_instead_of_slicing_a_line(self):
        self.assertIn(".feed-list.is-clipped", self.css)
        self.assertIn("mask-image", self.css)
        self.assertIn('classList.toggle("is-clipped"', self.js)

    def test_map_labels_avoid_each_other(self):
        self.assertIn("const hits = (r) =>", self.js)
        self.assertIn('label("DOOR"', self.js)
        self.assertIn("spots.find(", self.js)


if __name__ == "__main__":
    unittest.main()
