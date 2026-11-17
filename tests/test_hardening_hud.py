"""Tests for the HUD hardening pass (threat register T2, T3, T16, T42).

One class per threat ID; every test docstring names the threat it covers.
A last class pins the fixes for the content scanner's findings in the HUD
files (release blockers in the threat model's 4.2).
Like tests/test_hud.py, these run on a Mac WITHOUT rclpy: the app is driven
through FakeBridge / HudState and FastAPI's in-process TestClient, so no
socket is opened and no AWS call is made.

Run from the repo root:

    python3 -m unittest tests.test_hardening_hud -v

Needs: fastapi, httpx (TestClient), pillow (same as tests/test_hud.py).
"""

import ast
import contextlib
import csv
import importlib.util
import io
import ipaddress
import json
import os
import re
import shutil
import subprocess
import sys
import types
import unittest
from unittest import mock

from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from hud import server  # noqa: E402
from sim import facade_guard as fg  # noqa: E402  (pure stdlib: the facade's own records)

rb = server.ros_bridge
audit = server.audit

HUD_URL = "http://localhost:8080"
WS_URL = "ws://localhost:8080/ws"


def _app(port=server.DEFAULT_PORT, **kw):
    return server.create_app(rb.FakeBridge(state=rb.HudState(), realtime=False), port=port, **kw)


def _client(app=None, base_url=HUD_URL):
    return TestClient(app if app is not None else _app(), base_url=base_url)


def _ws(client, url=WS_URL, origin=HUD_URL, host=None):
    """'hello' if /ws served the client, else the refusal: ('close', code)
    before accept, or ('http', status) when a middleware answered over HTTP."""
    headers = {} if origin is None else {"origin": origin}
    if host is not None:
        headers["host"] = host
    try:
        with client.websocket_connect(url, headers=headers) as ws:
            return ws.receive_json()["type"]
    except WebSocketDisconnect as error:
        status = getattr(error, "status_code", None)   # WebSocketDenialResponse
        return ("http", status) if status is not None else ("close", error.code)


def _clocked_state(mode="ros"):
    """HudState on a fake clock: returns (state, advance(seconds))."""
    now = [1000.0]
    state = rb.HudState(mode=mode, wall_clock=lambda: now[0], monotonic=lambda: now[0])

    def advance(dt):
        now[0] += dt
    return state, advance


def _gas(h2s=1.0, pockets=None):
    """A payload/gas sample (the facade's shape); ``pockets`` = injected_pockets."""
    alarm = h2s >= 10.0
    sample = {"o2_pct": 20.9, "lel_pct": 1.0, "h2s_ppm": h2s, "co_ppm": 1.0,
              "alarms": {"o2": False, "lel": False, "h2s": alarm, "co": False, "any": alarm},
              "alarm": alarm}
    if pockets is not None:
        sample["injected_pockets"] = pockets
    return json.dumps(sample)


def _csv_rows(state):
    text = audit.csv_text(state.events())
    return list(csv.DictReader(io.StringIO(text.lstrip("\ufeff"))))


class _FakeUvicorn:
    """Stands in for uvicorn in sys.modules: records run() instead of serving."""

    def __init__(self):
        self.calls = []
        self.module = types.ModuleType("uvicorn")
        self.module.run = lambda app, **kw: self.calls.append((app, kw))


# ---------------------------------------------------------------------------
# T2
# ---------------------------------------------------------------------------


class TestT2HostOriginAndCaps(unittest.TestCase):
    """T2: Host and Origin allowlists that follow --port, connection caps,
    and a loopback-only bind unless --allow-non-loopback (M2, option A.1)."""

    ROUTES = ("/", "/api/state", "/api/audit.csv", "/api/audit.jsonl", "/api/camera.jpg",
              "/static/app.js")

    def test_dns_rebinding_host_is_refused_on_every_route(self):
        """T2: after a rebind, the attacker page's requests carry its own name
        as Host; every route answers 400 and leaks nothing."""
        with _client() as client:
            for path in self.ROUTES:
                for host in ("attacker.example:8080", "attacker.example", "localhost.attacker.example:8080",
                             "192.0.2.10:8080", "[::1]:8080", ""):
                    resp = client.get(path, headers={"host": host})
                    self.assertEqual(resp.status_code, 400, (path, host))
                    self.assertNotIn("permit", resp.text, (path, host))
                for host in ("localhost:8080", "127.0.0.1:8080"):
                    self.assertIn(client.get(path, headers={"host": host}).status_code,
                                  (200, 503), (path, host))   # 503: no camera frame yet

    def test_only_the_configured_port_is_answered(self):
        """T2: the Host allowlist follows --port; another port, or no port,
        is refused with a message that names the right URL."""
        with _client(_app(port=9123), base_url="http://localhost:9123") as client:
            self.assertEqual(client.get("/api/state").status_code, 200)
            for host in ("localhost:8080", "127.0.0.1:9124", "localhost", "localhost:09123"):
                resp = client.get("/api/state", headers={"host": host})
                self.assertEqual(resp.status_code, 400, host)
            self.assertIn("http://localhost:9123/", client.get(
                "/api/state", headers={"host": "localhost:8080"}).text)
        # Browsers leave out :80, so port 80 answers the bare loopback names.
        self.assertEqual(server.allowed_hosts(80),
                         {"localhost", "127.0.0.1", "localhost:80", "127.0.0.1:80"})
        self.assertEqual(server.allowed_origins(8080),
                         {"http://localhost:8080", "http://127.0.0.1:8080"})

    def test_refused_requests_still_carry_the_security_headers(self):
        """T2 (keeps T4): a 400 from the Host checks still has the CSP, nosniff and
        Referrer-Policy headers."""
        with _client() as client:
            for host in ("attacker.example:8080", "localhost:9999"):
                resp = client.get("/api/state", headers={"host": host})
                self.assertEqual(resp.status_code, 400)
                self.assertIn("script-src 'self'", resp.headers["content-security-policy"])
                self.assertEqual(resp.headers["x-content-type-options"], "nosniff")
                self.assertEqual(resp.headers["referrer-policy"], "no-referrer")
            self.assertEqual(client.get("/api/state").headers["referrer-policy"], "no-referrer")

    def test_cross_site_and_rebound_websockets_are_refused(self):
        """T2: /ws accepts only the page's own Origin. A rebound page (its
        Host and Origin both the attacker's name), a foreign or look-alike
        Origin, and a missing Origin are all refused before accept."""
        with _client() as client:
            # A refusal test: this foreign WebSocket URL must be refused.
            # nosemgrep: javascript.lang.security.detect-insecure-websocket.detect-insecure-websocket
            self.assertEqual(_ws(client, "ws://attacker.example:8080/ws",
                                 origin="http://attacker.example:8080"), ("http", 400))
            for origin in ("http://attacker.example", "http://localhost:9999", "null",
                           "https://localhost:8080", "http://localhost.attacker.example:8080",
                           "http://localhost:8080.attacker.example", "file://", None):
                self.assertEqual(_ws(client, origin=origin), ("close", 1008), origin)
            self.assertEqual(_ws(client), "hello")
            self.assertEqual(_ws(client, "ws://127.0.0.1:8080/ws", origin="http://127.0.0.1:8080"),
                             "hello")

    def test_origin_allowlist_follows_the_port(self):
        """T2: with --port 9123 only the :9123 origins are served."""
        with _client(_app(port=9123), base_url="http://localhost:9123") as client:
            url = "ws://localhost:9123/ws"
            self.assertEqual(_ws(client, url, origin="http://localhost:9123"), "hello")
            self.assertEqual(_ws(client, url, origin="http://localhost:8080"), ("close", 1008))

    def test_websocket_clients_are_capped(self):
        """T2: past the cap a new /ws client is refused (1013, the page falls
        back to polling) and the HUD keeps serving the clients it has; a
        closed client frees its slot."""
        with _client(_app(max_ws_clients=3)) as client:
            with contextlib.ExitStack() as stack:
                for _ in range(3):
                    ws = stack.enter_context(client.websocket_connect(
                        WS_URL, headers={"origin": HUD_URL}))
                    self.assertEqual(ws.receive_json()["type"], "hello")
                self.assertEqual(_ws(client), ("close", 1013))
                self.assertEqual(client.get("/api/state").status_code, 200)   # polling still works
            self.assertEqual(_ws(client), "hello")
        self.assertLessEqual(server.MAX_WS_CLIENTS, 16)

    def _main(self, *argv):
        fake = _FakeUvicorn()
        with mock.patch.dict(sys.modules, {"uvicorn": fake.module}), \
                mock.patch.object(server.logging, "basicConfig"):
            code = server.main(["--fake"] + list(argv))
        return code, fake.calls

    def test_uvicorn_runs_with_concurrency_and_frame_caps(self):
        """T2: uvicorn gets limit_concurrency and a small ws_max_size, and the
        app it serves checks the same port uvicorn listens on."""
        code, calls = self._main("--port", "9001")
        self.assertEqual(code, 0)
        ((app, kw),) = calls
        self.assertEqual((kw["host"], kw["port"]), ("127.0.0.1", 9001))
        self.assertEqual(kw["limit_concurrency"], server.LIMIT_CONCURRENCY)
        self.assertLessEqual(server.LIMIT_CONCURRENCY, 64)
        self.assertEqual(kw["ws_max_size"], server.WS_MAX_SIZE)
        self.assertLessEqual(server.WS_MAX_SIZE, 65536)
        with TestClient(app, base_url="http://localhost:9001") as client:
            self.assertEqual(client.get("/api/state").status_code, 200)
            self.assertEqual(client.get("/api/state", headers={"host": "localhost:8080"}).status_code,
                             400)

    def test_non_loopback_bind_needs_the_explicit_flag(self):
        """T2: a non-loopback --host is refused (exit 2, nothing served)
        unless --allow-non-loopback is given, and then it is logged as a
        warning."""
        # The wildcard binds (0.0.0.0, ::) are what the HUD must refuse: the
        # unspecified addresses, as the standard library defines them.
        wildcards = (str(ipaddress.IPv4Address(0)), str(ipaddress.IPv6Address(0)))
        for host in (*wildcards, "192.0.2.10", "hud.example"):
            with self.assertLogs("hud.server", level="CRITICAL") as logs:
                code, calls = self._main("--host", host)
            self.assertEqual((code, calls), (2, []), host)
            self.assertIn("--allow-non-loopback", logs.output[0])
        with self.assertLogs("hud.server", level="WARNING") as logs:
            code, calls = self._main("--host", "192.0.2.10", "--allow-non-loopback")
        self.assertEqual(code, 0)
        self.assertEqual(calls[0][1]["host"], "192.0.2.10")
        self.assertTrue(any("non-loopback" in line and "WARNING" in line for line in logs.output))
        for host in ("127.0.0.1", "localhost", "::1", "127.0.0.2"):
            code, calls = self._main("--host", host)
            self.assertEqual((code, len(calls)), (0, 1), host)

    def test_out_of_range_port_is_refused(self):
        """T2: the allowlists are built from --port, so it must be a real TCP port."""
        for port in ("0", "70000"):
            with self.assertLogs("hud.server", level="CRITICAL"):
                self.assertEqual(self._main("--port", port), (2, []))



# ---------------------------------------------------------------------------
# T3
# ---------------------------------------------------------------------------


class TestT3ProvenanceIsVisible(unittest.TestCase):
    """T3: no message authentication (owner decision 2), so the HUD checks
    the self-declared source against an allowlist, marks the rest
    unverified in the page and the exports, lets them change nothing, and
    never takes "automatic" from the source field (M3)."""

    def test_sources_outside_the_allowlist_are_marked_unverified(self):
        """T3: every event's receipt stamp, CSV row and JSONL line says whether
        its source is allowlisted; look-alikes and the HUD's own name are not."""
        state = rb.HudState()
        for source in rb.EVENT_SOURCES:   # the facade publishes only its record kinds
            kind = "world_control" if source == rb.FACADE_SOURCE else "info"
            state.add_event({"source": source, "kind": kind, "message": "ok"})
        for source in ("attacker", "Guard", "presenter ", "hud", "", None, "=cmd"):
            state.add_event({"source": source, "kind": "info", "message": "claimed"})
        checks = [ev["hud"]["source_check"] for ev in state.events()]
        self.assertEqual(checks, ["allowlisted"] * len(rb.EVENT_SOURCES) + ["unverified"] * 7)
        rows = _csv_rows(state)
        self.assertEqual([r["source_check"] for r in rows], checks)
        self.assertEqual(rows[-1]["source"], "'=cmd")         # T4's formula guard still applies
        lines = [json.loads(line) for line in audit.jsonl_lines(state.events())]
        self.assertEqual([line["hud"]["source_check"] for line in lines], checks)
        # A publisher cannot pre-fill the HUD's own stamp.
        state.add_event({"source": "attacker", "kind": "info", "message": "x",
                         "hud": {"source_check": "allowlisted"}})
        self.assertEqual(state.events()[-1]["hud"]["source_check"], "unverified")

    def test_unverified_events_change_no_hud_state(self):
        """T3: a forged permit, checklist, purge hold, probe, reset, injection
        disclosure or retreat note from an unverified source is logged and
        changes nothing the presenter or the audit relies on."""
        state, advance = _clocked_state()
        state.add_event({"source": "mission", "kind": "permit", "message": "testing",
                         "data": {"permit_state": "PRE_ENTRY_TESTING", "permit_id": "CSE-1"}})
        before = state.snapshot()
        forged = [
            {"kind": "permit", "data": {"permit_state": "COMPLETED", "permit_id": "CSE-9"}},
            {"kind": "permit_prepared", "data": {"permit": {"isolation": True, "loto": True,
                                                            "attendant": "x", "rescue_plan": "y"}}},
            {"kind": "purge_hold", "data": {"fraction_remaining": 0.5, "purge_target": 0.08}},
            {"kind": "atmos_probe", "data": {"samples": [{"z": 0.2, "o2_pct": 20.9, "lel_pct": 0,
                                                          "h2s_ppm": 0, "co_ppm": 0}],
                                             "passed": True}},
            {"kind": "demo_reset", "data": {}},
            {"kind": "demo_inject", "data": {"gas": "h2s"}},
            {"kind": "retreat_complete", "data": {"cause": "atmosphere"}},
        ]
        for ev in forged:
            state.add_event(dict(ev, source="attacker", message="forged"))
        after = state.snapshot(include_events=True)
        for key in ("permit", "purge_hold", "probe", "trail_epoch"):
            self.assertEqual(after[key], before[key], key)
        self.assertEqual(after["alert"]["seq"], 0)
        self.assertEqual(len(after["events"]), 1 + len(forged))
        # The forged disclosure does not stage a later genuine alarm.
        state.ingest_gas_json(_gas(1.0, 1))
        state.ingest_gas_json(_gas(25.0, 1))
        self.assertFalse(state.snapshot()["alert"]["staged"])

    def test_unverified_alarm_flashes_labelled_and_never_touches_a_verified_one(self):
        """T3: an alarm claim from an unverified source still flashes (fail
        safe) but as UNVERIFIED ALARM; it cannot refine or prolong a verified
        incident, and a verified signal after it flashes anew."""
        state, advance = _clocked_state()
        state.add_event({"source": "attacker", "kind": "alarm", "message": "fake gas",
                         "data": {"auto_retreat": True, "reading": json.loads(_gas(50.0))}})
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["title"], alert["unverified"], alert["note"]),
                         (1, rb.UNVERIFIED_TITLE, True, None))
        advance(1.0)
        state.add_event({"source": "guard", "kind": "alarm", "message": "GAS ALARM: H2S 25 ppm",
                         "data": {"auto_retreat": True, "reading": json.loads(_gas(25.0))}})
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["seq"], alert["title"], alert["unverified"]),
                         (2, "SAFETY GUARD: RETREAT", False))
        self.assertIn("25.0", alert["reading"])                  # nothing carried over
        advance(1.0)
        state.add_event({"source": "attacker", "kind": "alarm", "message": "x",
                         "data": {"reading": json.loads(_gas(99.0)), "stop_command_s": 9.0}})
        state.add_event({"source": "attacker", "kind": "retreat_complete", "message": "out",
                         "data": {"cause": "atmosphere"}})
        self.assertEqual(state.snapshot()["alert"], alert)

    def test_the_source_field_never_makes_a_retreat_automatic(self):
        """T3: "automatic" (banner note, alert title, the CSV retreat column)
        comes from the mission's cause / auto_retreat fields, never from a
        self-declared source of "guard"."""
        state, _advance = _clocked_state()
        state.add_event({"source": "guard", "kind": "retreat_start", "message": "RETREAT",
                         "data": {"cause": "operator"}})
        alert = state.snapshot()["alert"]
        self.assertEqual((alert["title"], alert["note"]), ("ALARM", rb.COMMAND_RETREATING_NOTE))
        state.add_event({"source": "mission", "kind": "retreat_complete", "message": "out",
                         "data": {"cause": "atmosphere"}})
        state.add_event({"source": "attacker", "kind": "retreat_complete", "message": "out",
                         "data": {"cause": "atmosphere"}})
        self.assertEqual([ev["hud"]["retreat"] for ev in state.events()],
                         ["commanded", "automatic", None])
        self.assertEqual([r["retreat"] for r in _csv_rows(state)], ["commanded", "automatic", ""])

    def test_a_reset_during_an_alarm_or_retreat_is_refused(self):
        """T3: a presenter demo_reset (forged or not) cannot clear an ABORTED
        permit while the gas alarm is on or a retreat is under way; it is
        logged as refused, and accepted once the incident is over."""
        state, advance = _clocked_state()
        state.add_event({"source": "mission", "kind": "permit", "message": "in",
                         "data": {"permit_state": "ENTRY_IN_PROGRESS", "permit_id": "CSE-2"}})
        state.ingest_gas_json(_gas(1.0))
        state.ingest_gas_json(_gas(30.0))
        state.add_event({"source": "guard", "kind": "alarm", "message": "GAS ALARM",
                         "data": {"auto_retreat": True}})
        state.add_event({"source": "guard", "kind": "permit", "message": "aborted",
                         "data": {"state": "ABORTED", "cause": "atmosphere"}})
        reset = {"source": "presenter", "kind": "demo_reset", "message": "reset"}
        state.add_event(reset)
        self.assertEqual(state.snapshot()["permit"]["state"], "ABORTED")
        self.assertEqual(state.events()[-1]["hud"]["reset_refused"], "gas alarm active")
        state.ingest_gas_json(_gas(1.0))
        advance(10.0)
        state.add_event(reset)
        self.assertEqual(state.snapshot()["permit"]["state"], "ABORTED")
        self.assertEqual(state.events()[-1]["hud"]["reset_refused"], "retreat in progress")
        state.add_event({"source": "guard", "kind": "retreat_complete", "message": "out",
                         "data": {"cause": "atmosphere"}})
        state.add_event(reset)
        self.assertEqual(state.snapshot()["permit"]["state"], "NOT_STARTED")
        self.assertNotIn("reset_refused", state.events()[-1]["hud"])
        # A retreat that never reports its end stops blocking after ALERT_ANNOTATE_S.
        state.add_event({"source": "mission", "kind": "retreat_start", "message": "RETREAT",
                         "data": {"cause": "operator"}})
        advance(rb.ALERT_ANNOTATE_S + 1.0)
        state.add_event(reset)
        self.assertNotIn("reset_refused", state.events()[-1]["hud"])

    def test_staging_follows_the_interface_contract(self):
        """T3 (contract): presenter-staged needs a disclosure (the HUD's, or
        the guard's staged_by_presenter) AND payload/gas injected_pockets > 0;
        anything less, including a sim that reports no count, is genuine."""
        def alarm(state, claimed):
            data = {"auto_retreat": True, "reading": json.loads(_gas(30.0))}
            if claimed:
                data["staged_by_presenter"] = [{"kind": "demo_inject", "source": "presenter"}]
            state.add_event({"source": "guard", "kind": "alarm", "message": "GAS ALARM",
                             "data": data})
            return state.snapshot()["alert"]["staged"]

        cases = (  # (pockets reported on payload/gas, disclosure source, guard claims) -> staged
            (None, None, True, False),
            (None, "presenter", True, False),
            (0, "presenter", True, False),
            (1, None, False, False),
            (1, "attacker", False, False),
            (1, None, True, True),
            (1, "presenter", False, True),
        )
        for pockets, disclosure, claimed, staged in cases:
            state, _advance = _clocked_state()
            state.ingest_gas_json(_gas(1.0, pockets))
            if disclosure:
                state.add_event({"source": disclosure, "kind": "demo_inject", "message": "x"})
            self.assertEqual(alarm(state, claimed), staged, (pockets, disclosure, claimed))
        # The count may trail the event across topics: a count > 0 within
        # STAGED_LATE_S of the flash still stages it.
        state, advance = _clocked_state()
        state.ingest_gas_json(_gas(1.0, 0))
        self.assertFalse(alarm(state, True))
        advance(0.3)
        state.ingest_gas_json(_gas(30.0, 1))
        self.assertTrue(state.snapshot()["alert"]["staged"])

    def test_page_badges_unverified_sources_and_refused_resets(self):
        """T3: the log badges an unverified source, the alarm banner says so,
        and a refused reset is tagged; text still goes in with textContent (T4)."""
        with open(os.path.join(server.STATIC_DIR, "app.js"), encoding="utf-8") as handle:
            js = handle.read()
        with open(os.path.join(server.STATIC_DIR, "style.css"), encoding="utf-8") as handle:
            css = handle.read()
        self.assertIn('ev.hud.source_check === "unverified"', js)
        self.assertIn('"UNVERIFIED SOURCE"', js)
        self.assertIn('"RESET REFUSED"', js)
        detail = js[js.index("function alertDetail("):]
        detail = detail[:detail.index("\n  }\n")]
        self.assertIn("unverified event source", detail)
        self.assertIn(".tag-unverified", css)
        self.assertNotRegex(js, r"\.(inner|outer)HTML\b")


# ---------------------------------------------------------------------------
# T3 / T23: the facade's own records on the HUD
# ---------------------------------------------------------------------------


def _facade(control, params=None, error=None, publishers=("/presenter_gas",), sim_time=50.0):
    """One facade world_control record, built and serialised by the same
    sim/facade_guard.py code sim/isaac_app.py publishes it with."""
    record = fg.world_control_event(control, params, error, list(publishers), None, 0)
    out = fg.FacadeAudit()
    out.record(*record, sim_time=sim_time)
    (text,) = out.drain()
    return text


def _facade_service():
    out = fg.FacadeAudit()
    out.record(*fg.service_event("estop/hard", (True, False), (False, True)), sim_time=51.0)
    out.record(*fg.PublisherWatch("/demo_spot/cmd_vel").update(["/a", "/b"]), sim_time=52.0)
    return out.drain()


# Disclosures as the shipped senders publish them.
VENT_ON = {"source": "operator", "kind": "ventilation", "message": "Operator: forced ventilation ON",
           "data": {"on": True, "tau_s": 30.0}}                                 # host/inject_gas.sh
MISSION_VENT_ON = {"source": "mission", "kind": "ventilation", "message": "Forced ventilation ON",
                   "data": {"on": True, "ok": True, "tau_s": 60.0, "purge_estimate_s": 190.0,
                            "simulated": True}}                                 # agent/mission.py
MISSION_VENT_PROGRESS = {"source": "mission", "kind": "ventilation", "message": "Ventilating",
                         "data": {"on": True, "tau_s": 2.0, "waited_s": 5.0}}
DEMO_INJECT = {"source": "presenter", "kind": "demo_inject", "message": "PRESENTER DEMO: H2S",
               "data": {"gas": "h2s", "ppm": 80.0, "x": 14.0, "y": 0.0, "z": 0.3, "radius": 1.0}}
DEMO_RESET = {"source": "presenter", "kind": "demo_reset", "message": "PRESENTER DEMO: reset",
              "data": {"reset_ventilation": True, "self_right": True, "battery_reset": True,
                       "view_reset": True}}                                     # host/reset_spot.sh
INJECT = {"gas": "h2s", "ppm": 80.0, "x": 14.0, "y": 0.0, "z": 0.3, "radius": 1.0}


class TestT23FacadeRecordsOnTheHud(unittest.TestCase):
    """T3 / T23 (M3, M23): the facade's world_control, facade_service and
    facade_publishers records are an allowlisted source on the HUD, kept in
    a protected buffer, shown in the log and the exports, and they change no
    HUD state; an atmosphere change (inject, clear, ventilation) that no
    presenter disclosure accompanies is flagged."""

    def _undisclosed(self, state):
        return [ev for ev in state.events() if ev["kind"] == "undisclosed_change"]

    def test_facade_records_are_allowlisted_protected_and_inert(self):
        """T3/T23: facade records are stamped allowlisted and safety in the
        receipt stamp, the CSV and the JSONL, appear on the page, and change
        no permit, alert, staging or probe state."""
        state, advance = _clocked_state()
        before = state.snapshot()
        state.ingest_event_json(_facade("gas/ventilation", {"on": True, "tau_s": 30.0}))
        for text in _facade_service():
            state.ingest_event_json(text)
        state.ingest_event_json(_facade("gas/ventilation", None, ValueError("tau_s 0.5")))
        events = state.events()
        self.assertEqual([ev["kind"] for ev in events],
                         ["world_control", "facade_service", "facade_publishers", "world_control"])
        self.assertEqual({(ev["hud"]["source_check"], ev["hud"]["record"]) for ev in events},
                         {("allowlisted", "safety")})
        self.assertEqual({r["source_check"] for r in _csv_rows(state)}, {"allowlisted"})
        lines = [json.loads(line) for line in audit.jsonl_lines(events)]
        self.assertEqual({line["hud"]["source_check"] for line in lines}, {"allowlisted"})
        after = state.snapshot(include_events=True)
        self.assertEqual([ev["source"] for ev in after["events"]], ["facade"] * 4)
        for key in ("permit", "purge_hold", "probe", "alert", "trail_epoch"):
            self.assertEqual(after[key], before[key], key)
        self.assertIsNone(state._inject_mono)

    def test_the_facade_name_covers_only_the_facade_record_kinds(self):
        """T3: the facade publishes nothing else, so any other kind under its
        name (a permit, an alarm, a disclosure) is unverified and inert."""
        state, _advance = _clocked_state()
        forged = [
            {"kind": "permit", "data": {"permit_state": "ENTRY_AUTHORIZED", "permit_id": "CSE-9"}},
            {"kind": "demo_inject", "data": {"gas": "h2s"}},
            {"kind": "demo_reset", "data": {}},
            {"kind": "info", "data": {}},
        ]
        for ev in forged:
            state.add_event(dict(ev, source="facade", message="forged"))
        self.assertEqual({ev["hud"]["source_check"] for ev in state.events()}, {"unverified"})
        self.assertEqual(state.snapshot()["permit"]["state"], "NOT_STARTED")
        self.assertIsNone(state._inject_mono)
        # A world_control record with a permit or alarm claim changes no state either.
        state.add_event({"source": "facade", "kind": "world_control", "message": "x",
                         "data": {"permit_state": "COMPLETED", "control": "view"}})
        self.assertEqual(state.snapshot()["permit"]["state"], "NOT_STARTED")

    def test_facade_records_survive_a_flood_of_other_records(self):
        """T3/T42: other sources' and unverified floods cannot evict the
        facade's records from the export, the CSV or the page."""
        state, advance = _clocked_state()
        state.ingest_event_json(_facade("gas/ventilation", {"on": True, "tau_s": 2.0}))
        for text in _facade_service():
            state.ingest_event_json(text)
        mine = [ev["message"] for ev in state.events()]
        for i in range(3000):
            state.add_event({"source": "mission", "kind": "info", "message": "noise %d" % i})
            state.add_event({"source": "guard", "kind": "alarm", "message": "alarm %d" % i})
            state.add_event({"source": "attacker-%d" % i, "kind": "world_control", "message": "f"})
        exported = [ev["message"] for ev in state.events()]
        page = [ev["message"] for ev in state.snapshot(include_events=True)["events"]]
        csv_messages = [r["message"] for r in _csv_rows(state)]
        for message in mine:
            self.assertIn(message, exported)
            self.assertIn(message, page)
            self.assertIn(message, csv_messages)
        self.assertNotIn("safety/facade", state.ingest_counts()["evicted"])

    def test_refused_controls_cannot_evict_applied_ones(self):
        """T23/T42: the facade records every refused safety control (up to
        2.5/s at its input limits); those and calls that changed nothing have
        their own buffer, so a refusal flood cannot push out what was applied."""
        state, advance = _clocked_state()
        state.ingest_event_json(_facade("gas/ventilation", {"on": True, "tau_s": 2.0}))
        for text in _facade_service():
            state.ingest_event_json(text)
        mine = [ev["message"] for ev in state.events()]
        unchanged = fg.service_event("estop/hard", (False, True), (False, True))
        self.assertFalse(unchanged[3])
        for _ in range(3 * rb.SAFETY_BUFFER):
            state.ingest_event_json(_facade("gas/inject", None, ValueError("radius out of range")))
            out = fg.FacadeAudit()
            out.record(*unchanged, sim_time=53.0)
            state.ingest_event_json(out.drain()[0])
        exported = [ev["message"] for ev in state.events()]
        page = [ev["message"] for ev in state.snapshot(include_events=True)["events"]]
        for message in mine:
            self.assertIn(message, exported)
            self.assertIn(message, page)
        evicted = state.ingest_counts()["evicted"]
        self.assertIn("safety/facade/no-change", evicted)
        self.assertNotIn("safety/facade", evicted)
        kept = [ev for ev in state.events() if ev["source"] == "facade"]
        self.assertEqual(len(kept), len(mine) + rb.SAFETY_BUFFER)

    def test_an_undisclosed_atmosphere_change_is_flagged(self):
        """T23 (M23): an applied inject, clear or ventilation change with no
        presenter disclosure within DISCLOSURE_WINDOW_S becomes the HUD's own
        local record, in the CSV, and is counted in the snapshot."""
        state, advance = _clocked_state()
        state.ingest_event_json(_facade("gas/ventilation", {"on": True, "tau_s": 2.0},
                                        publishers=("/intruder",)))
        advance(rb.DISCLOSURE_WINDOW_S - 1.0)
        self.assertEqual(state.snapshot()["world"]["undisclosed"], 0)   # still waiting
        self.assertEqual(state.snapshot()["world"]["pending"], 1)
        advance(2.0)
        world = state.snapshot()["world"]
        self.assertEqual((world["undisclosed"], world["pending"]), (1, 0))
        self.assertEqual(world["last"]["control"], "gas/ventilation")
        self.assertEqual(world["last"]["params"], {"on": True, "tau_s": 2.0})
        self.assertEqual(world["last"]["publishers"], ["/intruder"])
        (flag,) = self._undisclosed(state)
        self.assertEqual((flag["source"], flag["hud"]["source_check"], flag["hud"]["record"]),
                         ("hud", "local", "local"))
        self.assertIn("gas/ventilation", flag["message"])
        rows = [r for r in _csv_rows(state) if r["kind"] == "undisclosed_change"]
        self.assertEqual([r["source_check"] for r in rows], ["local"])
        # A late disclosure does not undo the flag; the next change is judged anew.
        state.add_event(VENT_ON)
        self.assertEqual(state.snapshot()["world"]["undisclosed"], 1)
        # A DDS publisher cannot write the HUD's flag or clear it.
        state.add_event({"source": "hud", "kind": "undisclosed_change", "message": "forged"})
        self.assertEqual(state.events()[-1]["hud"]["source_check"], "unverified")
        self.assertEqual(state.snapshot()["world"]["undisclosed"], 1)

    def test_the_shipped_senders_disclosures_are_matched(self):
        """T23 (M23): host/inject_gas.sh, host/reset_spot.sh, the mission and
        the agent's presenter tools disclose their changes, before or after
        the facade's record, so none of them is flagged."""
        cases = (
            ("inject_gas.sh inject", [DEMO_INJECT, ("gas/inject", INJECT)]),
            ("record before the disclosure", [("gas/inject", INJECT), 0.5, DEMO_INJECT]),
            ("inject_gas.sh clear", [("gas/clear", {}), {"source": "presenter", "kind": "demo_clear",
                                                         "message": "cleared", "data": {}}]),
            ("inject_gas.sh vent on", [("gas/ventilation", {"on": True, "tau_s": 30.0}), VENT_ON]),
            ("inject_gas.sh vent off", [("gas/ventilation", {"on": False, "tau_s": None}),
                                        dict(VENT_ON, data={"on": False})]),
            ("inject_gas.sh vent reset", [("gas/ventilation", {"reset": True}),
                                          {"source": "presenter", "kind": "ventilation",
                                           "message": "reset", "data": {"reset": True}}]),
            ("reset_spot.sh after self_right", [("gas/clear", {}), ("gas/ventilation", {"reset": True}),
                                                ("battery/reset", {}), 11.5, DEMO_RESET]),
            ("mission ventilation", [("gas/ventilation", {"on": True, "tau_s": 60.0}),
                                     MISSION_VENT_ON]),
            ("mission demo reset", [("gas/clear", {}), ("gas/ventilation", {"reset": True}),
                                    {"source": "mission", "kind": "sim_control", "message": "reset",
                                     "data": {"ok": True, "cleared": True,
                                              "ventilation_reset": True}}]),
            ("agent inject tool", [("gas/inject", INJECT),
                                   {"source": "agent", "kind": "gas_injected", "message": "x",
                                    "data": dict(INJECT)}]),
            ("agent clear tool", [("gas/clear", {}), {"source": "agent", "kind": "gas_cleared",
                                                      "message": "x", "data": {"ok": True}}]),
            ("run_mission harness", [("gas/inject", INJECT),
                                     {"source": "mission", "kind": "demo_inject", "message": "x",
                                      "data": dict(INJECT, waypoint=2, published=True)}]),
            ("refused and view controls need none", [("gas/ventilation", None, "tau_s 0.5"),
                                                     ("view", {"camera": "auto", "ui": None}),
                                                     ("battery/reset", {})]),
        )
        for name, steps in cases:
            state, advance = _clocked_state()
            for step in steps:
                if isinstance(step, float):
                    advance(step)
                elif isinstance(step, dict):
                    state.add_event(step)
                else:
                    control, params = step[0], step[1]
                    error = ValueError(step[2]) if len(step) > 2 else None
                    state.ingest_event_json(_facade(control, params, error))
            advance(rb.DISCLOSURE_WINDOW_S + 1.0)
            world = state.snapshot()["world"]
            self.assertEqual((world["undisclosed"], world["pending"]), (0, 0), name)
            self.assertEqual(self._undisclosed(state), [], name)

    def test_a_disclosure_that_does_not_match_is_no_cover(self):
        """T23 (M23): the disclosure must come from an allowlisted publisher,
        say the same thing, and covers one change: an unverified source, a
        purge progress line, another time constant, another gas, a
        disclosure outside the window or a second change are all flagged."""
        cases = (
            ("unverified source", [dict(VENT_ON, source="attacker"),
                                   ("gas/ventilation", {"on": True, "tau_s": 30.0})], 1),
            ("purge progress is not a disclosure", [MISSION_VENT_PROGRESS,
                                                    ("gas/ventilation", {"on": True, "tau_s": 2.0})], 1),
            ("another time constant", [MISSION_VENT_ON,
                                       ("gas/ventilation", {"on": True, "tau_s": 2.0})], 1),
            ("on is not off", [VENT_ON, ("gas/ventilation", {"on": False, "tau_s": None})], 1),
            ("another gas", [DEMO_INJECT, ("gas/inject", dict(INJECT, gas="lel", ppm=40.0))], 1),
            ("an unpublished harness inject", [dict(DEMO_INJECT, source="mission",
                                                    data=dict(INJECT, published=False)),
                                               ("gas/inject", INJECT)], 1),
            ("too early", [VENT_ON, rb.DISCLOSURE_WINDOW_S + 1.0,
                           ("gas/ventilation", {"on": True, "tau_s": 30.0})], 1),
            ("one disclosure, two changes", [DEMO_INJECT, ("gas/inject", INJECT),
                                             ("gas/inject", INJECT)], 1),
            ("a clear is not a ventilation reset", [{"source": "presenter", "kind": "demo_clear",
                                                     "message": "x", "data": {}},
                                                    ("gas/ventilation", {"reset": True})], 1),
            ("a forged facade disclosure", [dict(VENT_ON, source="facade"),
                                            ("gas/ventilation", {"on": True, "tau_s": 30.0})], 1),
        )
        for name, steps, flagged in cases:
            state, advance = _clocked_state()
            for step in steps:
                if isinstance(step, float):
                    advance(step)
                elif isinstance(step, dict):
                    state.add_event(step)
                else:
                    state.ingest_event_json(_facade(*step))
            advance(rb.DISCLOSURE_WINDOW_S + 1.0)
            self.assertEqual(state.snapshot()["world"]["undisclosed"], flagged, name)
            self.assertEqual(len(self._undisclosed(state)), flagged, name)

    def test_replayed_history_and_a_flood_of_changes_fail_closed(self):
        """T23/T42 (M23): TRANSIENT_LOCAL history is not judged (its
        disclosures may have gone with their short-lived writers); a flood of
        changes never drops one unflagged, and the flags have their own
        buffer, so they cannot evict the HUD's gas alarm edges."""
        state, advance = _clocked_state()
        state.ingest_event_json(_facade("gas/clear", {}), replay=True)
        advance(rb.DISCLOSURE_WINDOW_S + 1.0)
        self.assertEqual(state.snapshot()["world"], {"undisclosed": 0, "pending": 0, "last": None,
                                                      "window_s": rb.DISCLOSURE_WINDOW_S})
        state.ingest_gas_json(_gas(1.0))
        state.ingest_gas_json(_gas(30.0))           # one HUD gas_alarm_on edge
        n = rb.WORLD_PENDING_MAX + rb.LOCAL_BUFFER + 10
        for _ in range(n):
            state.ingest_event_json(_facade("gas/clear", {}))
        advance(rb.DISCLOSURE_WINDOW_S + 1.0)
        self.assertEqual(state.snapshot()["world"]["undisclosed"], n)
        kinds = [ev["kind"] for ev in state.events() if ev["source"] == "hud"]
        self.assertIn("gas_alarm_on", kinds)
        self.assertEqual(kinds.count("undisclosed_change"), rb.LOCAL_BUFFER)
        self.assertEqual(state.ingest_counts()["evicted"]["local/world"], n - rb.LOCAL_BUFFER)

    def test_the_page_shows_facade_records_and_the_undisclosed_flag(self):
        """T3/T23: the log labels the simulator's records and whether a control
        was applied or refused, shows the HUD's undisclosed-change line, and
        the header has a badge for it; the snapshot streams it."""
        self.assertIn("world", server.DIFF_KEYS)
        with open(os.path.join(server.STATIC_DIR, "app.js"), encoding="utf-8") as handle:
            js = handle.read()
        with open(os.path.join(server.STATIC_DIR, "index.html"), encoding="utf-8") as handle:
            html = handle.read()
        with open(os.path.join(server.STATIC_DIR, "style.css"), encoding="utf-8") as handle:
            css = handle.read()
        self.assertIn('facade: "SIMULATOR"', js)
        self.assertIn('"APPLIED"', js)
        self.assertIn('"REFUSED"', js)
        self.assertIn('"NO DISCLOSURE"', js)
        self.assertIn('kind === "undisclosed_change"', js)
        self.assertIn('id="world-badge"', html)
        self.assertIn('$("world-badge")', js)
        self.assertIn('"UNDISCLOSED SIM CHANGE · "', js)
        self.assertIn(".badge-undisclosed", css)
        render = js[js.index("  function render() {"):]
        self.assertIn("renderWorld();", render[:render.index("\n  }\n")])
        self.assertNotRegex(js, r"\.(inner|outer)HTML\b")



# ---------------------------------------------------------------------------
# T42
# ---------------------------------------------------------------------------


def _incident(state, advance):
    """The safety records of one guard retreat, in agent/mission.py's order."""
    reading = json.loads(_gas(25.0, 1))
    records = [
        {"source": "presenter", "kind": "demo_inject", "message": "PRESENTER DEMO: H2S pocket"},
        {"source": "mission", "kind": "atmos_result", "message": "H2S PASS", "data": {"passed": True}},
        {"source": "mission", "kind": "permit", "message": "authorized",
         "data": {"permit_state": "ENTRY_AUTHORIZED", "permit_id": "CSE-7"}},
        {"source": "guard", "kind": "alarm", "message": "GAS ALARM: H2S 25.0 ppm",
         "data": {"auto_retreat": True, "reading": reading}},
        {"source": "guard", "kind": "permit", "message": "aborted",
         "data": {"state": "ABORTED", "cause": "atmosphere"}},
        {"source": "guard", "kind": "retreat_start", "message": "RETREAT",
         "data": {"cause": "atmosphere"}},
        {"source": "guard", "kind": "retreat_complete", "message": "out",
         "data": {"cause": "atmosphere"}},
    ]
    for record in records:
        state.add_event(record)
        advance(0.1)
    return [r["message"] for r in records]


class TestT42FloodKeepsRealAlarms(unittest.TestCase):
    """T42 (HUD part): per-source/kind buffers keep guard, mission and gas
    alarm records in the page and the audit export through a flood, and
    per-topic ingest limits drop and count the excess visibly (M42)."""

    FLOOD = 3000

    def _flood(self, state):
        for i in range(self.FLOOD):
            state.add_event({"source": "agent", "kind": "say", "message": "chatter %d" % i})
            state.add_event({"source": "mission", "kind": "info", "message": "noise %d" % i})
            state.add_event({"source": "guard", "kind": "monitor", "message": "reading %d" % i})
            state.add_event({"source": "attacker-%d" % i, "kind": "alarm", "message": "fake %d" % i})

    def test_a_flood_cannot_push_alarm_permit_or_retreat_records_out(self):
        """T42: thousands of routine, monitor and unverified events later, the
        incident's records are still in the audit export, the CSV and the
        page's log; only routine and unverified buffers lost records."""
        state, advance = _clocked_state()
        messages = _incident(state, advance)
        self._flood(state)
        exported = [ev["message"] for ev in state.events()]
        page = [ev["message"] for ev in state.snapshot(include_events=True)["events"]]
        csv_messages = [r["message"] for r in _csv_rows(state)]
        for message in messages:
            self.assertIn(message, exported)
            self.assertIn(message, page)
            self.assertIn(message, csv_messages)
        evicted = state.ingest_counts()["evicted"]
        self.assertEqual(set(evicted), {"routine/agent", "routine/mission", "routine/guard",
                                        "unverified"})
        self.assertNotIn("safety/guard", evicted)
        self.assertLessEqual(len(exported), len(rb.EVENT_SOURCES) * (rb.SAFETY_BUFFER
                                                                     + rb.ROUTINE_BUFFER)
                             + rb.SAFETY_BUFFER          # the facade's no-change buffer
                             + rb.UNVERIFIED_BUFFER + 2 * rb.LOCAL_BUFFER)
        self.assertLessEqual(len(page), len(exported))

    def test_the_hud_records_the_simulators_gas_alarm_edges(self):
        """T42: payload/gas alarm on/off edges are the HUD's own records
        (source hud, check "local"), kept through a flood, with the gas
        values in the CSV."""
        state, advance = _clocked_state()
        state.ingest_gas_json(_gas(1.0))
        state.ingest_gas_json(_gas(30.0))
        state.ingest_gas_json(_gas(30.0))       # still on: no new record
        advance(2.0)
        state.ingest_gas_json(_gas(1.0))
        self._flood(state)
        rows = [r for r in _csv_rows(state) if r["source"] == "hud"]
        self.assertEqual([(r["kind"], r["source_check"], r["gas_alarm"]) for r in rows],
                         [("gas_alarm_on", "local", "true"), ("gas_alarm_off", "local", "false")])
        self.assertEqual(float(rows[0]["h2s_ppm"]), 30.0)
        # A DDS publisher cannot write into the HUD's own buffer.
        state.add_event({"source": "hud", "kind": "gas_alarm_off", "message": "forged"})
        self.assertEqual(state.events()[-1]["hud"]["source_check"], "unverified")

    def test_ingest_is_rate_limited_per_topic_and_the_excess_counted(self):
        """T42: past a topic's burst, messages are dropped before parsing and
        counted per topic; other topics are unaffected; the limit refills."""
        state = rb.HudState()
        bridge = rb.RosBridge(state)
        now = [0.0]
        bridge._clock = lambda: now[0]
        events = bridge._guard("payload/events", lambda m: state.ingest_event_json(m.data))
        gas = bridge._guard("payload/gas", lambda m: state.ingest_gas_json(m.data))
        msg = types.SimpleNamespace
        rate, burst = rb.INGEST_LIMITS["payload/events"]
        for i in range(burst + 1000):
            events(msg(data=json.dumps({"source": "attacker", "kind": "info", "message": str(i)})))
        gas(msg(data=_gas(1.0)))
        counts = state.ingest_counts()
        self.assertEqual(counts["dropped"], {"payload/events": 1000})
        self.assertEqual(state.snapshot()["events_seq"], burst)
        self.assertIsNotNone(state.snapshot()["gas"])
        now[0] += 1.0
        later = int(rate) + 100
        for i in range(later):
            events(msg(data=json.dumps({"source": "mission", "kind": "info", "message": "later"})))
        self.assertEqual(state.snapshot()["events_seq"], burst + int(rate))
        self.assertEqual(state.ingest_counts()["dropped"]["payload/events"], 1000 + later - int(rate))
        self.assertGreater(state.ingest_counts()["flood_total"], 1000)
        # A malformed message inside the limit is parsed, refused and counted.
        now[0] += 1.0
        events(msg(data="{nope"))
        self.assertEqual(state.ingest_counts()["rejected"], {"payload/events": 1})

    def test_a_flood_under_one_name_does_not_starve_the_guard(self):
        """T42: after parsing, payload/events is limited per declared source, so
        thousands of records a second under one name (any name, allowlisted or
        not) drop that name's excess, counted, and every guard and mission record
        sent alongside still reaches the export. A flood under the guard's own
        name would still drop the guard's records (sources are self-declared, T3),
        and one above the per-topic limit drops everyone's."""
        state = rb.HudState()
        bridge = rb.RosBridge(state)
        now = [0.0]
        bridge._clock = lambda: now[0]
        events = bridge._guard("payload/events", bridge._events_by_source())
        msg = types.SimpleNamespace
        genuine = []
        for step in range(60):                                   # 3 s in 50 ms ticks
            now[0] += 0.05
            for i in range(9):                                   # 180 flood records a second
                events(msg(data=json.dumps({"source": "attacker", "kind": "alarm", "message": "f"})))
            text = "GAS ALARM %d" % step
            genuine.append(text)
            events(msg(data=json.dumps({"source": "guard", "kind": "alarm", "message": text,
                                        "data": {"auto_retreat": True}})))
        exported = [ev["message"] for ev in state.events()]
        for text in genuine:
            self.assertIn(text, exported)
        self.assertGreater(state.ingest_counts()["dropped"]["payload/events"], 0)
        # The 16 named buckets are full: any further name shares one bucket.
        for i in range(40):
            events(msg(data=json.dumps({"source": "n%d" % i, "kind": "info", "message": "x"})))
        now[0] += 0.05
        events(msg(data=json.dumps({"source": "guard", "kind": "alarm", "message": "still here"})))
        self.assertIn("still here", [ev["message"] for ev in state.events()])

    def test_real_rates_and_the_startup_history_are_never_dropped(self):
        """T42: the limits sit above the observed peaks (events 7/s, gas 4 Hz,
        status 2 Hz, odometry 50 Hz, camera 10 Hz) and the payload/events
        burst covers the TRANSIENT_LOCAL history a restarted HUD receives."""
        state = rb.HudState()
        bridge = rb.RosBridge(state)
        now = [0.0]
        bridge._clock = lambda: now[0]
        seen = {}

        def counter(topic):
            return bridge._guard(topic, lambda m: seen.__setitem__(topic, seen.get(topic, 0) + 1))
        callbacks = {topic: counter(topic) for topic in rb.INGEST_LIMITS}
        for _ in range(3 * rb.EVENT_HISTORY_DEPTH // 2):         # history from several writers
            callbacks["payload/events"](None)
        peaks = {"payload/events": 10, "payload/gas": 8, "payload/status": 2, "odometry": 50,
                 "camera": 10, "payload/gas/probe_result": 1, "payload/site": 1,
                 "payload/guard": 2}
        self.assertEqual(set(peaks), set(rb.INGEST_LIMITS))
        for step in range(600):                                  # 60 s in 0.1 s ticks
            now[0] += 0.1
            for topic, per_s in peaks.items():
                n = per_s // 10 if per_s >= 10 else int(step % (10 // per_s) == 0)
                for _ in range(n):
                    callbacks[topic](None)
        self.assertEqual(state.ingest_counts()["dropped"], {})
        self.assertEqual((seen["odometry"], seen["payload/events"]),
                         (50 * 60, 3 * rb.EVENT_HISTORY_DEPTH // 2 + 10 * 60))

    def test_drops_and_evictions_are_visible_on_the_page_and_in_the_export(self):
        """T42: the snapshot streams the counters, the audit downloads carry
        them as headers, and the page has an INGEST LIMITED badge and keeps
        protected lines when it trims its log."""
        state, advance = _clocked_state()
        _incident(state, advance)
        for i in range(rb.UNVERIFIED_BUFFER + 10):
            state.add_event({"source": "x", "kind": "info", "message": str(i)})
        state.count_dropped("payload/gas")
        self.assertIn("ingest", server.DIFF_KEYS)
        ingest = state.snapshot()["ingest"]
        self.assertEqual(ingest["evicted"], {"unverified": 10})
        self.assertEqual(ingest["flood_total"], 11)
        app = server.create_app(rb.FakeBridge(state=state, realtime=False))
        with _client(app) as client:
            for path in ("/api/audit.csv", "/api/audit.jsonl"):
                resp = client.get(path)
                self.assertEqual((resp.headers["x-audit-dropped"], resp.headers["x-audit-evicted"]),
                                 ("1", "10"), path)
                self.assertIn("per-source buffers", resp.headers["x-audit-scope"])
        with open(os.path.join(server.STATIC_DIR, "app.js"), encoding="utf-8") as handle:
            js = handle.read()
        with open(os.path.join(server.STATIC_DIR, "index.html"), encoding="utf-8") as handle:
            html = handle.read()
        self.assertIn('id="ingest-badge"', html)
        self.assertIn('$("ingest-badge")', js)
        self.assertIn('"INGEST LIMITED · "', js)
        trim = js[js.index("function trimFeed("):]
        trim = trim[:trim.index("\n  }\n")]
        self.assertIn('classList.contains("ev-keep")', trim)
        self.assertIn('ev.hud.record === "safety"', js)



# ---------------------------------------------------------------------------
# T16
# ---------------------------------------------------------------------------


def _beat(state="monitoring", inside=True, seq=1, wall_t=1000.0):
    return json.dumps({"state": state, "robot_inside": inside, "seq": seq, "wall_t": wall_t})


def _guard(state):
    return state.snapshot()["guard"]


class TestT16GuardHeartbeat(unittest.TestCase):
    """T16 (HUD part): consume the agent's payload/guard heartbeat per the
    interface contract and show GUARD DOWN / GUARD NOT RUNNING prominently
    (M16)."""

    def _inside_state(self):
        state, advance = _clocked_state()
        state.ingest_site_json(json.dumps(rb.BUILTIN_SITE))
        return state, advance

    def test_never_seen_is_not_running_and_red_if_spot_is_inside(self):
        """T16: with no heartbeat ever, the guard is NOT RUNNING; the HUD's
        own odometry inside the tank makes it an alarm (fail closed)."""
        state, _advance = self._inside_state()
        self.assertEqual(_guard(state), {"status": "not_running", "alarm": False,
                                         "robot_inside": None, "state": None})
        state.ingest_pose(10.0, 0.0)                              # outside the shell
        self.assertFalse(_guard(state)["alarm"])
        state.ingest_pose(0.0, 0.0)                               # tank centre
        self.assertEqual((_guard(state)["status"], _guard(state)["alarm"]), ("not_running", True))

    def test_guard_down_more_than_3_s_after_the_last_heartbeat_inside(self):
        """T16: Spot last reported inside and no heartbeat for more than 3 s is
        GUARD DOWN; a resumed heartbeat clears it."""
        state, advance = self._inside_state()
        state.ingest_guard_json(_beat(inside=True))
        self.assertEqual(_guard(state), {"status": "monitoring", "alarm": False,
                                         "robot_inside": True, "state": "monitoring"})
        advance(rb.GUARD_TIMEOUT_S)
        self.assertEqual(_guard(state)["status"], "monitoring")   # exactly 3 s: not yet
        advance(0.1)
        self.assertEqual((_guard(state)["status"], _guard(state)["alarm"]), ("down", True))
        state.ingest_guard_json(_beat(inside=True, seq=9))
        self.assertEqual((_guard(state)["status"], _guard(state)["alarm"]), ("monitoring", False))

    def test_stale_heartbeat_with_spot_outside_is_not_running(self):
        """T16: the agent stopped with Spot reported outside: NOT RUNNING, not
        an alarm, unless the HUD's own odometry has Spot inside (then DOWN)."""
        state, advance = self._inside_state()
        state.ingest_pose(10.0, 0.0)
        state.ingest_guard_json(_beat(state="standby", inside=False))
        self.assertEqual(_guard(state)["status"], "standby")
        advance(3.5)
        self.assertEqual((_guard(state)["status"], _guard(state)["alarm"]), ("not_running", False))
        state.ingest_pose(1.0, 0.0)
        self.assertEqual((_guard(state)["status"], _guard(state)["alarm"]), ("down", True))

    def test_a_stalled_guard_is_an_alarm_wherever_spot_is(self):
        """T16: the agent reports "stalled" when its guard thread is gone or its
        gas checks have stopped completing; the HUD shows GUARD STALLED in red,
        inside or outside, and a monitoring beat clears it."""
        state, _advance = self._inside_state()
        for inside in (True, False):
            with self.subTest(inside=inside):
                state.ingest_guard_json(_beat(state="stalled", inside=inside, seq=3))
                self.assertEqual((_guard(state)["status"], _guard(state)["alarm"]), ("stalled", True))
        state.ingest_guard_json(_beat(inside=True, seq=4))
        self.assertEqual((_guard(state)["status"], _guard(state)["alarm"]), ("monitoring", False))
        with open(os.path.join(REPO, "hud", "static", "app.js"), encoding="utf-8") as handle:
            self.assertIn('stalled: ["GUARD STALLED", "guard-bad"]', handle.read())

    def test_standby_while_inside_is_not_armed(self):
        """T16: "monitoring" means armed; a live guard in standby while Spot is
        inside is shown as an alarm."""
        state, _advance = self._inside_state()
        state.ingest_guard_json(_beat(state="standby", inside=True))
        self.assertEqual((_guard(state)["status"], _guard(state)["alarm"]), ("not_armed", True))

    def test_malformed_heartbeats_are_not_proof_of_life(self):
        """T16: a heartbeat that breaks the contract is ignored and counted, so
        it can neither start nor extend "monitoring"."""
        state, advance = self._inside_state()
        bad = ['{"state": "armed", "robot_inside": true, "seq": 1, "wall_t": 1}',
               '{"state": "monitoring", "robot_inside": "yes", "seq": 1, "wall_t": 1}',
               '{"state": "monitoring", "robot_inside": true, "seq": true, "wall_t": 1}',
               '{"state": "monitoring", "robot_inside": true, "seq": -1, "wall_t": 1}',
               '{"state": "monitoring", "robot_inside": true, "seq": 1, "wall_t": "now"}',
               '{"state": "monitoring", "robot_inside": true, "seq": 1, "wall_t": "1000.0"}',
               '{"state": "monitoring", "robot_inside": true, "seq": 1.0, "wall_t": 1}',
               '{"state": "monitoring", "robot_inside": true, "seq": 1, "wall_t": NaN}',
               '{"state": "monitoring", "robot_inside": true, "seq": 1}',
               '["monitoring"]', "{broken", "x" * (rb.MAX_JSON_CHARS + 1)]
        for text in bad:
            state.ingest_guard_json(text)
        self.assertEqual(_guard(state)["status"], "not_running")
        self.assertEqual(state.ingest_counts()["rejected"], {"payload/guard": len(bad)})
        state.ingest_guard_json(_beat())
        advance(3.5)
        for text in bad:
            state.ingest_guard_json(text)
        self.assertEqual(_guard(state)["status"], "down")

    def test_replay_mode_says_the_heartbeat_was_not_recorded(self):
        """T16: the recorded dry runs predate the heartbeat; replay must not
        show them as a guard failure."""
        state = rb.HudState(mode="replay")
        self.assertEqual(_guard(state)["status"], "not_recorded")
        self.assertFalse(_guard(state)["alarm"])

    def test_ros_bridge_subscribes_with_the_contract_qos(self):
        """T16: RosBridge subscribes payload/guard reliable, VOLATILE, depth 10,
        through the ingest limit, and feeds the heartbeat to the state."""
        try:
            from test_hud import _fake_ros_modules
        except ImportError:  # pragma: no cover - depends on how the suite is launched
            from tests.test_hud import _fake_ros_modules
        mods = _fake_ros_modules()
        with mock.patch.dict(sys.modules, mods):
            state = rb.HudState(mode="ros")
            bridge = rb.RosBridge(state, site_defaults_path="/nonexistent.json", site_timeout_s=60)
            bridge.start()
            try:
                node = mods["rclpy"].nodes[-1]
                _msg_type, callback, qos = node.subs["payload/guard"]
                self.assertEqual((qos.reliability.name, qos.durability.name, qos.depth),
                                 ("RELIABLE", "VOLATILE", 10))
                self.assertEqual(set(node.subs) - {bridge.camera_topic},
                                 set(rb.INGEST_LIMITS) - {"camera"})
                callback(types.SimpleNamespace(data=_beat(inside=False, state="standby")))
                self.assertEqual(_guard(state)["status"], "standby")
            finally:
                bridge.stop()

    def test_fake_mode_publishes_the_contract_heartbeat(self):
        """T16: --fake shows a live guard (standby outside, monitoring inside)
        rather than a permanent GUARD NOT RUNNING."""
        bridge = rb.FakeBridge(realtime=False)
        for t in (0.0, 1.0):
            bridge.tick(t)
        self.assertEqual(_guard(bridge.state)["status"], "standby")
        t = 1.0
        while t < 17.0:
            t += 0.25
            bridge.tick(t)
        self.assertEqual(_guard(bridge.state), {"status": "monitoring", "alarm": False,
                                                "robot_inside": True, "state": "monitoring"})

    def test_the_page_shows_guard_down_prominently(self):
        """T16: the header always has a guard pill; DOWN / NOT ARMED / NOT
        RUNNING take the header strip, red with a page outline when Spot is
        inside; "guard" is streamed in every diff."""
        self.assertIn("guard", server.DIFF_KEYS)
        with open(os.path.join(server.STATIC_DIR, "index.html"), encoding="utf-8") as handle:
            html = handle.read()
        with open(os.path.join(server.STATIC_DIR, "app.js"), encoding="utf-8") as handle:
            js = handle.read()
        with open(os.path.join(server.STATIC_DIR, "style.css"), encoding="utf-8") as handle:
            css = handle.read()
        for element_id in ("guard", "guard-strip"):
            self.assertIn('id="%s"' % element_id, html)
            self.assertIn('$("%s")' % element_id, js)
        for text in ('"GUARD DOWN"', '"GUARD NOT RUNNING"', '"GUARD NOT ARMED"',
                     '"no safety-guard heartbeat for over 3 s"', '"Spot is inside the tank"'):
            self.assertIn(text, js)
        self.assertIn('classList.toggle("guard-down", alarm)', js)
        for rule in (".guard-strip-alarm", "body.guard-down .hud", ".guard-bad"):
            self.assertIn(rule, css)



# ---------------------------------------------------------------------------
# T32: camera frames
# ---------------------------------------------------------------------------


def _sim_camera():
    """(width, height) the simulator publishes: sim/sensors.py
    create_spot_cameras' defaults, which sim/isaac_app.py does not override."""
    import inspect
    from sim import sensors
    params = inspect.signature(sensors.create_spot_cameras).parameters
    return params["width"].default, params["height"].default


def _frame(width, height, encoding="rgb8", step=None, data=None):
    """A sensor_msgs/Image stand-in with a zero-filled buffer of the right size."""
    bpp = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}[encoding]
    step = width * bpp if step is None else step
    return types.SimpleNamespace(width=width, height=height, encoding=encoding, step=step,
                                 data=bytes(step * height) if data is None else data)


class TestT9BackendConflictFlashes(unittest.TestCase):
    """T9 (HUD part): the agent's backend_conflict record, which holds forward motion
    and entry because the backend's identity was contradicted, fires the full-screen
    flash, and the page's rule is the bridge's."""

    def test_a_backend_conflict_flashes(self):
        state, _advance = _clocked_state()
        state.ingest_event_json(json.dumps({
            "kind": "backend_conflict", "source": "mission",
            "message": "BACKEND IDENTITY CONFLICT: a second payload/status publisher",
            "data": {"reason": "second status publisher"}}))
        self.assertEqual(state.alert_seq, 1)

    def test_the_page_uses_the_same_rule(self):
        with open(os.path.join(REPO, "hud", "static", "app.js"), encoding="utf-8") as handle:
            page = re.search(r"const ALERT_KIND_RE = /(.*)/;", handle.read()).group(1)
        self.assertEqual(page, rb._ALERT_WORDS.pattern)


class TestT17T12NewMissionRecords(unittest.TestCase):
    """T17 and T12 (HUD part): a guard retreat that fails after its last retry
    (retreat_failed) flashes as a failed retreat and drops the "retreated" note,
    and a purge hold judged on anything but the simulator's current reading is
    labelled an estimate."""

    def test_a_final_retreat_failure_is_a_failed_retreat(self):
        state, advance = _clocked_state()
        state.add_event({"source": "guard", "kind": "retreat_start", "message": "Retreat started",
                         "data": {"auto_retreat": True}})
        advance(1.0)
        state.add_event({"source": "guard", "kind": "retreat_failed",
                         "message": "Retreat failed after 3 attempts; operator attention required",
                         "data": {"auto_retreat": True, "retry": False}})
        alert = state.snapshot()["alert"]
        self.assertEqual(alert["title"], rb.RETREAT_FAILED_TITLE)
        self.assertNotIn(alert["note"], rb._DONE_NOTES)
        self.assertIsNone(rb.HudState._retreat_note({"kind": "retreat_failed", "source": "guard",
                                                      "data": {"auto_retreat": True}}))

    def test_an_estimated_purge_hold_is_labelled(self):
        base = {"purge_target": 0.08, "fraction_remaining": 0.05}
        self.assertFalse(rb.purge_hold_from("purge_hold", dict(base, fraction_source="sim",
                                                                current_feed=True))["estimate"])
        for extra in ({"fraction_source": "estimate"}, {"fraction_source": "last_seen"},
                      {"fraction_source": "sim", "current_feed": False}):
            with self.subTest(extra):
                self.assertTrue(rb.purge_hold_from("purge_hold", dict(base, **extra))["estimate"])
        with open(os.path.join(REPO, "hud", "static", "app.js"), encoding="utf-8") as handle:
            self.assertIn("hold.estimate ? \"Purge hold (ESTIMATE", handle.read())


class TestT32CameraFrameCaps(unittest.TestCase):
    """T32 (M32): any DDS publisher can send the HUD a camera frame, which it
    decodes and re-encodes with Pillow, so frame dimensions and size are
    capped before Pillow sees them and refused frames are counted. The caps
    sit well above the simulator's real camera (640x480 rgb8)."""

    def _bridge(self):
        state = rb.HudState()
        bridge = rb.RosBridge(state)
        return state, bridge, bridge._guard("camera", bridge._on_image)

    def test_the_limits_sit_above_the_simulators_camera(self):
        """T32: the caps leave room for the sim's 640x480 rgb8 frames (and a
        1920x1080 4-byte frame, three times wider and taller)."""
        width, height = _sim_camera()
        self.assertEqual((width, height), (640, 480))
        with open(os.path.join(REPO, "sim", "sensors.py"), encoding="utf-8") as handle:
            self.assertIn("(sensor_msgs/Image, rgb8)", handle.read())
        self.assertGreaterEqual(rb.CAMERA_MAX_WIDTH, 3 * width)
        self.assertGreaterEqual(rb.CAMERA_MAX_HEIGHT, 2 * height)
        self.assertGreaterEqual(rb.CAMERA_MAX_BYTES,
                                rb.CAMERA_MAX_WIDTH * rb.CAMERA_MAX_HEIGHT * 4)
        self.assertGreater(rb.CAMERA_MAX_BYTES, 8 * width * height * 3)

    def test_frames_up_to_the_limits_are_accepted(self):
        """T32: the sim's frame and the largest allowed one reach the encoder
        and come out as a JPEG; nothing is counted as refused."""
        state, bridge, callback = self._bridge()
        frames = (_frame(*_sim_camera()), _frame(*_sim_camera(), "rgb8", step=640 * 3 + 64),
                  _frame(rb.CAMERA_MAX_WIDTH, rb.CAMERA_MAX_HEIGHT, "bgra8"))
        for msg in frames:
            bridge._cam_pending = None
            callback(msg)
            self.assertIs(bridge._cam_pending, msg)
            jpeg, out_w, _out_h = rb.encode_ros_image(msg.data, msg.width, msg.height,
                                                      msg.encoding, msg.step, 800)
            self.assertEqual((jpeg[:2], out_w), (b"\xff\xd8", min(800, msg.width)))
        self.assertEqual(state.ingest_counts()["rejected"], {})

    def test_oversize_frames_are_refused_before_decoding_and_counted(self):
        """T32: too wide, too tall, dimensions with no data behind them, rows
        padded past the byte cap, or a buffer over it: refused in the ROS
        callback (never queued, never decoded) and counted, so the INGEST
        LIMITED badge and the export headers show them."""
        top_w, top_h = rb.CAMERA_MAX_WIDTH, rb.CAMERA_MAX_HEIGHT
        bad = (
            _frame(top_w + 1, 1),
            _frame(1, top_h + 1),
            _frame(65535, 65535, data=b""),
            _frame(top_w, top_h, "bgra8", step=top_w * 4 + 100),
            _frame(16, 16, data=bytes(rb.CAMERA_MAX_BYTES + 1)),
            _frame(16, 16, data=bytes(10)),                      # truncated
            _frame(16, 16, "rgb8", step=8),                      # step < row
            types.SimpleNamespace(width=16, height=16, encoding="32FC1", step=64,
                                  data=bytes(1024)),
        )
        state, bridge, callback = self._bridge()
        with mock.patch.object(rb.Image, "frombytes", side_effect=AssertionError("decoded")):
            for msg in bad:
                callback(msg)
                self.assertIsNone(bridge._cam_pending, (msg.width, msg.height, msg.step))
                with self.assertRaises(ValueError):
                    rb.encode_ros_image(msg.data, msg.width, msg.height, msg.encoding, msg.step)
        self.assertEqual(state.ingest_counts()["rejected"], {"camera": len(bad)})
        self.assertGreaterEqual(state.snapshot()["ingest"]["flood_total"], len(bad))


# ---------------------------------------------------------------------------
# Content-scanner findings (threat model 4.2, release blockers)
# ---------------------------------------------------------------------------


def _load(*parts):
    """Import a repository script by path (tools/ is not a package)."""
    path = os.path.join(REPO, *parts)
    spec = importlib.util.spec_from_file_location("_hardening_" + parts[-1][:-3], path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, path


def _calls(path, names):
    """Every call in ``path`` whose function's last attribute is in ``names``."""
    with open(path, encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), filename=path)
    return [node for node in ast.walk(tree) if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute) and node.func.attr in names]


class TestScannerFindingsResolved(unittest.TestCase):
    """Content scanner: Bandit B310 (urlopen) in hud/replay_screens.py
    and tools/hud_watch.py and Semgrep insecure-websocket in hud/static/app.js,
    restricted and justified inline; Semgrep dangerous-subprocess in
    hud/replay_screens.py, removed by naming both programs in the source."""

    GOOD = ("http://127.0.0.1:8080/api/state", "https://127.0.0.1/", "http://127.0.0.1:1/x?y=1")
    BAD = ("file:///etc/passwd", "ftp://127.0.0.1/", "http://localhost:8080/api/state",
           "http://127.0.0.1.attacker.example/", "http://attacker.example/?h=127.0.0.1",
           "http://127.0.0.1@attacker.example/", "jar:http://127.0.0.1/!/", "127.0.0.1:8080")

    def _check_loopback_only(self, module, path):
        for url in self.GOOD:
            self.assertEqual(module.loopback_url(url), url)
        for url in self.BAD:
            with self.assertRaises(ValueError, msg=url):
                module.loopback_url(url)
        with open(path, encoding="utf-8") as handle:
            source = handle.read()
        for call in _calls(path, {"urlopen"}):
            line = source.splitlines()[call.lineno - 1]
            self.assertIn("# nosec B310", line)                  # justified inline...
            arg = ast.unparse(call.args[0])                     # ...after loopback_url()
            self.assertTrue(arg.startswith("loopback_url(") or "%s = loopback_url(" % arg in source,
                            arg)

    def test_replay_screens_opens_only_loopback_urls(self):
        """Bandit B310 at hud/replay_screens.py:58: urlopen is reached only
        through loopback_url (http(s)://127.0.0.1), with an inline nosec."""
        self._check_loopback_only(*_load("hud", "replay_screens.py"))

    def test_hud_watch_opens_only_loopback_urls_and_keeps_its_moments(self):
        """Bandit B310 at tools/hud_watch.py:40: same restriction; importing
        the tool no longer runs it, and the storyline triggers are unchanged."""
        module, path = _load("tools", "hud_watch.py")
        self._check_loopback_only(module, path)
        self.assertIn("error", module.fetch_state("file:///etc/passwd"))
        records = [{"kind": "atmos_result", "data": {"passed": False}},
                   {"kind": "ventilation", "message": "Ventilating: 40 %"},
                   {"kind": "permit", "permit": "ENTRY_AUTHORIZED"},
                   {"kind": "alarm"}, {"kind": "alarm"}, {"kind": "retreat_incomplete"}]
        done, moments = set(), []
        for rec in records:
            name = module.moment_for(rec, done)
            if name:
                done.add(name)
                moments.append(name)
        self.assertEqual(moments, list(module.MOMENTS))

    def test_replay_screens_names_both_programs_in_the_source(self):
        """Semgrep dangerous-subprocess in hud/replay_screens.py: the tool's two
        subprocess calls name their program in the source, "python3" running
        hud/server.py and Chrome at CHROME, each with an argument list and no
        shell. No option chooses another program: --chrome is gone, and main()
        starts nothing when CHROME is missing or python3 is another interpreter."""
        module, path = _load("hud", "replay_screens.py")
        calls = sorted(_calls(path, {"Popen", "run", "call", "check_call", "check_output"}),
                       key=lambda call: call.lineno)
        self.assertEqual(len(calls), 2)
        server_call, chrome_call = calls
        self.assertEqual([ast.literal_eval(arg) for arg in server_call.args[0].elts[:2]],
                         ["python3", "hud/server.py"])
        self.assertEqual(ast.unparse(chrome_call.args[0].elts[0]), "CHROME")
        self.assertEqual(module.CHROME, "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome")
        for call in calls:
            self.assertNotIn("shell", [kw.arg for kw in call.keywords])
            self.assertNotIn(None, [kw.arg for kw in call.keywords])   # no **kwargs
        with mock.patch.object(module.subprocess, "Popen") as popen:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                module.main(["--chrome", "/bin/sh"])
            with mock.patch.object(module, "CHROME", os.path.join(REPO, "README.md")), \
                    contextlib.redirect_stderr(io.StringIO()) as err:
                self.assertEqual(module.main([]), 2)
            self.assertIn("Google Chrome", err.getvalue())
            with mock.patch.object(module, "executable_path"), \
                    mock.patch.object(module, "python_env", side_effect=RuntimeError("other python3")), \
                    contextlib.redirect_stderr(io.StringIO()) as err:
                self.assertEqual(module.main([]), 2)
            self.assertIn("other python3", err.getvalue())
        popen.assert_not_called()

    def test_replay_screens_runs_the_server_on_this_python(self):
        """hud/replay_screens.py: the literal "python3" is this interpreter.
        python_env() puts its directory first on PATH, adds no empty entry (which
        would search the current directory), and refuses when the python3 found
        there is another program or none."""
        module, _path = _load("hud", "replay_screens.py")
        here = os.path.dirname(sys.executable)
        found = shutil.which("python3", path=here)
        if not found or os.path.realpath(found) != os.path.realpath(sys.executable):
            self.skipTest("no python3 next to this interpreter")
        with mock.patch.dict(os.environ, {"PATH": ""}):
            self.assertEqual(module.python_env()["PATH"], here)
        with mock.patch.dict(os.environ, {"PATH": "/usr/bin"}):
            self.assertEqual(module.python_env()["PATH"].split(os.pathsep), [here, "/usr/bin"])
        for other in ("/usr/bin/false", None):
            with self.subTest(found=other), mock.patch.object(module.shutil, "which", return_value=other):
                with self.assertRaises(RuntimeError):
                    module.python_env()

    def test_replay_screens_waits_for_chrome_like_subprocess_run(self):
        """hud/replay_screens.py: the screenshot is checked and bounded as
        subprocess.run(check=True, timeout=60) would: a failure raises, a hang
        is killed and reaped."""
        module, _path = _load("hud", "replay_screens.py")
        try:
            env = module.python_env()
        except RuntimeError as err:
            self.skipTest(str(err))
        self.assertIsNone(module.wait_ok(subprocess.Popen(["python3", "-c", "pass"], env=env), timeout=30))
        with self.assertRaises(subprocess.CalledProcessError) as failed:
            module.wait_ok(subprocess.Popen(["python3", "-c", "raise SystemExit(3)"], env=env), timeout=30)
        self.assertEqual(failed.exception.returncode, 3)
        hung = subprocess.Popen(["python3", "-c", "import time; time.sleep(60)"], env=env)
        with self.assertRaises(subprocess.TimeoutExpired):
            module.wait_ok(hung, timeout=0.2)
        self.assertIsNotNone(hung.returncode)   # killed and reaped, not left running

    def test_page_websocket_uses_wss_on_an_https_page(self):
        """Semgrep insecure-websocket at hud/static/app.js:1224: the socket URL
        is the page's own origin with wss: on https:, justified inline (T2
        still refuses any other Origin)."""
        with open(os.path.join(server.STATIC_DIR, "app.js"), encoding="utf-8") as handle:
            js = handle.read()
        self.assertIn('url.protocol = location.protocol === "https:" ? "wss:" : "ws:";', js)
        self.assertIn('new URL("/ws", location.href)', js)
        # Asserts the plain-ws literal is gone, so the scanner's own pattern appears here.
        # nosemgrep: javascript.lang.security.detect-insecure-websocket.detect-insecure-websocket
        self.assertNotIn('"ws://"', js)
        self.assertIn("Scanner note: reviewed", js)


if __name__ == "__main__":
    unittest.main()
