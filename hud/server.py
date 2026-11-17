"""Presenter HUD web server (FastAPI + uvicorn) for the Spot confined-space demo.

What it is
----------
Serves the single-page dashboard (hud/static/, vanilla JS/CSS, no CDN, no
build step) and streams HUD state from hud/ros_bridge.py:

    GET  /                 index.html
    GET  /static/*         app.js, style.css
    GET  /api/state        full JSON snapshot (the page's log: every alarm,
                           permit and retreat record + the newest 200 others;
                           the trail; ingest counters; undisclosed
                           simulator changes, M23)
    GET  /api/camera.jpg   latest front-left camera frame (JPEG, <= 5 fps);
                           503 until the first frame arrives
    GET  /api/audit.jsonl  every buffered event (per-source buffers, as
                           received, each with its 'hud' receipt stamp) as a
                           JSONL download; X-Audit-* headers give the scope
                           and the dropped / rejected / evicted counts
    GET  /api/audit.csv    the same, flattened (t, source, source check,
                           kind, message, permit state, retreat, gas values
                           + provenance); hud/audit.py
    WS   /ws               {"type":"hello","state":{...}} on connect, then
                           {"type":"events","events":[...]} immediately per event,
                           {"type":"diff","changes":{...}} at ~4 Hz (changed
                           top-level keys only), {"type":"ping"} when idle

How it runs
-----------
    python3 hud/server.py --fake                  # Mac: synthetic data, no ROS
    python3 hud/server.py --replay docs/evidence/events-ok1.jsonl \
        --replay docs/evidence/hud-audit-ok1.csv --replay-at 450
                                                  # Mac: a RECORDED dry run, held at sim t=450
    python3 hud/server.py                         # HUD container: live rclpy bridge
    python3 hud/server.py --host 127.0.0.1 --port 8080 --namespace /demo_spot

Binds 127.0.0.1 by default: on the sim host (--network=host) the SSM
port-forwarding session connects to the instance's loopback, so nothing
needs to listen on an external interface (the security group has no
ingress either). See hud/README.md and scripts/connect-hud.sh.

Request validation (T2; the HUD has no login by design, so it relies on
these and on the loopback bind):
  * Host must be exactly localhost:<port> or 127.0.0.1:<port>
    (TrustedHostMiddleware for the name, _PortGuard for the port). A DNS
    rebinding page sends its own name as Host and gets 400. The browser's
    port must equal --port, so the SSM tunnel's local port must too;
  * /ws needs an Origin of exactly http://localhost:<port> or
    http://127.0.0.1:<port>; a missing or foreign Origin is refused, which
    stops cross-site WebSocket hijacking;
  * at most MAX_WS_CLIENTS WebSockets at once (the page falls back to
    polling), uvicorn's limit_concurrency, and a small ws_max_size;
  * a --host that is not loopback is refused unless --allow-non-loopback
    is given, and then logged as a warning.

Robustness choices (this runs live in front of executives):
  * every WebSocket send has a timeout; a stalled or vanished client is
    dropped instead of blocking the loop, and a reader task notices closes
    promptly;
  * the server only reads robot state -- it never commands the robot;
  * the client reconnects with backoff and falls back to polling
    /api/state if the WebSocket cannot be established.
"""

import argparse
import asyncio
import ipaddress
import json
import logging
import os
import sys
from contextlib import asynccontextmanager

_HUD_DIR = os.path.dirname(os.path.abspath(__file__))
if _HUD_DIR not in sys.path:
    sys.path.insert(0, _HUD_DIR)

import ros_bridge  # noqa: E402  (sibling module; works as a script and via "hud.server")
import audit  # noqa: E402

from fastapi import FastAPI, WebSocket  # noqa: E402
from fastapi.responses import (FileResponse, JSONResponse, PlainTextResponse,  # noqa: E402
                               Response, StreamingResponse)
from fastapi.staticfiles import StaticFiles  # noqa: E402
from starlette.datastructures import Headers  # noqa: E402
from starlette.middleware.trustedhost import TrustedHostMiddleware  # noqa: E402

log = logging.getLogger("hud.server")

STATIC_DIR = os.path.join(_HUD_DIR, "static")
DIFF_PERIOD_S = 0.25     # ~4 Hz state diffs (task contract)
PING_AFTER_S = 2.0       # idle keepalive so the client watchdog can detect a dead link
SEND_TIMEOUT_S = 2.0     # a client that can't take a frame in 2 s is dropped

# T2 request validation and caps (see the module docstring).
DEFAULT_PORT = 8080
LOOPBACK_NAMES = ("localhost", "127.0.0.1")
MAX_WS_CLIENTS = 8       # presenter screen, a spare screen, reloads; beyond that is a flood
LIMIT_CONCURRENCY = 32   # uvicorn: open connections + tasks; beyond it new ones get 503
WS_MAX_SIZE = 4096       # bytes; the page never sends data frames (uvicorn's default is 16 MiB)

# Keys streamed as diffs; events/trail are streamed incrementally instead.
DIFF_KEYS = ("server", "permit", "status", "gas", "ventilation", "probe", "purge_hold", "pose",
             "site", "camera", "link", "alert", "thresholds", "events_seq", "trail_epoch",
             "ingest", "guard", "world")
AUDIT_SCOPE = ("hud-event-buffers: payload/events seen by this HUD since it started, in "
               "per-source buffers (up to %d safety and %d other records per source, the "
               "simulator's refusals apart, %d from "
               "unverified sources, %d of each of the HUD's own two); the mission's full JSONL "
               "audit log is "
               "on the sim host (scripts/fetch-log.sh)"
               % (ros_bridge.SAFETY_BUFFER, ros_bridge.ROUTINE_BUFFER,
                  ros_bridge.UNVERIFIED_BUFFER, ros_bridge.LOCAL_BUFFER))

# Everything is same-origin; no CDN. 'ws:'/'wss:' because older Safari does
# not treat same-host WebSockets as 'self'.
_CSP = ("default-src 'self'; script-src 'self'; style-src 'self'; "
        "img-src 'self' blob: data:; connect-src 'self' ws: wss:; "
        "frame-ancestors 'none'; base-uri 'none'; form-action 'none'")


def allowed_hosts(port):
    """The exact Host header values the HUD answers: the loopback names at
    ``port``. Browsers leave out :80, so port 80 also allows the bare names."""
    hosts = {"%s:%d" % (name, port) for name in LOOPBACK_NAMES}
    if port == 80:
        hosts.update(LOOPBACK_NAMES)
    return frozenset(hosts)


def allowed_origins(port):
    """The exact Origin values /ws accepts: the page's own loopback origins."""
    return frozenset("http://" + host for host in allowed_hosts(port))


_refusals = ros_bridge._RateLimitedLog(logger=log)   # one line per reason per 10 s


def _origin_allowed(websocket, origins):
    """Reject cross-site WebSocket hijacking (T2): the Origin must be one of
    ``origins`` exactly. The page always sends one, so a missing Origin
    (a non-browser client) is refused too."""
    origin = websocket.headers.get("origin")
    return bool(origin) and origin.lower() in origins


def _is_loopback(host):
    """True for localhost and loopback IP literals (127.0.0.0/8, ::1)."""
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host.strip("[]")).is_loopback
    except ValueError:
        return False


class _PortGuard:
    """Pure ASGI: refuse a request whose Host header names another port (T2).

    TrustedHostMiddleware, which runs inside this, checks the host name but
    ignores the port. Through the SSM tunnel the browser's Host is
    localhost:<local port>, and /ws only accepts that origin when the local
    port equals --port, so a mismatch fails here with a message that says
    so, not as a page that loads and then never connects.
    """

    def __init__(self, app, port):
        self.app = app
        self.port = port
        self.message = ("This HUD serves port %d: open http://localhost:%d/ (the SSM tunnel's "
                        "local port must equal the HUD's --port)." % (port, port))

    async def __call__(self, scope, receive, send):
        if scope["type"] in ("http", "websocket"):
            host = Headers(scope=scope).get("host", "")
            _name, sep, port = host.rpartition(":")
            if not sep or "]" in port:   # no port given: http's default
                port = "80"
            if port != str(self.port):
                await PlainTextResponse(self.message, status_code=400)(scope, receive, send)
                return
        await self.app(scope, receive, send)


async def _send(websocket, message):
    # allow_nan=False: HudState already sanitizes; fail loudly here rather
    # than emit JSON the browser cannot parse.
    text = json.dumps(message, allow_nan=False, separators=(",", ":"))
    await asyncio.wait_for(websocket.send_text(text), timeout=SEND_TIMEOUT_S)


async def _drain(websocket):
    """Consume client frames so a close is noticed immediately."""
    while True:
        message = await websocket.receive()
        if message.get("type") == "websocket.disconnect":
            return


def create_app(bridge, port=DEFAULT_PORT, max_ws_clients=MAX_WS_CLIENTS):
    """Build the FastAPI app around a started-on-startup bridge.

    ``bridge`` is a ros_bridge.RosBridge or ros_bridge.FakeBridge (anything
    with ``.state``, ``start()``, ``stop()``). ``port`` is the one the
    browser uses (--port): the Host and Origin allowlists follow it.
    """
    state = bridge.state
    origins = allowed_origins(port)
    ws_clients = [0]   # open /ws connections; only touched on the event loop

    @asynccontextmanager
    async def lifespan(_app):
        bridge.start()
        try:
            yield
        finally:
            bridge.stop()
            log.info("bridge stopped")

    app = FastAPI(title="Spot demo presenter HUD", lifespan=lifespan,
                  docs_url=None, redoc_url=None, openapi_url=None)
    app.state.bridge = bridge

    # Added innermost first: security_headers (below) wraps both, so a
    # refused request still carries the CSP and nosniff headers.
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=list(LOOPBACK_NAMES),
                       www_redirect=False)
    app.add_middleware(_PortGuard, port=port)

    @app.middleware("http")  # nosemgrep: useless-inner-function
    async def security_headers(request, call_next):
        response = await call_next(request)
        response.headers.setdefault("Content-Security-Policy", _CSP)
        response.headers.setdefault("X-Content-Type-Options", "nosniff")
        response.headers.setdefault("Referrer-Policy", "no-referrer")
        return response

    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

    @app.get("/", include_in_schema=False)  # nosemgrep: useless-inner-function
    async def index():
        # no-cache: a redeployed frontend must load on the next refresh.
        return FileResponse(os.path.join(STATIC_DIR, "index.html"),
                            headers={"Cache-Control": "no-cache"})

    @app.get("/api/state")  # nosemgrep: useless-inner-function
    async def api_state():
        return JSONResponse(state.snapshot(include_events=True, include_trail=True),
                            headers={"Cache-Control": "no-store"})

    @app.get("/api/camera.jpg")  # nosemgrep: useless-inner-function
    async def api_camera():
        jpeg, seq = state.camera_jpeg()
        if jpeg is None:
            return JSONResponse({"detail": "no camera frame received yet"}, status_code=503,
                                headers={"Cache-Control": "no-store", "Retry-After": "1"})
        return Response(content=jpeg, media_type="image/jpeg",
                        headers={"Cache-Control": "no-store", "X-Frame-Seq": str(seq)})

    def _audit_headers(ext, count):
        ingest = state.ingest_counts()   # T42: what this export may be missing, and why
        return {"Cache-Control": "no-store",
                "Content-Disposition": 'attachment; filename="%s"' % audit.filename(ext),
                "X-Audit-Events": str(count), "X-Audit-Scope": AUDIT_SCOPE,
                "X-Audit-Dropped": str(sum(ingest["dropped"].values())),
                "X-Audit-Rejected": str(sum(ingest["rejected"].values())),
                "X-Audit-Evicted": str(sum(ingest["evicted"].values()))}

    @app.get("/api/audit.jsonl")  # nosemgrep: useless-inner-function
    async def api_audit_jsonl():
        events = state.events()   # copied under the lock: the stream is consistent
        return StreamingResponse(audit.jsonl_lines(events), media_type="application/x-ndjson",
                                 headers=_audit_headers("jsonl", len(events)))

    @app.get("/api/audit.csv")  # nosemgrep: useless-inner-function
    async def api_audit_csv():
        events = state.events()
        return Response(content=audit.csv_text(events), media_type="text/csv; charset=utf-8",
                        headers=_audit_headers("csv", len(events)))

    @app.websocket("/ws")  # nosemgrep: useless-inner-function
    async def ws_endpoint(websocket: WebSocket):
        if not _origin_allowed(websocket, origins):
            _refusals.warning("origin", "ws: refused Origin %r (allowed: %s)",
                              (websocket.headers.get("origin") or "")[:100], sorted(origins))
            await websocket.close(code=1008)
            return
        if ws_clients[0] >= max_ws_clients:
            _refusals.warning("cap", "ws: refused a client: %d already connected (cap); "
                              "the page falls back to polling", ws_clients[0])
            await websocket.close(code=1013)
            return
        ws_clients[0] += 1   # no await since the check: nothing can interleave
        try:
            await _serve_ws(websocket)
        finally:
            ws_clients[0] -= 1

    async def _serve_ws(websocket):  # nosemgrep: useless-inner-function
        await websocket.accept()
        peer = "%s:%s" % (websocket.client.host, websocket.client.port) if websocket.client else "?"
        loop = asyncio.get_running_loop()
        wake = asyncio.Event()

        def on_new_data():  # called from producer threads
            loop.call_soon_threadsafe(wake.set)

        # Register BEFORE the snapshot so nothing falls between the two.
        token = state.add_listener(on_new_data)
        reader = asyncio.create_task(_drain(websocket))
        log.info("ws: client %s connected", peer)
        try:
            snap = state.snapshot(include_events=True, include_trail=True)
            last_seq = snap["events_seq"]
            last_alert = snap["alert"]["seq"]
            last_sent = {k: snap[k] for k in DIFF_KEYS}
            await _send(websocket, {"type": "hello", "state": snap})
            last_tx = next_diff = loop.time()
            next_diff += DIFF_PERIOD_S
            while not reader.done():
                try:
                    await asyncio.wait_for(wake.wait(), max(0.0, next_diff - loop.time()))
                except asyncio.TimeoutError:
                    pass
                wake.clear()
                events = state.events_since(last_seq)
                if events:
                    last_seq = events[-1]["seq"]
                    await _send(websocket, {"type": "events", "events": events})
                    last_tx = loop.time()
                alert_changed = state.alert_seq != last_alert
                if loop.time() >= next_diff or alert_changed:
                    next_diff = loop.time() + DIFF_PERIOD_S
                    snap = state.snapshot()
                    last_alert = snap["alert"]["seq"]
                    changes = {k: snap[k] for k in DIFF_KEYS if last_sent.get(k) != snap[k]}
                    if changes:
                        last_sent.update(changes)
                        await _send(websocket, {"type": "diff", "changes": changes})
                        last_tx = loop.time()
                    elif loop.time() - last_tx >= PING_AFTER_S:
                        await _send(websocket, {"type": "ping"})
                        last_tx = loop.time()
        except asyncio.TimeoutError:
            log.warning("ws: client %s too slow (send > %.0f s); dropping", peer, SEND_TIMEOUT_S)
        except Exception as error:  # disconnect mid-send, closed transport, ...
            log.info("ws: client %s gone (%s)", peer, type(error).__name__)
        finally:
            state.remove_listener(token)
            reader.cancel()
            if reader.done() and not reader.cancelled():
                reader.exception()  # consume it: avoids "exception never retrieved" noise
            try:
                await websocket.close()
            except Exception:
                pass
            log.info("ws: client %s disconnected", peer)

    return app


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Spot demo presenter HUD (FastAPI + uvicorn)")
    parser.add_argument("--fake", action="store_true",
                        help="Synthetic data (no ROS): moving pose, gas, alarm after 20 s, events")
    parser.add_argument("--fake-speed", type=float, default=1.0,
                        help="Fake scenario time multiplier (default 1.0)")
    parser.add_argument("--replay", action="append", metavar="FILE", default=None,
                        help="Replay a RECORDED dry run instead of ROS (no simulator): a mission "
                             "audit log (.jsonl, first) plus optionally that session's HUD audit "
                             "CSV (agent transcript + gas samples). Repeatable.")
    parser.add_argument("--replay-at", type=float, default=None, metavar="SIM_T",
                        help="Jump to this recorded sim time (s, as shown T+mm:ss) and HOLD there")
    parser.add_argument("--replay-from", type=float, default=None, metavar="SIM_T",
                        help="Jump to this recorded sim time, then play on in real time")
    parser.add_argument("--replay-speed", type=float, default=1.0,
                        help="Replay time multiplier (default 1.0)")
    parser.add_argument("--replay-speech-filter", action="store_true",
                        help="Pass replayed agent 'say' lines through the CURRENT agent speech "
                             "post-filter (agent/spot_agent.sanitize_speech), i.e. show what "
                             "today's agent would publish for the recorded replies")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind address (default 127.0.0.1; the SSM tunnel targets loopback). "
                             "A non-loopback address is refused without --allow-non-loopback")
    parser.add_argument("--allow-non-loopback", action="store_true",
                        help="Allow a non-loopback --host (logged as a warning). The HUD has no "
                             "login and still answers only Host localhost/127.0.0.1:<port>")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="Listen port; also the only port the Host and Origin checks accept "
                             "(default %d)" % DEFAULT_PORT)
    parser.add_argument("--namespace", default=ros_bridge.DEFAULT_NAMESPACE)
    parser.add_argument("--camera-topic", default=ros_bridge.DEFAULT_CAMERA_TOPIC,
                        help="Relative to --namespace (default camera/frontleft/image)")
    parser.add_argument("--camera-fps", type=float, default=5.0, help="JPEG rate cap (<= 5)")
    parser.add_argument("--camera-width", type=int, default=800,
                        help="Downsample frames wider than this before JPEG encoding")
    parser.add_argument("--site-defaults", default=None,
                        help="Site fallback JSON (default agent/site_defaults.json)")
    parser.add_argument("--site-timeout", type=float, default=10.0,
                        help="Seconds to wait for latched payload/site before falling back")
    parser.add_argument("--log-level", default="info",
                        choices=("critical", "error", "warning", "info", "debug"))
    return parser.parse_args(argv)


def check_bind(host, port, allow_non_loopback):
    """None if ``host``/``port`` may be bound, else the reason (T2)."""
    if not 0 < port < 65536:
        return "--port %d is not a TCP port" % port
    if _is_loopback(host):
        return None
    if not allow_non_loopback:
        return ("refusing --host %s: not a loopback address. The HUD has no login; anyone who "
                "can reach it reads operator prompts, the transcript, permit data and camera "
                "frames. Pass --allow-non-loopback to bind it anyway" % host)
    log.warning("binding non-loopback --host %s (--allow-non-loopback): the HUD has no login, "
                "and it answers only Host localhost:%d / 127.0.0.1:%d, so use it only behind a "
                "port forward or proxy that delivers loopback traffic", host, port, port)
    return None


def main(argv=None):
    args = parse_args(argv)
    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    refused = check_bind(args.host, args.port, args.allow_non_loopback)
    if refused:
        log.critical("%s", refused)
        return 2
    if args.replay:
        say_filter = None
        if args.replay_speech_filter:
            sys.path.insert(0, os.path.dirname(_HUD_DIR))
            from agent.spot_agent import sanitize_speech   # Mac dev tool only
            say_filter = lambda text: sanitize_speech(text)[0]  # noqa: E731
        start = args.replay_at if args.replay_at is not None else args.replay_from
        try:
            bridge = ros_bridge.ReplayBridge(
                args.replay, realtime=True, speed=args.replay_speed, start_at=start,
                hold=args.replay_at is not None, say_filter=say_filter)
        except (OSError, ValueError) as error:
            log.critical("cannot replay %s: %s", args.replay, error)
            return 2
    elif args.fake:
        state = ros_bridge.HudState(mode="fake")
        bridge = ros_bridge.FakeBridge(state, realtime=True, speed=args.fake_speed)
    else:
        try:
            ros_bridge.check_ros_available()
        except ros_bridge.BridgeUnavailable as error:
            log.critical("%s", error)
            return 2
        state = ros_bridge.HudState(mode="ros")
        bridge = ros_bridge.RosBridge(
            state, namespace=args.namespace, camera_topic=args.camera_topic,
            camera_fps=args.camera_fps, camera_max_width=args.camera_width,
            site_defaults_path=args.site_defaults, site_timeout_s=args.site_timeout)

    import uvicorn  # deferred: tests import this module without needing a server

    log.info("HUD on http://%s:%d (%s mode)", args.host, args.port,
             "REPLAY" if args.replay else "FAKE/synthetic" if args.fake else "ROS")
    uvicorn.run(create_app(bridge, port=args.port), host=args.host, port=args.port,
                log_level=args.log_level, access_log=False,  # camera polling would flood it
                limit_concurrency=LIMIT_CONCURRENCY, ws_max_size=WS_MAX_SIZE,
                ws_ping_interval=10.0, ws_ping_timeout=10.0, timeout_graceful_shutdown=3)
    return 0


if __name__ == "__main__":
    sys.exit(main())
