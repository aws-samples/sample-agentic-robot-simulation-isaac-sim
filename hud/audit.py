"""Audit export of the HUD's event ring buffer (JSONL + flattened CSV).

What it is
----------
Pure formatting helpers behind ``GET /api/audit.jsonl`` and
``GET /api/audit.csv`` (hud/server.py). Input is ``HudState.events()``: the
payload/events records in the HUD's per-source buffers (T42; see
ros_bridge.SAFETY_BUFFER), plus the HUD's own records (payload/gas alarm
edges, and ``undisclosed_change`` flags for simulator atmosphere changes no
disclosure accompanied, M23), as received, each with ``seq``, ``rx_wall``
and a ``hud`` stamp (permit state after the event, source check, record
class, and the latest payload/gas sample if it was fresh -- see
HudState._receipt_stamp).

Scope, stated honestly in the download: this is what THIS HUD saw since it
started (the X-Audit-* headers give the buffer sizes and how many messages
were dropped, rejected or evicted). The authoritative audit trail is the
mission's append-only JSONL on the sim host (/opt/spot-demo/logs/*.jsonl),
pulled to the Mac with scripts/fetch-log.sh.

CSV columns
-----------
seq, t, wall_t, wall_utc, rx_wall_utc, source, source_check, kind, message,
permit_state, retreat, o2_pct, lel_pct, h2s_ppm, co_ppm, gas_values_from,
gas_alarm, data_json

``t`` is the publisher's timestamp: sim time for mission/agent/guard events
while the facade reports it (agent/robot_client.py), otherwise wall time.
``wall_t`` is the publisher's unix wall time (s) and ``wall_utc`` the same
as UTC ISO 8601; both are empty for publishers that do not send ``wall_t``.
``rx_wall_utc`` is when THIS HUD received the event.

Provenance (T3): nothing on payload/events is authenticated, so ``source``
is what the publisher declared. ``source_check`` is ``allowlisted`` when it
is one of the sources this sample's publishers use
(ros_bridge.EVENT_SOURCES, the simulator facade's records included; still
not proof), ``unverified`` otherwise (such an event changed no HUD state),
and ``local`` for records the HUD wrote itself. ``retreat`` is the HUD's
own reading of a retreat record: ``automatic`` (the guard's cause or
auto_retreat flag) or ``commanded``, never taken from the source field.
Both are empty in exports made before these columns existed.

``gas_values_from`` says where the four gas columns came from: ``event``
(values the event itself carries: a monitor reading, the reading that
triggered an alarm/retreat, or an atmospheric-test entry), ``hud_latest``
(the latest payload/gas sample when the HUD received the event), or empty.
``gas_alarm`` is the HUD's OSHA App. E check of event-carried values, or the
payload/gas alarm flag for ``hud_latest`` values.
Text cells that a spreadsheet would treat as a formula (= + - @ tab CR) are
prefixed with a single quote: messages include LLM output.
"""

import csv
import datetime
import io
import json

import ros_bridge

CSV_COLUMNS = ("seq", "t", "wall_t", "wall_utc", "rx_wall_utc", "source", "source_check",
               "kind", "message", "permit_state", "retreat",
               "o2_pct", "lel_pct", "h2s_ppm", "co_ppm", "gas_values_from", "gas_alarm",
               "data_json")
GAS_COLUMNS = ("o2_pct", "lel_pct", "h2s_ppm", "co_ppm")
_FORMULA_PREFIXES = ("=", "+", "-", "@", "\t", "\r")


def _utc(epoch):
    """Unix seconds -> UTC ISO 8601, or "" when missing or unrepresentable.

    ``wall_t`` comes from the publisher (payload/events is untrusted): a
    value outside datetime's range must blank this one cell, not make
    fromtimestamp raise and fail the whole CSV export."""
    if epoch is None:
        return ""
    try:
        return datetime.datetime.fromtimestamp(float(epoch), datetime.timezone.utc).isoformat(
            timespec="milliseconds").replace("+00:00", "Z")
    except (OverflowError, OSError, ValueError, TypeError):
        return ""


_SOURCE_CHECKS = (ros_bridge.SOURCE_ALLOWLISTED, ros_bridge.SOURCE_UNVERIFIED, ros_bridge.SOURCE_LOCAL)


def _enum_cell(value, allowed):
    """``value`` if it is one of ``allowed`` (the HUD's own stamp), else ""."""
    return value if value in allowed else ""


def safe_cell(text):
    """Neutralize spreadsheet formula injection in a text cell."""
    text = "" if text is None else str(text)
    return "'" + text if text.startswith(_FORMULA_PREFIXES) else text


def jsonl_lines(events):
    """One compact JSON line (with trailing newline) per event, oldest first."""
    for ev in events:
        yield json.dumps(ev, allow_nan=False, separators=(",", ":"), sort_keys=True) + "\n"


def _alarm_cell(origin, values, hud):
    """'true'/'false' for the row's gas values: the HUD's OSHA App. E check
    for event-carried values, the payload/gas alarm flag for hud_latest."""
    if origin == "event":
        over = any(ros_bridge.gas_exceeds(gas, values[spec["key"]])
                   for gas, spec in ros_bridge.OSHA_THRESHOLDS.items() if spec["key"] in values)
        return "true" if over else "false"
    if origin == "hud_latest" and hud.get("gas_alarm") is not None:
        return "true" if hud["gas_alarm"] else "false"
    return ""


def flatten(ev):
    """One CSV row dict for an event (see module docstring)."""
    data = ev.get("data") if isinstance(ev.get("data"), dict) else None
    hud = ev.get("hud") if isinstance(ev.get("hud"), dict) else {}
    values = ros_bridge.event_gas_values(data) if data else {}
    origin = "event" if values else ""
    if not values and isinstance(hud.get("gas"), dict):
        values, origin = hud["gas"], "hud_latest"
    row = {
        "seq": ev.get("seq", ""),
        "t": "" if ev.get("t") is None else ev["t"],
        "wall_t": "" if ev.get("wall_t") is None else ev["wall_t"],
        "wall_utc": _utc(ev.get("wall_t")),
        "rx_wall_utc": _utc(ev.get("rx_wall")),
        "source": safe_cell(ev.get("source")),
        "source_check": _enum_cell(hud.get("source_check"), _SOURCE_CHECKS),
        "kind": safe_cell(ev.get("kind")),
        "message": safe_cell(ev.get("message")),
        "permit_state": hud.get("permit_state") or "",
        "retreat": _enum_cell(hud.get("retreat"), ("automatic", "commanded")),
        "gas_values_from": origin,
        "gas_alarm": _alarm_cell(origin, values, hud),
        "data_json": safe_cell(json.dumps(data, separators=(",", ":"), sort_keys=True))
                     if data is not None else "",
    }
    for key in GAS_COLUMNS:
        value = values.get(key)
        row[key] = "" if value is None else value
    return row


def csv_text(events):
    """The whole CSV document (UTF-8 BOM so Excel shows the dashes/units)."""
    out = io.StringIO()
    out.write("\ufeff")
    writer = csv.DictWriter(out, fieldnames=CSV_COLUMNS, lineterminator="\r\n")
    writer.writeheader()
    for ev in events:
        writer.writerow(flatten(ev))
    return out.getvalue()


def filename(ext, now=None):
    stamp = (now or datetime.datetime.now(datetime.timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
    return "spot-hud-audit-%s.%s" % (stamp, ext)
