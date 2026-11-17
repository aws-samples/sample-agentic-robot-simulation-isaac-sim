/*
 * Presenter HUD client for the Spot confined-space entry demo.
 *
 * Vanilla JS, no build step, no CDN (the presenter laptop may have nothing
 * but the SSM tunnel). Talks only to hud/server.py:
 *   WS  /ws              hello (full state), then events (pushed immediately),
 *                        diff (changed top-level keys, ~4 Hz) and ping (idle)
 *   GET /api/state       polling fallback while the WebSocket is down
 *   GET /api/camera.jpg  fetched only when state.camera.seq changes
 *   GET /api/audit.jsonl / .csv   plain download links (no script involved)
 *
 * Panels fed by server-normalized state (hud/ros_bridge.py):
 *   permit.checklist   isolation / LOTO / attendant / rescue plan
 *                      (ok true = check, false = cross, null = unknown '—')
 *   ventilation        {state on|off|unknown, fraction_remaining, tau_s, ...}
 *   purge_hold         the mission's purge_hold event {fraction_remaining,
 *                      purge_target, eta_s, active, reached, ...}: the amber
 *                      PURGE HOLD panel shows exactly these numbers (the
 *                      agent quotes the same ones)
 *   probe              {rows [{z, inside_m, o2_pct.., alarms}] (one per point: the
 *                      mission samples 2 depths x 3 heights), worst, verdict,
 *                      xy, points_xy, depths_m, ...}
 *   alert.reading / alert.note   triggering reading + retreat note (banner)
 *   ev.hud.source_check          "unverified" when the self-declared source
 *                      is not one this demo's publishers use (T3): badged
 *                      in the log; such an event changed no HUD state
 *   ev.hud.record      "safety" / "local" lines (alarms, permit, retreat,
 *                      test verdicts) are trimmed from the log last (T42)
 *   ingest             per-topic dropped / rejected / evicted counters: the
 *                      INGEST LIMITED badge shows when a flood was cut (T42)
 *   world              {undisclosed, pending, last, window_s}: simulator
 *                      atmosphere changes (the facade's world_control
 *                      records) with no presenter disclosure; the UNDISCLOSED
 *                      SIM CHANGE badge and the HUD's own log line (M23)
 *   guard              the agent's SafetyGuard heartbeat status (T16): a
 *                      header pill always, and a header strip for GUARD
 *                      DOWN / NOT ARMED / NOT RUNNING (red if Spot is inside)
 *
 * Safety of the display itself:
 *   - all event/agent text is inserted with textContent, never innerHTML
 *     (LLM output flows straight into this page);
 *   - a watchdog drops a silent socket after 6 s and reconnects with backoff;
 *   - stale topics are labeled STALE / NO DATA instead of showing frozen
 *     numbers as if they were live.
 */
"use strict";

(function () {
  const STALE_S = 3.0;          // topic age beyond which values are shown as stale
  const WATCHDOG_MS = 6000;     // server pings every 2 s; 6 s of silence = dead link
  const CONNECT_TIMEOUT_MS = 5000;
  const FLASH_MS = 8000;
  const FLASH_MAX_AGE_S = 10;   // never replay an old alert after a reload/reconnect
  const MAX_FEED = 200;        // routine lines kept in the log
  const MAX_FEED_KEPT = 600;   // hard cap, protected (safety) lines included
  const TRAIL_MAX = 1500;
  const TRAIL_MIN_STEP_M = 0.1;
  const DOOR_WIDTH_M = 1.3;     // sim/build_scene.py DOOR_WIDTH (not in the site contract)

  const PERMIT = {
    NOT_STARTED: ["NOT STARTED", "permit-gray"],
    PRE_ENTRY_TESTING: ["PRE-ENTRY TESTING", "permit-amber"],
    ENTRY_AUTHORIZED: ["ENTRY AUTHORIZED", "permit-green"],
    ENTRY_IN_PROGRESS: ["ENTRY IN PROGRESS", "permit-green"],
    COMPLETED: ["COMPLETED", "permit-blue"],
    ABORTED: ["ABORTED", "permit-red"],
  };
  // Mission text names permit states in code form ("Permit CSE-0427:
  // PRE_ENTRY_TESTING -> ENTRY_AUTHORIZED"); the audience reads them the way
  // the permit card shows them. Display only: the audit exports stay raw.
  const PERMIT_WORD_RE = new RegExp("\\b(" + Object.keys(PERMIT).join("|") + ")\\b", "g");
  const plainText = (s) => String(s).replace(PERMIT_WORD_RE, (m) => PERMIT[m][0]).replace(/ -> /g, " \u2192 ");

  // OSHA sampling order: O2 -> LEL -> H2S -> CO. Scales chosen so the
  // threshold line sits well inside the bar.
  const GAUGES = [
    { id: "o2", key: "o2_pct", label: "O₂", unit: "%", min: 15, max: 25, digits: 1 },
    { id: "lel", key: "lel_pct", label: "LEL", unit: "%", min: 0, max: 25, digits: 1 },
    { id: "h2s", key: "h2s_ppm", label: "H₂S", unit: "ppm", min: 0, max: 40, digits: 1 },
    { id: "co", key: "co_ppm", label: "CO", unit: "ppm", min: 0, max: 100, digits: 1 },
  ];
  // Used only until the server's snapshot (which carries them) arrives.
  const FALLBACK_THRESHOLDS = {
    o2: { low: 19.5, high: 23.5 }, lel: { alarm_at: 10 }, h2s: { alarm_at: 10 }, co: { alarm_at: 35 },
  };

  // Fallback labels; the server's permit.checklist carries its own.
  const CHECKLIST = [
    { key: "isolation", label: "Isolation" },
    { key: "loto", label: "LOTO" },
    { key: "attendant", label: "Attendant" },
    { key: "rescue_plan", label: "Rescue plan" },
  ];
  const INCIDENT_WINDOW_S = 120;  // alert <-> ABORTED transition pairing window

  // T16: hud/ros_bridge.py HudState._guard_view statuses.
  const GUARD = {
    monitoring: ["GUARD MONITORING", "guard-ok"],
    standby: ["GUARD STANDBY", "guard-standby"],
    not_armed: ["GUARD NOT ARMED", "guard-bad"],
    stalled: ["GUARD STALLED", "guard-bad"],
    down: ["GUARD DOWN", "guard-bad"],
    not_running: ["GUARD NOT RUNNING", "guard-warn"],
    not_recorded: ["GUARD NOT RECORDED", "guard-off"],
  };
  // [title, why]; "Spot is inside the tank" goes between them, so the part
  // a narrow header cuts off is the explanation, not the state.
  const GUARD_STRIP = {
    down: ["GUARD DOWN", "no safety-guard heartbeat for over 3 s"],
    not_armed: ["GUARD NOT ARMED", "the safety guard reports standby"],
    stalled: ["GUARD STALLED", "the safety guard's gas checks have stopped completing"],
    not_running: ["GUARD NOT RUNNING", "no safety-guard heartbeat"],
  };

  // Plain-language names for the agent's tools in the mission log (the
  // audience should never have to read "ventilation_status").
  const TOOL_LABELS = {
    get_status: "Check robot status",
    get_gas_reading: "Read the gas sensor",
    prepare_permit: "Record the entry permit",
    claim_and_power_on: "Claim the robot and power on",
    stand_up: "Stand up",
    sit_down: "Sit down",
    walk_to: "Walk to a point",
    walk_to_named: "Walk to a named place",
    run_atmospheric_test: "Pre-entry atmospheric test",
    start_ventilation: "Start forced ventilation (simulated)",
    ventilation_status: "Check ventilation progress",
    enter_confined_space: "Enter the tank",
    full_confined_space_entry: "Full confined-space entry",
    retreat_now: "Retreat now",
    explain_last_incident: "Review the last incident",
    emergency_stop: "Emergency stop",
    inject_gas_demo_spike: "Presenter: inject a simulated gas spike",
    clear_injected_gas: "Presenter: clear the simulated gas",
  };
  const KIND_LABELS = {
    atmos_test: "atmospheric test", atmos_test_start: "test started", atmos_probe: "probe readings",
    atmos_result: "test result", entry_denied: "entry denied", purge_incomplete: "purge hold",
    permit_prepared: "permit recorded", retreat_start: "retreat started",
    retreat_complete: "retreat complete", demo_inject: "presenter demo", demo_clear: "presenter demo",
    demo_reset: "presenter demo", monitor: "continuous monitoring", ventilation: "ventilation (simulated)",
    operator: "instruction", exit_start: "exit", robot: "robot",
    world_control: "simulator control", facade_service: "robot service call",
    facade_publishers: "command publishers", undisclosed_change: "no disclosure",
  };
  const SOURCE_LABELS = {
    guard: "SAFETY GUARD", presenter: "PRESENTER", hud: "HUD", facade: "SIMULATOR",
  };
  // Nothing on DDS is authenticated; a source outside the allowlist is shown
  // as claimed but badged, and the HUD let it change nothing (T3).
  const UNVERIFIED_HINT = "Self-declared source not used by this demo's publishers. " +
    "Shown as received; it changed no permit, checklist or alarm label on this HUD.";
  // Not drawn as log lines: purge_hold is shown as state (the vent panel),
  // not as a second line repeating the purge_incomplete explanation;
  // probe_seat is a rehearsal audit record (seat skip vs hop).
  const STATE_ONLY_KINDS = ["purge_hold", "probe_seat"];

  const $ = (id) => document.getElementById(id);
  const dom = {
    conn: $("conn"), modeBadge: $("mode-badge"), footerMode: $("footer-mode"),
    ingestBadge: $("ingest-badge"), worldBadge: $("world-badge"),
    alarmStrip: $("alarm-strip"), guard: $("guard"), guardStrip: $("guard-strip"),
    permitState: $("permit-state"), permitId: $("permit-id"),
    permitSince: $("permit-since"), permitMsg: $("permit-msg"),
    checklist: $("checklist"), checklistSummary: $("checklist-summary"),
    incident: $("incident"), incidentReading: $("incident-reading"), incidentNote: $("incident-note"),
    vent: $("vent"), ventChip: $("vent-chip"), ventFill: $("vent-fill"), ventRes: $("vent-res"),
    ventText: $("vent-text"), ventLabel: $("vent-label"), ventSub: $("vent-sub"),
    incidentTrigger: $("incident-trigger"), incidentNow: $("incident-now"),
    probeWhere: $("probe-where"), probeVerdict: $("probe-verdict"), probeBody: $("probe-body"),
    gauges: $("gauges"), chips: $("chips"), feed: $("feed"),
    cam: $("cam"), camOverlay: $("cam-overlay"), map: $("map"),
    flash: $("flash"), flashTitle: $("flash-title"), flashReason: $("flash-reason"),
    flashReading: $("flash-reading"), flashNote: $("flash-note"), flashDetail: $("flash-detail"),
  };

  let model = {};             // latest server state (without events/trail)
  let trail = [];
  let trailEpoch = null;
  let lastEventSeq = 0;
  let lastAlertSeq = null;    // null until the first state arrives
  let clockOffset = 0;        // server wall clock - local wall clock (s)

  // ------------------------------------------------------------------ utils

  function el(tag, cls, text) {
    const node = document.createElement(tag);
    if (cls) node.className = cls;
    if (text !== undefined && text !== null) node.textContent = String(text);
    return node;
  }
  const num = (v) => (typeof v === "number" && isFinite(v) ? v : null);
  const serverNow = () => Date.now() / 1000 + clockOffset;
  const clamp01 = (v) => Math.max(0, Math.min(1, v));
  // One rounding rule for every ventilation percentage on the page (the
  // mission's messages and the agent use the same: 0.1 %). Targets print as
  // set: 0.08 -> "8 %".
  const fmtPct1 = (frac) => (100 * frac).toFixed(1) + "\u00a0%";
  const fmtTarget = (frac) => String(Math.round(1000 * frac) / 10) + "\u00a0%";
  function activeHold() {
    const h = model.purge_hold;
    return h && typeof h === "object" && h.active ? h : null;
  }

  function linkAge(topic) {
    const link = model.link || {};
    return num(link[topic]);
  }
  function isStale(topic) {
    const age = linkAge(topic);
    return !online() || age === null || age > STALE_S;
  }

  function fmtSim(t) {
    if (num(t) === null) return "--";
    const m = Math.floor(t / 60);
    const s = t - m * 60;
    return "T+" + String(m).padStart(2, "0") + ":" + s.toFixed(1).padStart(4, "0");
  }
  function fmtDuration(sec) {
    if (num(sec) === null || sec < 0) return "";
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return m + ":" + String(s).padStart(2, "0");
  }
  function fmtEventTime(ev) {
    const t = num(ev.t);
    // Contract: t is sim time (s) or wall epoch; epoch values are huge.
    if (t !== null && t < 1e8) return fmtSim(t);
    const wall = t !== null ? t : num(ev.rx_wall);
    if (wall === null) return "";
    return new Date(wall * 1000).toLocaleTimeString([], { hour12: false });
  }

  function thresholds() {
    return model.thresholds || FALLBACK_THRESHOLDS;
  }

  function alarmList(gas) {
    if (!gas || !gas.alarms) return "";
    const th = thresholds();
    const parts = [];
    for (const g of GAUGES) {
      const v = num(gas[g.key]);
      if (!gas.alarms[g.id] || v === null) continue;
      if (g.id === "o2") {
        parts.push("O₂ " + v.toFixed(1) + " % (safe " + th.o2.low + "–" + th.o2.high + " %)");
      } else {
        parts.push(g.label + " " + v.toFixed(g.digits) + " " + g.unit + " (≥ " + th[g.id].alarm_at + " " + g.unit + ")");
      }
    }
    return parts.join(" · ");
  }

  // -------------------------------------------------------- static DOM build

  const gaugeEls = {};
  function buildGauges() {
    for (const g of GAUGES) {
      const root = el("div", "gauge is-stale");
      const row = el("div", "gauge-row");
      row.append(el("span", "gauge-name", g.label));
      const flag = el("span", "gauge-flag", "NO DATA");
      row.append(flag);
      const value = el("div", "gauge-value");
      const numEl = el("span", "gauge-num", "--");
      value.append(numEl, el("span", "gauge-unit", g.unit));
      const track = el("div", "gauge-track");
      const band = el("div", "gauge-band");
      const fill = el("div", "gauge-fill");
      const thrA = el("div", "gauge-thr");
      const thrB = el("div", "gauge-thr");
      const marker = el("div", "gauge-marker");
      track.append(band);
      if (g.id !== "o2") track.append(fill);
      track.append(thrA);
      if (g.id === "o2") track.append(thrB);
      track.append(marker);
      const scale = el("div", "gauge-scale");
      const limit = el("span", "gauge-limit", "");
      scale.append(el("span", "", g.min), limit, el("span", "", g.max + " " + g.unit));
      root.append(row, value, track, scale);
      dom.gauges.append(root);
      gaugeEls[g.id] = { root, flag, numEl, band, fill, thrA, thrB, marker, limit };
    }
  }

  const checkEls = {};
  function buildChecklist() {
    for (const def of CHECKLIST) {
      const li = el("li", "check check-unknown");
      const mark = el("span", "check-mark", "—");
      mark.setAttribute("aria-hidden", "true");
      const label = el("span", "check-label", def.label);
      const text = el("span", "check-text", "—");
      li.append(mark, label, text);
      dom.checklist.append(li);
      checkEls[def.key] = { li, mark, label, text };
    }
  }

  const CHIPS = ["Lease", "Power", "Stance", "E-stop", "Battery", "Speed", "Sim time", "Sim link"];
  const chipEls = {};
  function buildChips() {
    for (const name of CHIPS) {
      const chip = el("div", "chip chip-off");
      const value = el("span", "chip-value", "--");
      chip.append(el("span", "chip-label", name), value);
      dom.chips.append(chip);
      chipEls[name] = { chip, value };
    }
  }

  // ---------------------------------------------------------------- render

  let renderQueued = false;
  function scheduleRender() {
    if (renderQueued) return;
    renderQueued = true;
    requestAnimationFrame(() => {
      renderQueued = false;
      try {
        render();
      } catch (err) {
        console.error("HUD render failed", err);  // keep the next frame alive
      }
    });
  }

  function render() {
    renderConnection();
    renderPermit();
    renderChecklist();
    renderVentilation();
    renderProbe();
    renderGauges();
    renderChips();
    renderCameraOverlay();
    renderIngest();
    renderWorld();
    renderAlarmStrip();
    renderIncident();   // after the alarm strip: it yields the header slot to it
    renderGuard();      // last: its warning strip yields to the incident summary
    drawMap();
  }

  function renderConnection() {
    const server = model.server || {};
    let text, cls;
    if (conn.state === "live" && server.bridge_ok === false) {
      text = "BRIDGE DOWN"; cls = "conn-offline";
    } else if (conn.state === "live") {
      // "LIVE" only for the simulator feed: a green LIVE beside "REPLAY ·
      // RECORDED EVENTS" (or synthetic data) contradicts the page.
      text = server.mode === "replay" || server.mode === "fake" ? "CONNECTED" : "LIVE";
      cls = "conn-live";
    } else if (conn.state === "polling") {
      text = "POLLING"; cls = "conn-polling";
    } else {
      text = "HUD OFFLINE · RECONNECTING"; cls = "conn-offline";
    }
    dom.conn.textContent = text;
    dom.conn.className = "conn-pill " + cls;
    document.body.classList.toggle("is-offline", !online());
    const fake = server.mode === "fake";
    const replay = server.mode === "replay";
    dom.modeBadge.hidden = !(fake || replay);
    dom.modeBadge.textContent = replay ? "REPLAY · RECORDED EVENTS" : "SYNTHETIC DATA";
    dom.footerMode.textContent = fake
      ? "Data: SYNTHETIC (--fake), not the simulator"
      : replay ? "Data: REPLAY of a recorded dry run, not live"
      : server.mode === "ros" ? "Data: live ROS 2 topics" : "";
  }

  function renderPermit() {
    const p = model.permit || {};
    const [label, cls] = PERMIT[p.state] || [String(p.state || "UNKNOWN"), "permit-gray"];
    dom.permitState.textContent = label;
    dom.permitState.className = "permit-state " + cls;
    dom.permitId.textContent = p.permit_id ? "Permit " + p.permit_id : "";
    const since = num(p.since_wall);
    dom.permitSince.textContent = since !== null ? "for " + fmtDuration(serverNow() - since) : "";
    const hold = p.state === "PRE_ENTRY_TESTING" ? activeHold() : null;
    const target = hold ? num(hold.purge_target) : null;
    dom.permitMsg.textContent = hold
      ? "Purge hold: entry waits for ventilation to reach " + (target !== null ? "≤ " + fmtTarget(target) : "its target")
      : plainText(p.message || "");
  }

  function renderChecklist() {
    const p = model.permit || {};
    const rows = Array.isArray(p.checklist) ? p.checklist : [];
    for (const def of CHECKLIST) {
      const e = checkEls[def.key];
      const row = rows.find((r) => r && r.key === def.key) || {};
      const known = row.ok === true || row.ok === false;
      const kind = row.ok === true ? "ok" : row.ok === false ? "bad" : "unknown";
      e.li.className = "check check-" + kind;
      e.mark.textContent = kind === "ok" ? "✓" : kind === "bad" ? "✗" : "—";
      e.label.textContent = row.label || def.label;
      e.text.textContent = known ? (row.text || (row.ok ? "Confirmed" : "Not set")) : "—";
      e.li.title = (row.label || def.label) + ": " + (known ? (row.ok ? "confirmed" : "NOT confirmed") : "unknown");
    }
    const s = p.checklist_summary;
    const anyKnown = rows.some((r) => r && (r.ok === true || r.ok === false));
    if (!s || !anyKnown) {
      dom.checklistSummary.textContent = "awaiting permit details";
      dom.checklistSummary.className = "checklist-summary";
    } else {
      dom.checklistSummary.textContent = s.confirmed + " / " + s.total + " confirmed";
      dom.checklistSummary.className = "checklist-summary " + (s.complete ? "sum-ok" : "sum-bad");
    }
  }

  // After an abort, keep the triggering reading + retreat note on screen in
  // the header (the full-screen flash only lasts FLASH_MS); the live alarm
  // strip takes the header slot while the gas alarm is still on.
  function renderIncident() {
    const p = model.permit || {};
    const a = model.alert || {};
    const t = num(a.t_wall);
    const since = num(p.since_wall);
    const show = p.state === "ABORTED" && a.seq > 0 && !!(a.reading || a.note) &&
      t !== null && since !== null && Math.abs(t - since) < INCIDENT_WINDOW_S &&
      dom.alarmStrip.hidden && !guardAlarm();
    dom.incident.hidden = !show;
    if (!show) return;
    // "Trigger" is the reading that fired the alarm; "now" is the live
    // gauge value of the same gas, so the two numbers never look like a
    // contradiction on one screen.
    dom.incidentTrigger.textContent = a.reading || "";
    dom.incidentNow.textContent = nowText(a.reading);
    dom.incidentReading.hidden = !a.reading;
    const detail = alertDetail(a);
    dom.incidentNote.textContent = [a.note, detail].filter(Boolean).join(" · ");
    dom.incidentNote.hidden = !(a.note || detail);
  }

  function nowText(reading) {
    const gas = model.gas;
    if (!reading || !gas || isStale("gas")) return "";
    const parts = [];
    for (const g of GAUGES) {
      const name = g.id === "o2" ? "O2" : g.id.toUpperCase();
      const v = num(gas[g.key]);
      if (v !== null && new RegExp("\\b" + name + "\\b").test(reading)) parts.push(g.label + " " + v.toFixed(g.digits) + " " + g.unit);
    }
    return parts.length ? "now " + parts.join(", ") : "";
  }

  function renderVentilation() {
    const v = model.ventilation && typeof model.ventilation === "object" ? model.ventilation : null;
    let cls = "vent-unknown", chip = "—", text = "No ventilation data";
    let label = "Forced ventilation", sub = "";
    let frac = null, residual = null, fill = null;
    if (v) {
      frac = num(v.fraction_remaining);
      residual = num(v.residual);
      const tau = num(v.tau_s);
      if (v.state === "on") { cls = "vent-on"; chip = "ON"; }
      else if (v.state === "off") { cls = "vent-off"; chip = "OFF"; }
      else { chip = v.mode ? String(v.mode).toUpperCase() : "UNKNOWN"; }
      if (frac !== null) {
        if (v.state === "off" && frac >= 0.999) text = "Tank not yet ventilated (simulated)";
        else text = (v.state === "off" ? "Ventilation stopped: " : "") + fmtPct1(frac) + " of the unventilated contamination remains";
        if (tau !== null && v.state === "on") text += " · τ " + (tau < 10 ? tau.toFixed(1) : Math.round(tau)) + " s (sim)";
        fill = frac;
      } else {
        text = v.state === "off" ? "Tank unventilated" : (v.mode ? "Mode: " + v.mode : "");
      }
      if (isStale("gas")) { chip = "STALE"; cls += " vent-stale"; }
    }
    const hold = activeHold();
    if (hold) {
      // Amber PURGE HOLD: the numbers are the mission's purge_hold event
      // (what the agent says); the bar is live progress toward the target.
      const f0 = num(hold.fraction_remaining), target = num(hold.purge_target);
      const live = v && !isStale("gas") ? num(v.fraction_remaining) : null;
      const reached = !!hold.reached || (live !== null && target !== null && live <= target + 1e-9);
      const eta = num(hold.eta_s), since = num(hold.rx_wall);
      const left = eta !== null && since !== null ? Math.max(0, eta - (serverNow() - since)) : null;
      if (reached) fill = 1;
      else if (f0 !== null && target !== null && live !== null && f0 > target) fill = clamp01((f0 - live) / (f0 - target));
      else if (eta && since !== null) fill = clamp01((serverNow() - since) / eta);
      else fill = 0;
      cls = "vent-hold" + (reached ? " vent-reached" : "");
      label = hold.estimate ? "Purge hold (ESTIMATE: no current simulator reading)" : "Purge hold";
      chip = reached ? "TARGET REACHED" : "HOLD";
      sub = reached ? "Ready to re-test"
        : v && v.state === "off" ? "Ventilation OFF: start it to clear the hold"
        : "Waiting for ventilation to reach target";
      // The mission's own rounded percent (its message and the agent say
      // the same): re-rounding the 4-digit fraction can differ by 0.1.
      const pct0 = num(hold.percent_remaining);
      const at = pct0 !== null ? pct0.toFixed(1) + "\u00a0%" : f0 !== null ? fmtPct1(f0) : null;
      text = (at !== null ? at + " left at test" : "Remaining: unknown") +
        (target !== null ? " · target ≤\u00a0" + fmtTarget(target) : "");
      if (reached && live !== null) text += " · now " + fmtPct1(live);
      else if (left !== null && left >= 0.5) text += " · about " + Math.ceil(left) + " s";
      residual = null;
    }
    dom.vent.className = "vent " + cls;
    dom.ventLabel.textContent = label;
    dom.ventSub.textContent = sub;
    dom.ventSub.hidden = !sub;
    dom.ventChip.textContent = chip;
    dom.ventText.textContent = text;
    dom.ventFill.style.width = (fill === null ? 0 : 100 * clamp01(fill)) + "%";
    dom.ventRes.hidden = residual === null;
    if (residual !== null) dom.ventRes.style.left = 100 * clamp01(residual) + "%";
  }

  function fmtAge(sec) {
    if (num(sec) === null) return "";
    if (sec < 5) return "just now";
    if (sec < 90) return Math.round(sec) + " s ago";
    return Math.round(sec / 60) + " min ago";
  }

  function fmtHeight(z) {
    const two = z.toFixed(2);
    return (two.endsWith("0") ? z.toFixed(1) : two) + " m";
  }

  let probeKey = null;
  function renderProbe() {
    const pr = model.probe && typeof model.probe === "object" ? model.probe : null;
    const rows = pr && Array.isArray(pr.rows) ? pr.rows : [];
    if (!rows.length) {
      if (probeKey !== "empty") {
        // Placeholder rows keep the panel height fixed, so the first result
        // does not shift the layout (or the mission log below it).
        probeKey = "empty";
        dom.probeBody.textContent = "";
        for (const label of ["—", "—", "—", "—", "—", "—", "Worst"]) addProbeRow(label, {}, null, label === "Worst" ? "probe-worst" : "");
      }
      if (pr && pr.error) {   // the facade rejected the request (e.g. beyond reach)
        dom.probeWhere.textContent = "Probe request rejected: " + pr.error;
        dom.probeVerdict.hidden = false;
        dom.probeVerdict.textContent = "NO READING";
        dom.probeVerdict.className = "probe-verdict pv-warn";
        dom.probeVerdict.title = "The simulator rejected the probe request";
      } else {
        dom.probeWhere.textContent = "No probe result yet";
        dom.probeVerdict.hidden = true;
      }
      return;
    }
    const where = [];
    const inside = num(pr.inside_m);
    const depths = Array.isArray(pr.depths_m) ? pr.depths_m.map(num).filter((d) => d !== null) : [];
    if (depths.length > 1) where.push(depths.map((d) => d.toFixed(1)).join(" / ") + " m inside the door");
    else if (inside !== null) where.push(inside.toFixed(1) + " m inside the door");
    if (num(pr.t) !== null && pr.t < 1e8) where.push(fmtSim(pr.t));
    const age = num(pr.rx_wall) !== null ? fmtAge(serverNow() - pr.rx_wall) : "";
    if (age) where.push(age);
    dom.probeWhere.textContent = where.join(" · ");

    let verdict, vcls, vtitle;
    if (pr.verdict === true) {
      verdict = "PASS"; vcls = "pv-ok"; vtitle = "Verdict published with the probe result";
    } else if (pr.verdict === false && !pr.any_alarm) {
      // Every reading is within the limits, yet entry was not authorized:
      // an amber hold, never a red FAIL over an all-green table.
      verdict = pr.held ? "PURGE HOLD" : "NOT AUTHORIZED";
      vcls = "pv-warn";
      vtitle = pr.held ? "All readings within the limits; entry waits for the ventilation purge"
        : "Readings within the limits, but the mission did not authorize entry (see the log)";
    } else if (pr.verdict === false) {
      verdict = "FAIL"; vcls = "pv-bad"; vtitle = "Verdict published with the probe result";
    } else {
      verdict = pr.any_alarm ? "OVER LIMIT" : pr.complete ? "WITHIN LIMITS" : "INCOMPLETE";
      vcls = pr.any_alarm ? "pv-bad" : pr.complete ? "pv-ok" : "pv-warn";
      vtitle = "HUD check of these readings against OSHA 1910.146 App. E";
    }
    dom.probeVerdict.hidden = false;
    dom.probeVerdict.textContent = verdict;
    dom.probeVerdict.className = "probe-verdict " + vcls;
    dom.probeVerdict.title = vtitle;

    const key = JSON.stringify([pr.rx_wall, rows, pr.worst]);
    if (key === probeKey) return;   // rebuild the table only when the result changes
    probeKey = key;
    dom.probeBody.textContent = "";
    // Highest first, so the table reads like the tank wall (floor at the
    // bottom); at each height the nearer depth first.
    const ordered = rows.slice().sort((a, b) =>
      (num(b.z) ?? -1) - (num(a.z) ?? -1) || (num(a.inside_m) ?? 0) - (num(b.inside_m) ?? 0));
    for (const r of ordered) {
      const z = num(r.z), d = num(r.inside_m);
      const label = (z === null ? "?" : fmtHeight(z)) + (d === null ? "" : " · " + d.toFixed(1) + " in");
      const title = (z === null ? "unknown height" : fmtHeight(z) + " above the floor") +
        (d === null ? "" : ", " + d.toFixed(1) + " m inside the door");
      addProbeRow(label, r, r.alarms, "", title);
    }
    if (pr.worst) addProbeRow("Worst", pr.worst, pr.worst.alarms, "probe-worst");
  }

  function addProbeRow(label, values, alarms, cls, title) {
    const tr = el("tr", cls || "");
    const th = el("th", "", label);
    th.scope = "row";
    if (title) th.title = title;
    tr.append(th);
    for (const g of GAUGES) {
      const v = num(values[g.key]);
      const alarm = !!(alarms && alarms[g.id]);
      const td = el("td", v === null ? "cell-na" : alarm ? "cell-alarm" : "cell-ok",
        v === null ? "—" : v.toFixed(g.digits) + (alarm ? " ▲" : ""));
      if (alarm) td.title = g.label + " over the OSHA App. E limit";
      tr.append(td);
    }
    dom.probeBody.append(tr);
  }

  const pct = (g, v) => 100 * clamp01((v - g.min) / (g.max - g.min));

  function renderGauges() {
    const gas = model.gas;
    const stale = isStale("gas");
    const th = thresholds();
    for (const g of GAUGES) {
      const e = gaugeEls[g.id];
      const v = gas ? num(gas[g.key]) : null;
      const alarm = !!(gas && gas.alarms && gas.alarms[g.id]);
      const t = th[g.id] || FALLBACK_THRESHOLDS[g.id];
      if (g.id === "o2") {
        e.band.style.left = pct(g, t.low) + "%";
        e.band.style.width = pct(g, t.high) - pct(g, t.low) + "%";
        e.thrA.style.left = pct(g, t.low) + "%";
        e.thrB.style.left = pct(g, t.high) + "%";
        e.limit.textContent = "alarm < " + t.low + " or > " + t.high;
      } else {
        e.band.style.left = "0%";
        e.band.style.width = pct(g, t.alarm_at) + "%";
        e.thrA.style.left = pct(g, t.alarm_at) + "%";
        e.limit.textContent = "alarm ≥ " + t.alarm_at;
      }
      e.numEl.textContent = v === null ? "--" : v.toFixed(g.digits);
      if (v !== null) {
        e.marker.style.left = pct(g, v) + "%";
        e.fill.style.width = pct(g, v) + "%";
      }
      const showStale = stale || v === null;
      e.root.classList.toggle("is-stale", showStale);
      e.root.classList.toggle("is-alarm", alarm && !showStale);
      e.flag.textContent = showStale ? (gas ? "STALE" : "NO DATA") : alarm ? "ALARM" : "OK";
    }
  }

  function setChip(name, value, kind) {
    const c = chipEls[name];
    c.value.textContent = value;
    c.chip.className = "chip chip-" + kind;
  }

  function renderChips() {
    const s = model.status;
    const statusStale = isStale("status");
    const age = linkAge("status");
    if (!s || statusStale) {
      for (const name of CHIPS) setChip(name, "--", "off");
    } else {
      setChip("Lease", s.claimed ? "CLAIMED" : "RELEASED", s.claimed ? "ok" : "off");
      setChip("Power", s.powered ? "MOTORS ON" : "OFF", s.powered ? "ok" : "off");
      setChip("Stance", s.standing_up ? "STANDING UP" : s.standing ? "STANDING" : "SITTING",
        s.standing_up ? "warn" : s.standing ? "ok" : "off");
      setChip("E-stop", s.estopped ? "ENGAGED" : "CLEAR", s.estopped ? "bad" : "ok");
      const b = num(s.battery_pct);
      setChip("Battery", b === null ? "--" : Math.round(b) + " %",
        b === null ? "off" : b < 15 ? "bad" : b < 30 ? "warn" : "ok");
      setChip("Sim time", fmtSim(num(s.sim_time)), "neutral");
    }
    const pose = model.pose;
    const speed = pose && !isStale("odometry") ? num(pose.speed) : null;
    setChip("Speed", speed === null ? "--" : speed.toFixed(2) + " m/s", speed === null ? "off" : "neutral");
    if (!online()) setChip("Sim link", "HUD OFFLINE", "bad");
    else if ((model.server || {}).mode === "replay" && age !== null && age <= STALE_S) setChip("Sim link", "REPLAY", "warn");
    else if (age === null) setChip("Sim link", "NO DATA", "off");
    else if (age > STALE_S) setChip("Sim link", "STALE " + Math.round(age) + " s", "bad");
    else setChip("Sim link", "LIVE", "ok");
  }

  function renderCameraOverlay() {
    const cam = model.camera || {};
    const age = linkAge("camera");
    if (!cam.seq || age === null) {
      dom.camOverlay.hidden = false;
      dom.camOverlay.textContent = "NO CAMERA SIGNAL";
    } else if (isStale("camera")) {
      dom.camOverlay.hidden = false;
      dom.camOverlay.textContent = "CAMERA STALE";
    } else {
      dom.camOverlay.hidden = true;
    }
  }

  // T16: is the guard status an alarm (down, not armed, stalled, or not running
  // while Spot is inside)? Takes the header slot from the incident summary.
  function guardAlarm() {
    const g = model.guard;
    return online() && !!(g && typeof g === "object" && g.alarm);
  }

  function renderGuard() {
    const g = model.guard && typeof model.guard === "object" ? model.guard : null;
    const [text, cls] = (online() && g && GUARD[g.status]) || ["GUARD —", "guard-off"];
    dom.guard.textContent = text;
    dom.guard.className = "conn-pill " + cls;
    const alarm = guardAlarm();
    const line = online() && g ? GUARD_STRIP[g.status] : null;
    // The live gas alarm keeps the slot; a guard warning (Spot outside)
    // also yields to the incident summary.
    const show = !!line && dom.alarmStrip.hidden && (alarm || dom.incident.hidden);
    dom.guardStrip.hidden = !show;
    if (show) {
      const parts = [line[0], g.robot_inside ? "Spot is inside the tank" : null, line[1]];
      dom.guardStrip.textContent = parts.filter(Boolean).join(" · ");
      dom.guardStrip.classList.toggle("guard-strip-alarm", alarm);
    }
    document.body.classList.toggle("guard-down", alarm);
  }

  // T42: a flood was cut at the HUD's ingest limits, or pushed records out
  // of its buffers. The count stays visible (the audit export is short).
  function renderIngest() {
    const ing = model.ingest && typeof model.ingest === "object" ? model.ingest : null;
    const total = ing ? num(ing.flood_total) : null;
    dom.ingestBadge.hidden = !(total > 0);
    if (!(total > 0)) return;
    const parts = [];
    for (const [label, counts] of [["dropped", ing.dropped], ["rejected", ing.rejected], ["evicted", ing.evicted]]) {
      for (const [key, n] of Object.entries(counts || {})) parts.push(label + " " + key + ": " + n);
    }
    dom.ingestBadge.textContent = "INGEST LIMITED · " + total.toLocaleString() + " dropped";
    dom.ingestBadge.title = "Messages over the HUD's per-topic limits, malformed, or pushed out of " +
      "its event buffers (possible flood). The audit export lacks them. " + parts.join("; ");
  }

  // M23: the simulator applied an atmosphere change (gas inject / clear,
  // ventilation) that no presenter disclosure accompanied within the window.
  function renderWorld() {
    const w = model.world && typeof model.world === "object" ? model.world : null;
    const n = w ? num(w.undisclosed) : null;
    dom.worldBadge.hidden = !(n > 0);
    if (!(n > 0)) return;
    const last = w.last && typeof w.last === "object" ? w.last : {};
    dom.worldBadge.textContent = "UNDISCLOSED SIM CHANGE · " + n.toLocaleString();
    dom.worldBadge.title = "The simulator applied " + n + " gas or ventilation change(s) that no " +
      "presenter, operator or mission disclosure accompanied within " + (num(w.window_s) || "?") +
      " s. Latest: " + String(last.control || "?") +
      (Array.isArray(last.publishers) && last.publishers.length ? " (publishers: " +
        last.publishers.slice(0, 3).join(", ") + ")" : "") + ". See the log and the audit export.";
  }

  function renderAlarmStrip() {
    const gas = model.gas;
    const active = !!(gas && gas.alarm) && !isStale("gas");
    document.body.classList.toggle("alarm-active", active);
    dom.alarmStrip.hidden = !active;
    if (active) dom.alarmStrip.textContent = "ALARM ACTIVE · " + (alarmList(gas) || "atmospheric alarm");
  }

  // -------------------------------------------------------------- mini map

  let mapBounds = null;
  let mapBoundsKey = "";

  function siteBounds(site) {
    const [cx, cy] = site.tank_center_xy;
    const r = site.tank_radius;
    const pts = [[cx - r, cy - r], [cx + r, cy + r], site.entry_xy, site.door_xy];
    if (site.spawn_xy) pts.push(site.spawn_xy);
    for (const p of site.interior_waypoints || []) pts.push(p);
    const b = { minx: Infinity, maxx: -Infinity, miny: Infinity, maxy: -Infinity };
    for (const [x, y] of pts) {
      b.minx = Math.min(b.minx, x); b.maxx = Math.max(b.maxx, x);
      b.miny = Math.min(b.miny, y); b.maxy = Math.max(b.maxy, y);
    }
    const pad = 1.0;
    b.minx -= pad; b.maxx += pad; b.miny -= pad; b.maxy += pad;
    return b;
  }

  // Bounds only grow (never shrink) so the map does not rescale while the
  // robot walks around inside it.
  function includeInBounds(b, x, y) {
    const m = 1.0;
    if (x < b.minx + m) b.minx = x - 2 * m;
    if (x > b.maxx - m) b.maxx = x + 2 * m;
    if (y < b.miny + m) b.miny = y - 2 * m;
    if (y > b.maxy - m) b.maxy = y + 2 * m;
  }

  function drawMap() {
    const canvas = dom.map;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (!w || !h) return;
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const rem = parseFloat(getComputedStyle(document.documentElement).fontSize) || 16;
    const font = (scale, weight) => (weight || 600) + " " + Math.round(rem * scale) + "px system-ui, sans-serif";
    // Every label and marker registers its box here; Spot's label then
    // takes the first free spot around the arrow (it used to print on top
    // of SPAWN / DOOR / ENTRY when Spot stood on them).
    const taken = [];
    const box = (x0, y0, x1, y1) => ({ x0, y0, x1, y1 });
    const hits = (r) => taken.some((t) => r.x0 < t.x1 && r.x1 > t.x0 && r.y0 < t.y1 && r.y1 > t.y0);
    function textBox(text, x, y, align, px) {
      const wd = ctx.measureText(text).width;
      const x0 = align === "center" ? x - wd / 2 : align === "right" ? x - wd : x;
      return box(x0 - 2, y - px * 0.85, x0 + wd + 2, y + px * 0.25);
    }
    function label(text, x, y, align, px) {
      ctx.textAlign = align;
      ctx.fillText(text, x, y);
      taken.push(textBox(text, x, y, align, px));
    }

    const site = model.site;
    if (!site || !site.tank_center_xy) {
      ctx.fillStyle = "#9aa4ad";
      ctx.font = font(1.1);
      ctx.textAlign = "center";
      ctx.fillText("Waiting for site layout (payload/site)…", w / 2, h / 2);
      return;
    }
    const key = JSON.stringify(site);
    if (key !== mapBoundsKey) {
      mapBounds = siteBounds(site);
      mapBoundsKey = key;
    }
    const pose = model.pose;
    if (pose) includeInBounds(mapBounds, pose.x, pose.y);
    const b = mapBounds;
    const s = Math.min(w / (b.maxx - b.minx), h / (b.maxy - b.miny));
    const ox = (w - (b.maxx - b.minx) * s) / 2;
    const oy = (h - (b.maxy - b.miny) * s) / 2;
    const X = (x) => ox + (x - b.minx) * s;
    const Y = (y) => h - oy - (y - b.miny) * s;   // world y up, canvas y down

    // 1 m grid, every 5 m stronger.
    ctx.lineWidth = 1;
    for (let gx = Math.ceil(b.minx); gx <= b.maxx; gx++) {
      ctx.strokeStyle = gx % 5 === 0 ? "#27303a" : "#181e24";
      ctx.beginPath(); ctx.moveTo(X(gx), Y(b.miny)); ctx.lineTo(X(gx), Y(b.maxy)); ctx.stroke();
    }
    for (let gy = Math.ceil(b.miny); gy <= b.maxy; gy++) {
      ctx.strokeStyle = gy % 5 === 0 ? "#27303a" : "#181e24";
      ctx.beginPath(); ctx.moveTo(X(b.minx), Y(gy)); ctx.lineTo(X(b.maxx), Y(gy)); ctx.stroke();
    }

    // Tank: dark interior + shell with the doorway gap facing door_xy.
    const [tcx, tcy] = site.tank_center_xy;
    const R = site.tank_radius;
    const door = site.door_xy;
    const doorAng = Math.atan2(door[1] - tcy, door[0] - tcx);
    const half = Math.asin(Math.min(1, (site.door_width || DOOR_WIDTH_M) / 2 / R));
    ctx.fillStyle = "#0f1316";
    ctx.beginPath(); ctx.arc(X(tcx), Y(tcy), R * s, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = "#a4623f";
    ctx.lineWidth = Math.max(4, 0.12 * s);
    ctx.beginPath();
    // Canvas angles run clockwise (y down), so the world angle is negated.
    ctx.arc(X(tcx), Y(tcy), R * s, -doorAng + half, -doorAng - half + 2 * Math.PI);
    ctx.stroke();
    ctx.fillStyle = "#6b757e";
    ctx.font = font(0.85, 700);
    label("TANK (unlit interior)", X(tcx), Y(tcy + R) + rem * 2.2, "center", rem * 0.85);  // just inside the top of the shell

    // Door label below the gap; ENTRY goes above its marker and SPOT to the
    // lower right of the arrow, so the three never stack on each other when
    // the robot waits at the entry point 1 m outside the door.
    ctx.fillStyle = "#c9d1d8";
    ctx.font = font(0.8, 700);
    label("DOOR", X(door[0]), Y(door[1]) + rem * 1.9, "center", rem * 0.8);

    // Planned route: entry -> door -> interior waypoints (dashed).
    const route = [site.entry_xy, door].concat(site.interior_waypoints || []);
    ctx.setLineDash([6, 6]);
    ctx.strokeStyle = "#7b8a98";
    ctx.lineWidth = 2;
    ctx.beginPath();
    route.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))));
    ctx.stroke();
    ctx.setLineDash([]);
    (site.interior_waypoints || []).forEach(([x, y], i) => {
      ctx.fillStyle = "#1b2127";
      ctx.strokeStyle = "#7b8a98";
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(X(x), Y(y), rem * 0.65, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      taken.push(box(X(x) - rem * 0.7, Y(y) - rem * 0.7, X(x) + rem * 0.7, Y(y) + rem * 0.7));
      ctx.fillStyle = "#c9d1d8";
      ctx.font = font(0.7, 700);
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(String(i + 1), X(x), Y(y) + 0.5);
      ctx.textBaseline = "alphabetic";
    });

    // Entry + spawn markers.
    const marker = (xy, text, above) => {
      const px = X(xy[0]), py = Y(xy[1]), r = rem * 0.45;
      ctx.fillStyle = "#9aa4ad";
      ctx.beginPath();
      ctx.moveTo(px, py - r); ctx.lineTo(px + r, py); ctx.lineTo(px, py + r); ctx.lineTo(px - r, py);
      ctx.closePath(); ctx.fill();
      taken.push(box(px - r, py - r, px + r, py + r));
      ctx.font = font(0.75, 700);
      label(text, px, above ? py - r - rem * 0.4 : py + r + rem * 0.95, "center", rem * 0.75);
    };
    marker(site.entry_xy, "ENTRY", true);
    if (site.spawn_xy) marker(site.spawn_xy, "SPAWN");

    // Last probe stations (each depth the result sampled, else its xy):
    // amber ok, red over limit.
    const probe = model.probe;
    const stations = probe && Array.isArray(probe.points_xy) && probe.points_xy.length
      ? probe.points_xy : probe && Array.isArray(probe.xy) ? [probe.xy] : [];
    for (const xy of stations) {
      if (!Array.isArray(xy) || num(xy[0]) === null || num(xy[1]) === null) continue;
      const px = X(xy[0]), py = Y(xy[1]);
      ctx.fillStyle = probe.any_alarm ? "#ff3b4f" : "#f2a900";
      ctx.strokeStyle = "#0b0d0f";
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(px, py, rem * 0.3, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      // No text label: it would collide with DOOR / ENTRY; the legend names it.
    }

    // Path walked.
    const pts = trail.slice();
    if (pose) pts.push([pose.x, pose.y]);
    if (pts.length > 1) {
      ctx.strokeStyle = "rgba(79, 195, 247, 0.85)";
      ctx.lineWidth = 3;
      ctx.lineJoin = "round";
      ctx.beginPath();
      pts.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))));
      ctx.stroke();
    }

    // Spot: arrow at the pose, pointing along the heading.
    if (pose) {
      const alarm = !!(model.gas && model.gas.alarm);
      const stale = isStale("odometry");
      const L = Math.max(rem * 1.6, 1.1 * s);
      const W = Math.max(rem * 0.9, 0.5 * s);
      ctx.save();
      ctx.translate(X(pose.x), Y(pose.y));
      ctx.rotate(-(num(pose.yaw) || 0));
      ctx.beginPath();
      ctx.moveTo(L * 0.6, 0);
      ctx.lineTo(-L * 0.4, W / 2);
      ctx.lineTo(-L * 0.2, 0);
      ctx.lineTo(-L * 0.4, -W / 2);
      ctx.closePath();
      ctx.fillStyle = stale ? "#69737c" : alarm ? "#ff3b4f" : "#f4f6f8";
      ctx.strokeStyle = "#0b0d0f";
      ctx.lineWidth = 2;
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      ctx.fillStyle = stale ? "#69737c" : "#f4f6f8";
      ctx.font = font(0.8, 800);
      const text = stale ? "SPOT (stale)" : "SPOT";
      const px = rem * 0.8, cx = X(pose.x), cy = Y(pose.y), gap = L * 0.5;
      // Lower right first (as before), then the other sides of the arrow.
      const spots = [
        [cx + gap, cy + W + rem * 0.5, "left"], [cx + gap, cy - W, "left"],
        [cx - gap, cy + W + rem * 0.5, "right"], [cx - gap, cy - W, "right"],
        [cx, cy - W - rem * 0.4, "center"], [cx, cy + W + rem * 1.2, "center"],
      ];
      const inside = (r) => r.x0 >= 0 && r.x1 <= w && r.y0 >= 0 && r.y1 <= h;
      const pick = spots.find(([x, y, a]) => { const r = textBox(text, x, y, a, px); return inside(r) && !hits(r); }) || spots[0];
      label(text, pick[0], pick[1], pick[2], px);
    }

    // Scale bar (bottom-left): 1, 2 or 5 m, whichever is ~6-12 rem long.
    const meters = [1, 2, 5, 10].find((m) => m * s >= rem * 6) || 10;
    const sx = rem, sy = h - rem;
    ctx.strokeStyle = "#c9d1d8";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(sx, sy - 5); ctx.lineTo(sx, sy); ctx.lineTo(sx + meters * s, sy); ctx.lineTo(sx + meters * s, sy - 5);
    ctx.stroke();
    ctx.fillStyle = "#c9d1d8";
    ctx.font = font(0.75, 700);
    ctx.textAlign = "center";
    ctx.fillText(meters + " m", sx + (meters * s) / 2, sy - rem * 0.4);
  }

  function pushTrail(pose) {
    if (!pose || num(pose.x) === null || num(pose.y) === null) return;
    const last = trail[trail.length - 1];
    if (!last || Math.hypot(pose.x - last[0], pose.y - last[1]) >= TRAIL_MIN_STEP_M) {
      trail.push([pose.x, pose.y]);
      if (trail.length > TRAIL_MAX) trail.splice(0, trail.length - TRAIL_MAX);
    }
  }

  // ------------------------------------------------------------ event feed

  const isTool = (kind) => kind.startsWith("tool");
  const lastToolArgs = {};
  const cap = (s) => (s ? s.charAt(0).toUpperCase() + s.slice(1) : s);

  // "walk_to_named(place=entry)" -> "Walk to entry"; unknown tools fall
  // back to their name with spaces (never an empty line).
  function toolText(name, args) {
    const a = args && typeof args === "object" ? args : {};
    const place = typeof a.place === "string" ? a.place.replace(/_/g, " ") : null;
    switch (name) {
      case "walk_to_named": return place ? "Walk to " + place : TOOL_LABELS[name];
      case "walk_to": return num(a.x) !== null && num(a.y) !== null
        ? "Walk to (" + a.x.toFixed(1) + ", " + a.y.toFixed(1) + ")" : TOOL_LABELS[name];
      case "prepare_permit": return TOOL_LABELS[name] + (typeof a.permit_id === "string" ? " " + a.permit_id : "");
      case "ventilation_status": return num(a.wait_s) > 0
        ? "Wait for the ventilation purge (up to " + Math.round(a.wait_s) + " s)" : TOOL_LABELS[name];
      case "emergency_stop": return a.confirm ? "Emergency stop (confirmed)" : "Emergency stop requested (not confirmed)";
      default: return TOOL_LABELS[name] || cap(String(name || "robot action").replace(/_/g, " "));
    }
  }
  const RESULT_RE = /->\s*(ok|failed:\s*([\s\S]*?))\s*\((\d+(?:\.\d+)?) s\)\s*$/;

  function renderToolEvent(li, kind, ev, data, time) {
    li.classList.add("ev-tool");
    const name = data && typeof data.tool === "string" ? data.tool : (String(ev.message || "").match(/^\w+/) || [""])[0];
    let tag, tagCls = "ev-tag", text;
    if (kind === "tool_result") {
      const m = String(ev.message || "").match(RESULT_RE);
      const ok = data && typeof data.ok === "boolean" ? data.ok : m ? m[1] === "ok" : null;
      const secs = m ? m[3] : data && num(data.duration_s) !== null ? String(data.duration_s) : null;
      const why = m && m[2] ? m[2].trim() : "";
      tag = ok === false ? "NOT DONE" : "DONE";
      tagCls += ok === false ? " tag-warn" : " tag-pass";
      text = toolText(name, lastToolArgs[name]) + (ok === false && why ? " · " + why : secs !== null ? " · " + secs + " s" : "");
    } else {
      const args = data && data.args && typeof data.args === "object" ? data.args : null;
      if (name) lastToolArgs[name] = args;
      tag = "ACTION";
      text = toolText(name, args);
    }
    li.append(el("span", "ev-time", time), el("span", tagCls, tag));
    if (isUnverified(ev)) li.append(unverifiedTag(ev));
    li.append(el("span", "ev-code", text));
    li.title = ev.message || kind;   // raw call for whoever debugs from the HUD
    return li;
  }
  const isUnverified = (ev) => !!(ev.hud && ev.hud.source_check === "unverified");
  function unverifiedTag(ev) {
    const tag = el("span", "ev-tag tag-unverified", "UNVERIFIED SOURCE");
    tag.title = UNVERIFIED_HINT + " Claimed source: " + String(ev.source || "unknown");
    return tag;
  }
  const isSay = (kind) => kind === "say" || kind === "speech" || kind === "narration";
  const cssToken = (s) => String(s).toLowerCase().replace(/[^a-z0-9_-]/g, "");

  function permitOf(data) {
    for (const key of ["permit_state", "state", "to"]) {
      const v = data[key];
      if (typeof v === "string" && PERMIT[v.toUpperCase()]) return v.toUpperCase();
    }
    return null;
  }
  function verdictOf(data) {
    for (const key of ["pass", "passed", "ok"]) {
      if (typeof data[key] === "boolean") return data[key];
    }
    return null;
  }
  // Same rule as HudState._is_alert_event in ros_bridge.py. No regex
  // lookbehind: older Safari rejects it at parse time, which would kill the
  // whole script. "incomplete" is stripped first so retreat_incomplete stays red.
  const ALERT_KIND_RE = /alarm|abort|e[-_]?stop|retreat|hazard|excursion|conflict/;
  const CLEAR_KIND_RE = /complete|done|clear|resolved|finished|end\b|ok\b|release|reset/;
  function isAlertEvent(ev, kind, data) {
    if (data && data.alarm === true) return true;
    return ALERT_KIND_RE.test(kind) && !CLEAR_KIND_RE.test(kind.replace(/incomplete/g, ""));
  }

  function renderEvent(ev) {
    const kind = String(ev.kind || "").toLowerCase();
    const source = String(ev.source || "unknown").toLowerCase();
    const data = ev.data && typeof ev.data === "object" ? ev.data : null;
    const li = el("li", "ev src-" + cssToken(source));
    const time = fmtEventTime(ev);

    if (STATE_ONLY_KINDS.indexOf(kind) >= 0) return null;
    // The HUD's own payload/gas alarm edges are for the audit export; the
    // alarm strip and the guard's own alarm line already show them live.
    // Its undisclosed-change flags (M23) are shown: nothing else says so.
    if (ev.hud && ev.hud.source_check === "local" && kind !== "undisclosed_change") return null;
    if (isTool(kind)) return renderToolEvent(li, kind, ev, data, time);

    const meta = el("div", "ev-meta");
    meta.append(el("span", "ev-time", time), el("span", "ev-src", SOURCE_LABELS[source] || source.toUpperCase()));
    if (isUnverified(ev)) {
      li.classList.add("ev-unverified");
      meta.append(unverifiedTag(ev));
    }
    if (kind === "undisclosed_change") {
      li.classList.add("ev-undisclosed");
      meta.append(el("span", "ev-tag tag-warn", "NO DISCLOSURE"));
    }
    // The facade's world_control records: what the simulator did with a control.
    const sim = source === "facade" && kind === "world_control" && data;
    if (sim && typeof data.accepted === "boolean") {
      meta.append(el("span", data.accepted ? "ev-tag tag-sim" : "ev-tag tag-warn",
        data.accepted ? "APPLIED" : "REFUSED"));
    }
    if (ev.hud && typeof ev.hud.reset_refused === "string") {
      const tag = el("span", "ev-tag tag-warn", "RESET REFUSED");
      tag.title = "The HUD kept the current permit and alarm on screen: " + ev.hud.reset_refused;
      meta.append(tag);
    }
    if (kind && !isSay(kind) && kind !== "info" && kind !== "permit") {
      meta.append(el("span", "ev-kind", KIND_LABELS[kind] || kind.replace(/_/g, " ")));
    }
    if (data) {
      const verdict = verdictOf(data);
      // A purge-gate refusal is a hold on readings that passed, not a FAIL.
      if (data.purge_incomplete === true || kind === "purge_incomplete") meta.append(el("span", "ev-tag tag-warn", "PURGE HOLD"));
      else if (verdict !== null) meta.append(el("span", verdict ? "ev-tag tag-pass" : "ev-tag tag-fail", verdict ? "PASS" : "FAIL"));
      const permit = permitOf(data);
      if (permit) {
        li.classList.add("ev-permit", "edge-" + PERMIT[permit][1]);
        meta.append(el("span", "ev-tag tag-permit", "PERMIT → " + PERMIT[permit][0]));
      }
    }
    li.append(meta, el("div", "ev-msg", ev.message ? plainText(ev.message) : "(no message)"));
    if (isSay(kind)) li.classList.add("ev-say");
    // Red only for real alarms: the guard also logs a benign gas reading
    // every 5 s while inside, and those must not look like alarms.
    if (isAlertEvent(ev, kind, data)) li.classList.add("ev-guard");
    return li;
  }

  // Follow the log unless the presenter scrolled up to read something. Set
  // from scroll events (not re-measured per append): a layout change that
  // shrinks the feed fires no scroll event and must not stop the follow.
  let followFeed = true;
  function onFeedScroll() {
    const feed = dom.feed;
    followFeed = feed.scrollTop + feed.clientHeight >= feed.scrollHeight - 48;
    updateFeedClip();
  }

  function appendEvents(events) {
    if (!Array.isArray(events) || !events.length) return;
    const feed = dom.feed;
    for (const ev of events) {
      if (!ev || typeof ev.seq !== "number" || ev.seq <= lastEventSeq) continue;
      lastEventSeq = ev.seq;
      const li = renderEvent(ev);
      if (!li) continue;
      if (ev.hud && (ev.hud.record === "safety" || ev.hud.record === "local")) li.classList.add("ev-keep");
      feed.append(li);
    }
    trimFeed(feed);
    if (followFeed) feed.scrollTop = feed.scrollHeight;
    updateFeedClip();
  }

  // Oldest routine lines go first: a flood of chatter must not push an
  // alarm, permit or retreat line out of the log (T42).
  function trimFeed(feed) {
    let excess = feed.childElementCount - MAX_FEED;
    for (let node = feed.firstElementChild; excess > 0 && node;) {
      const next = node.nextElementSibling;
      if (!node.classList.contains("ev-keep")) {
        node.remove();
        excess--;
      }
      node = next;
    }
    while (feed.childElementCount > MAX_FEED_KEPT) feed.firstElementChild.remove();
  }

  // The newest entry is kept at the bottom, so the oldest visible one is
  // cut at the top edge: fade that edge instead of showing a sliced line.
  function updateFeedClip() {
    dom.feed.classList.toggle("is-clipped", dom.feed.scrollTop > 2);
  }

  // ----------------------------------------------------------------- alert

  let flashTimer = null;
  // Title + triggering reading + retreat note. The reason line is hidden when
  // it would only repeat the reading (gas-edge text, or reading + note known).
  function setFlashText(alert) {
    dom.flashTitle.textContent = alert.title || "ALARM";
    const hideReason = !!alert.reading && (!!alert.note || alert.source === "gas");
    dom.flashReason.textContent = hideReason ? "" : alert.reason || "";
    dom.flashReason.hidden = hideReason || !alert.reason;
    dom.flashReading.textContent = alert.reading ? "Triggering reading: " + alert.reading : "";
    dom.flashReading.hidden = !alert.reading;
    dom.flashNote.textContent = alert.note || "";
    dom.flashNote.hidden = !alert.note;
    const detail = alertDetail(alert);
    dom.flashDetail.textContent = detail;
    dom.flashDetail.hidden = !detail;
  }

  // Guard reaction time (detection -> zero-twist command, software side) and
  // whether the presenter staged this excursion: honest context for the room.
  // The staging disclosure goes first: the one-line header strip ellipsizes
  // its end, and at 1920 and 1280 wide it cut "simulated excursion staged
  // by the presenter" off after the reaction time.
  function alertDetail(alert) {
    const parts = [];
    const r = num(alert.reaction_s);
    if (alert.unverified) parts.push("reported by an unverified event source");
    if (alert.staged) parts.push("simulated excursion staged by the presenter");
    if (r !== null) parts.push("stop commanded " + fmtReaction(r) + " after detection");
    const text = parts.join(" · ");
    return text ? text.charAt(0).toUpperCase() + text.slice(1) : "";
  }
  // 0.2 ms must not read "0 ms" (it looks like a missing value): < 1 ms.
  function fmtReaction(r) {
    if (r < 0.0005) return "< 1 ms";
    if (r < 1) return Math.round(r * 1000) + " ms";
    return r.toFixed(1) + " s";
  }
  function showFlash(alert) {
    setFlashText(alert);
    dom.flash.hidden = false;
    // Restart the pulse animation for back-to-back alerts.
    dom.flash.style.animation = "none";
    void dom.flash.offsetWidth;
    dom.flash.style.animation = "";
    clearTimeout(flashTimer);
    flashTimer = setTimeout(hideFlash, FLASH_MS);
  }
  function hideFlash() {
    clearTimeout(flashTimer);
    dom.flash.hidden = true;
  }

  function checkAlert() {
    const a = model.alert;
    if (!a || typeof a.seq !== "number") return;
    const fresh = num(a.t_wall) !== null && serverNow() - a.t_wall < FLASH_MAX_AGE_S;
    if (lastAlertSeq === null || a.seq < lastAlertSeq) {   // first state, or server restarted
      lastAlertSeq = a.seq;
      if (a.seq > 0 && fresh) showFlash(a);
      return;
    }
    if (a.seq > lastAlertSeq) {
      lastAlertSeq = a.seq;
      if (fresh) showFlash(a);
    } else if (!dom.flash.hidden) {
      // Same incident, refined text (e.g. guard event after the gas edge).
      setFlashText(a);
    }
  }

  // ---------------------------------------------------------------- camera

  let camInFlight = false;
  let camShownSeq = -1;
  let camUrl = null;
  async function refreshCamera() {
    const cam = model.camera;
    if (camInFlight || !cam || !cam.seq || cam.seq === camShownSeq) return;
    camInFlight = true;
    const ctl = new AbortController();
    const timer = setTimeout(() => ctl.abort(), 3000);
    try {
      const resp = await fetch("/api/camera.jpg?seq=" + cam.seq, { cache: "no-store", signal: ctl.signal });
      if (resp.ok) {
        const url = URL.createObjectURL(await resp.blob());
        dom.cam.src = url;
        if (camUrl) URL.revokeObjectURL(camUrl);
        camUrl = url;
        camShownSeq = cam.seq;
      }
    } catch (err) {
      /* timeout or tunnel hiccup: the next state update retries */
    } finally {
      clearTimeout(timer);
      camInFlight = false;
    }
  }

  // ------------------------------------------------------------ state apply

  function afterModelUpdate() {
    const wall = model.server && num(model.server.wall_time);
    if (wall !== null) clockOffset = wall - Date.now() / 1000;
    checkAlert();
    scheduleRender();
    refreshCamera();
  }

  function applyFull(snap) {
    if (!snap || typeof snap !== "object") return;
    const { events, trail: fullTrail, ...rest } = snap;
    if (typeof rest.events_seq === "number" && rest.events_seq < lastEventSeq) {
      dom.feed.textContent = "";   // server restarted: its seq counter reset
      lastEventSeq = 0;
    }
    model = rest;
    if (Array.isArray(fullTrail)) {
      trail = fullTrail.slice(-TRAIL_MAX);
      trailEpoch = rest.trail_epoch;
    }
    appendEvents(events);
    afterModelUpdate();
  }

  function applyDiff(changes) {
    if (!changes || typeof changes !== "object") return;
    Object.assign(model, changes);
    if ("trail_epoch" in changes && changes.trail_epoch !== trailEpoch) {
      trail = [];
      trailEpoch = changes.trail_epoch;
    }
    if (changes.pose) pushTrail(changes.pose);
    afterModelUpdate();
  }

  function handleMessage(msg) {
    switch (msg && msg.type) {
      case "hello": applyFull(msg.state); break;
      case "diff": applyDiff(msg.changes); break;
      case "events": appendEvents(msg.events); break;
      default: break;  // ping
    }
  }

  // ------------------------------------------------------------ connection

  const conn = { state: "offline", ws: null, open: false, createdAt: 0, lastMsgAt: 0,
    backoff: 1000, reconnectTimer: null, lastPollOk: 0, pollInFlight: false };

  function online() {
    return conn.state === "live" || conn.state === "polling";
  }

  function setConn(state) {
    conn.state = state;
    scheduleRender();
  }

  function scheduleReconnect() {
    if (conn.reconnectTimer) return;
    conn.reconnectTimer = setTimeout(connect, conn.backoff);
    conn.backoff = Math.min(conn.backoff * 2, 5000);
  }

  // Detach first, then close: a half-open socket can take minutes to fire
  // onclose, and we must not wait for it before reconnecting.
  function dropSocket() {
    const old = conn.ws;
    conn.ws = null;
    conn.open = false;
    if (old) {
      old.onopen = old.onmessage = old.onclose = old.onerror = null;
      try { old.close(); } catch (err) { /* already closed */ }
    }
    if (Date.now() - conn.lastPollOk > 5000) setConn("offline");
    scheduleReconnect();
  }

  function connect() {
    conn.reconnectTimer = null;
    let sock;
    try {
      // The page's own origin: wss: whenever the page was served over https:.
      // Plain ws: is only for the plain-http page on localhost, which the
      // HUD serves through the (encrypted) SSM tunnel; the server accepts no
      // other Origin (T2). Scanner note: reviewed, not an insecure socket.
      const url = new URL("/ws", location.href);
      url.protocol = location.protocol === "https:" ? "wss:" : "ws:";
      sock = new WebSocket(url.href);
    } catch (err) {
      scheduleReconnect();
      return;
    }
    conn.ws = sock;
    conn.createdAt = Date.now();
    sock.onopen = () => {
      conn.open = true;
      conn.backoff = 1000;
      conn.lastMsgAt = Date.now();
      setConn("live");
    };
    sock.onmessage = (m) => {
      conn.lastMsgAt = Date.now();
      let msg;
      try { msg = JSON.parse(m.data); } catch (err) { return; }
      handleMessage(msg);
    };
    sock.onclose = sock.onerror = () => { if (conn.ws === sock) dropSocket(); };
  }

  async function pollOnce() {
    if (conn.pollInFlight) return;
    conn.pollInFlight = true;
    const ctl = new AbortController();
    const timer = setTimeout(() => ctl.abort(), 2500);
    try {
      const resp = await fetch("/api/state", { cache: "no-store", signal: ctl.signal });
      if (resp.ok) {
        const snap = await resp.json();
        conn.lastPollOk = Date.now();
        if (!conn.open) setConn("polling");
        applyFull(snap);
      }
    } catch (err) {
      if (!conn.open) setConn("offline");
    } finally {
      clearTimeout(timer);
      conn.pollInFlight = false;
    }
  }

  function watchdog() {
    const now = Date.now();
    if (conn.open && now - conn.lastMsgAt > WATCHDOG_MS) dropSocket();
    else if (conn.ws && !conn.open && now - conn.createdAt > CONNECT_TIMEOUT_MS) dropSocket();
    if (!conn.open) {
      pollOnce();  // keep the HUD alive at ~1 Hz while the socket is down
      if (conn.state === "polling" && now - conn.lastPollOk > 5000) setConn("offline");
    }
    scheduleRender();  // keeps "for m:ss" and staleness labels ticking
  }

  // ------------------------------------------------------------------ init

  buildChecklist();
  buildGauges();
  buildChips();
  dom.flash.addEventListener("click", hideFlash);
  dom.feed.addEventListener("scroll", onFeedScroll, { passive: true });
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") hideFlash(); });
  window.addEventListener("resize", scheduleRender);
  connect();
  setInterval(watchdog, 1000);
  scheduleRender();
})();
