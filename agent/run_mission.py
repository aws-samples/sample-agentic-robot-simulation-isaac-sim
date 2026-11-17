"""CLI: run the scripted confined-space entry against the Spot facade.

What it does
------------
Connects a RobotClient to /demo_spot, reads the latched site geometry
(payload/site, falling back to agent/site_defaults.json after 10 s), records
the entry permit (permit id, isolation, LOTO, attendant, rescue plan - Spot
does not approach the entry without them), and runs
ConfinedSpaceMission.run(): stand, approach, face the door, O2 -> LEL ->
H2S/CO atmospheric test with the extendable gas probe (simulated; arm-mounted
on the real robot) at six points - 0.5 and 1.0 m inside the opening, each at
three heights - entry through the door and interior waypoints, continuous
monitoring, exit, sit. The JSONL audit log is appended to --log (default
logs/mission-<utc>.jsonl under the repository, which the agent container
mounts from the host; each record carries the SHA-256 of the previous one).
A log that cannot be written stops the run before Spot moves. The chain head
(record count and SHA-256 of the last record) is printed as an ``[audit
head]`` line at mission start, after each confirmation, at mission end and
when the log is closed, so the Session Manager session log keeps a copy off
the host (T38; check with ``python3 mission.py --verify``).

    python3 run_mission.py [--storyline] [--no-auto-retreat]
                           [--inject-h2s-at-waypoint N] [--log logs/mission-<utc>.jsonl]
                           [--permit-id ID] [--attendant ROLE] [--rescue-plan TEXT]
                           [--no-isolation] [--no-loto] [--rehearsal-auto-confirm]
                           [--unattributed-run]

It does not start when the login that started the session (SUDO_USER, which
host/run_agent.sh passes as SPOT_OPERATOR_LOGIN) is a shared account
(ssm-user, ubuntu, root) or unknown: the record would not say who ran it
(T38). Connect as your own account (Session Manager Run As), then
``sudo -iu ubuntu``. --unattributed-run starts anyway and records the run as
unattributed in mission_start.

--storyline runs the demo storyline: (demo reset of the SIMULATED
atmosphere unless --no-gas-reset) -> test the unventilated tank (expected to
FAIL: entry denied, permit stays PRE_ENTRY_TESTING) -> start SIMULATED forced
ventilation (--vent-tau) -> wait for the purge (--vent-wait; the purge
target is fraction_remaining <= --purge-target, default 0.08) -> re-test (up
to --max-retests; a re-test that reads within limits before the purge target
is "purge incomplete - wait", not an authorization) -> enter -> exit ->
COMPLETED. Without it a failed test aborts the permit (the original scripted
run).

Permit fields default to clearly labelled demo values (DEMO_PERMIT) and are
written to the audit log as such. The attendant is a role label (optionally
with a roster id, e.g. "attendant 2"), never a person's name: anything else
given to --attendant stops the run before any prompt, record or event, and
is not echoed. Isolation, LOTO and the attendant are
never defaulted: the operator attests them by typing y at this terminal
(agent/operator_console.py), and entering the permit space is confirmed the
same way once the test has passed; no answer, or no terminal, refuses.
--rehearsal-auto-confirm answers both itself for automated rehearsals: it
works only once the simulator backend is identified, and every
auto-confirmation is logged as such. --no-isolation / --no-loto skip the
attestation to show the gate: the run then stops before Spot moves.

--inject-h2s-at-waypoint N makes the demo's surprise excursion reproducible:
when Spot reaches interior waypoint N (0-based index into the site's
interior_waypoints; 0 = just inside the door) the harness publishes an H2S
pocket at Spot's position on payload/gas/inject - the operator-console
stand-in - and the SafetyGuard, not this script, reacts to it.

--no-auto-retreat is a REHEARSAL mode for validating the route: alarms are
still detected and logged, but Spot keeps going. Never use it for the demo.

Exit codes: 0 = permit COMPLETED; 2 = ABORTED as designed (entry denied or
retreated safely on an atmospheric alarm, Spot outside the space); 1 = any
failure (permit incomplete, no connection, robot not ready, navigation
fault, retreat incomplete, telemetry loss, Ctrl-C).

Runs inside the ros:jazzy agent container (needs rclpy); ``execute()`` takes
an injected robot so tests drive it with a fake on a Mac.
"""

import argparse
import json
import logging
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mission import (ABORTED, ATTENDANT_ROLE, COMPLETED, ENTRY_SUPERVISOR_ROLE,  # noqa: E402
                     ConfinedSpaceMission, default_log_path, role_label)
from operator_console import (LineReader, attribution_refusal, inert_console_markers,  # noqa: E402
                              operator_identity, terminal_confirmer)
from robot_client import backend_refusal  # noqa: E402

LOG = logging.getLogger("run_mission")

EXIT_COMPLETED = 0
EXIT_FAILURE = 1
EXIT_ABORTED_AS_DESIGNED = 2

# Demo permit defaults: fictional, labelled as such in the audit log. Roles,
# never names (T37).
DEMO_PERMIT = {
    "attendant": ATTENDANT_ROLE,
    "rescue_plan": "Standby rescue team with retrieval gear at the cleanout door (demo value)",
}


def build_parser():
    parser = argparse.ArgumentParser(
        description="Scripted permit-required confined-space entry for the Spot demo.")
    parser.add_argument("--no-auto-retreat", action="store_true",
                        help="REHEARSAL ONLY: log gas alarms but do not retreat")
    parser.add_argument("--inject-h2s-at-waypoint", type=int, default=None, metavar="N",
                        help="inject an H2S excursion at Spot's position on reaching "
                             "interior waypoint N (0-based)")
    parser.add_argument("--inject-ppm", type=float, default=80.0,
                        help="peak H2S ppm of the injected pocket (default 80)")
    # 1 m like host/inject_gas.sh: a 2 m pocket near the door still read
    # ~14-19 ppm H2S at the sit standoff after the retreat (host, 2026-09-27),
    # so Spot sat down in an alarming atmosphere.
    parser.add_argument("--inject-radius", type=float, default=1.0,
                        help="1-sigma radius (m) of the injected pocket (default 1.0)")
    parser.add_argument("--inject-z", type=float, default=0.3,
                        help="pocket center height (m); H2S pools low (default 0.3)")
    parser.add_argument("--log", default=None,
                        help="JSONL audit log, appended (default logs/mission-<utc>.jsonl in the "
                             "mounted logs directory)")
    parser.add_argument("--namespace", default="/demo_spot")
    parser.add_argument("--connect-timeout", type=float, default=30.0)
    parser.add_argument("--dwell", type=float, default=4.0,
                        help="atmospheric-test dwell in seconds (default 4)")
    parser.add_argument("--storyline", action="store_true",
                        help="test (expected FAIL) -> ventilate -> wait -> re-test -> enter")
    parser.add_argument("--vent-tau", type=float, default=None, metavar="S",
                        help="ventilation time constant in seconds (default: the sim's 15 s)")
    parser.add_argument("--vent-wait", type=float, default=180.0, metavar="S",
                        help="max seconds to wait for the purge before re-testing (default 180)")
    parser.add_argument("--purge-target", type=float, default=None, metavar="F",
                        help="ventilation fraction_remaining a ventilated (or earlier-failed) "
                             "tank must reach before entry is authorized (default 0.08)")
    parser.add_argument("--max-retests", type=int, default=2,
                        help="re-tests after ventilation before entry is denied (default 2)")
    parser.add_argument("--no-gas-reset", action="store_true",
                        help="storyline: do not clear injected gas / reset the simulated tank first")
    parser.add_argument("--permit-id", default=None,
                        help="permit number (default: generated CSE-<utc>)")
    parser.add_argument("--attendant", default=DEMO_PERMIT["attendant"], metavar="ROLE",
                        help="attendant role label, optionally with a roster id such as "
                             "'attendant 2' (names are not recorded; default 'attendant')")
    parser.add_argument("--rescue-plan", default=DEMO_PERMIT["rescue_plan"])
    parser.add_argument("--no-isolation", action="store_true",
                        help="leave isolation unconfirmed (demonstrates the permit gate)")
    parser.add_argument("--no-loto", action="store_true",
                        help="leave LOTO unconfirmed (demonstrates the permit gate)")
    parser.add_argument("--rehearsal-auto-confirm", action="store_true",
                        help="AUTOMATED REHEARSAL ONLY: answer the operator attestation and the "
                             "entry confirmation with yes (simulator backend only; each one is "
                             "logged as auto-confirmed)")
    parser.add_argument("--unattributed-run", action="store_true",
                        help="start although the login that started this session is a shared "
                             "account (ssm-user, ubuntu, root) or unknown; mission_start then "
                             "records the run as unattributed (T38)")
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser


def exit_code_for(summary):
    """Map a mission summary to the CLI exit code (see module docstring)."""
    if summary.get("outcome") == COMPLETED:
        return EXIT_COMPLETED
    if summary.get("outcome") == ABORTED and summary.get("aborted_as_designed"):
        return EXIT_ABORTED_AS_DESIGNED
    return EXIT_FAILURE


def make_injector(robot, mission, args):
    """on_waypoint hook implementing --inject-h2s-at-waypoint."""
    target = args.inject_h2s_at_waypoint
    fired = []

    def on_waypoint(index, x, y):
        if index != target or fired:
            return
        fired.append(index)
        ok = robot.inject_gas("h2s", args.inject_ppm, x, y, args.inject_z, args.inject_radius)
        # Logged openly: the audit trail must show the excursion was staged.
        text = ("Demo harness: injected H2S excursion (%.0f ppm, r=%.1f m) at waypoint %d "
                "(%.2f, %.2f)%s" % (args.inject_ppm, args.inject_radius, index, x, y,
                                    "" if ok else " - PUBLISH FAILED"))
        mission.log_event("demo_inject", text,
                          {"gas": "h2s", "ppm": args.inject_ppm, "x": x, "y": y,
                           "z": args.inject_z, "radius": args.inject_radius,
                           "waypoint": index, "published": bool(ok)})

    return on_waypoint


_READER = []


def _terminal_reader():
    """The one LineReader on this terminal; EOFError without one."""
    if not _READER:
        try:
            _READER.append(LineReader(sys.stdin.fileno()))
        except (AttributeError, OSError, ValueError):
            raise EOFError from None
    return _READER[0]


def _terminal_readline(timeout=None):
    """One line from this terminal within ``timeout`` (None = no answer);
    EOFError without a terminal or at end of input."""
    return _terminal_reader().readline(timeout)


def _terminal_drain():
    """Discard type-ahead before a question (a stray "y" typed during the
    ventilation wait must not answer the entry confirmation)."""
    try:
        return _terminal_reader().drain()
    except EOFError:
        return []


def _write(text):
    sys.stdout.write(text)
    sys.stdout.flush()


def make_confirmer(mission, args, readline=None, write=None, drain=None):
    """confirm(action, summary, details) -> (ok, record) for this run: the
    operator's y/N at the terminal, or --rehearsal-auto-confirm (simulator
    only). Every decision goes to the mission JSONL."""
    ask = terminal_confirmer(readline or _terminal_readline, write or _write,
                             auto_confirm=getattr(args, "rehearsal_auto_confirm", False),
                             allow_auto=lambda: mission.simulator,
                             drain=drain if drain is not None else
                             (_terminal_drain if readline is None else None))

    def confirm(action, summary, details=None):
        mission.log_event("confirmation_requested", "Operator confirmation requested: %s"
                          % summary, {"action": action, "summary": summary})
        ok, record = ask(action, summary, details)
        auto = record.get("mode") == "rehearsal_auto_confirm"
        if auto:
            LOG.warning("REHEARSAL AUTO-CONFIRM: %s", action)
        mission.log_event("operator_confirmation", "Operator %s %s (answer %r%s)" % (
            "CONFIRMED" if ok else "did not confirm", action, record.get("answer"),
            ", rehearsal auto-confirm" if auto else ""), record)
        return ok, record

    return confirm


def _entry_text(mission, test):
    parts = ["%s %.1f %s" % (e.get("gas", "?").upper(), float(e["value"]), e.get("unit") or "")
             for e in (test or {}).get("results") or []
             if isinstance(e, dict) and isinstance(e.get("value"), (int, float))]
    return ("ENTRY INTO THE PERMIT SPACE (permit %s): Spot walks through the cleanout door and "
            "follows the %d interior waypoints. Atmospheric test: %s%s."
            % (mission.permit_id, len(mission.waypoints),
               "PASS" if (test or {}).get("passed") else "FAIL",
               (": " + ", ".join(parts)) if parts else ""))


def execute(args, robot, clock=None, readline=None, write=None):
    """Run the mission on a connected robot; returns the exit code.
    ``readline``/``write`` replace the terminal for the confirmations."""
    # Roles, never names (T37): anything but a role label is refused here,
    # before it can reach a prompt, a JSONL record or payload/events. The
    # value is not echoed.
    attendant = role_label(args.attendant, ATTENDANT_ROLE)
    if attendant is None:
        LOG.error("--attendant must be a role label such as %r or %r (a roster id), not a name; "
                  "the value given was not recorded and nothing was started",
                  ATTENDANT_ROLE, ATTENDANT_ROLE + " 2")
        return EXIT_FAILURE
    args.attendant = attendant
    site = robot.site()
    # The default log is this run's own new file, created exclusively (T38).
    new_log = args.log is None
    if args.log is None:
        args.log = default_log_path("mission")
    if not args.log:
        LOG.error("an audit log is required (--log PATH)")
        return EXIT_FAILURE
    try:
        mission = ConfinedSpaceMission(robot, site, args.log, clock=clock, dwell_s=args.dwell,
                                       require_permit=True,
                                       purge_target=getattr(args, "purge_target", None),
                                       new_log=new_log)
    except (ValueError, OSError) as error:
        LOG.error("cannot start mission: %s", error)
        return EXIT_FAILURE
    # The chain head goes to this terminal, and with it to the Session Manager
    # session log off the host (T38); attribution goes into mission_start.
    mission.on_audit_head = lambda line: (write or _write)(line + "\n")
    mission.unattributed_run = bool(getattr(args, "unattributed_run", False))
    # Simulator only (no override): nothing is commanded on any other backend.
    refusal = backend_refusal(mission.backend(), "running the mission")
    if refusal:
        LOG.error("%s", refusal)
        mission.log_event("backend_refused", refusal, {"backend": mission.backend()})
        mission.close()
        return EXIT_FAILURE
    mission.start_guard()   # the SafetyGuard and its payload/guard heartbeat from now on
    # A failed retreat is shown on the terminal at once (T17).
    mission.notify_operator = lambda text: (write or _write)("OPERATOR ATTENTION: %s\n" % text)
    demo_values = args.rescue_plan == DEMO_PERMIT["rescue_plan"]
    prepared = mission.prepare_permit(
        permit_id=args.permit_id, rescue_plan=args.rescue_plan,
        prepared_by="run_mission.py CLI" + (" (demo default values)" if demo_values else ""))
    confirm = make_confirmer(mission, args, readline, write)
    if not (args.no_isolation or args.no_loto):
        ok, record = confirm(
            "permit_attestation",
            "PERMIT ATTESTATION for permit %s. You, the operator at this terminal, attest as the "
            "%s that: energy and line isolation is in place; lockout/tagout is in place; an %s is "
            "stationed at the cleanout door. Rescue plan: %s." % (
                mission.permit_id, ENTRY_SUPERVISOR_ROLE, args.attendant, args.rescue_plan))
        if ok:
            prepared = mission.prepare_permit(
                isolation_confirmed=True, loto_confirmed=True, attendant=args.attendant,
                entry_supervisor=ENTRY_SUPERVISOR_ROLE, attestation=record,
                prepared_by="operator at the terminal (%s)" % ENTRY_SUPERVISOR_ROLE)
    if not prepared["complete"]:
        # run() refuses too (and writes the summary record); say why here.
        LOG.error("permit incomplete (missing %s): Spot will not approach the entry",
                  ", ".join(prepared["missing"]))
    n_wp = len(mission.waypoints)
    for i, (x, y) in enumerate(mission.waypoints):
        LOG.info("interior waypoint %d: (%.2f, %.2f)", i, x, y)
    hook = None
    if args.inject_h2s_at_waypoint is not None:
        if not 0 <= args.inject_h2s_at_waypoint < n_wp:
            LOG.error("--inject-h2s-at-waypoint must be in 0..%d", n_wp - 1)
            mission.close()
            return EXIT_FAILURE
        hook = make_injector(robot, mission, args)
    if args.no_auto_retreat:
        LOG.warning("AUTO-RETREAT DISABLED: rehearsal mode, alarms are logged only")
    try:
        summary = mission.run(auto_retreat=not args.no_auto_retreat, on_waypoint=hook,
                              storyline=args.storyline, vent_tau_s=args.vent_tau,
                              vent_max_s=args.vent_wait, max_retests=args.max_retests,
                              reset_atmosphere=args.storyline and not args.no_gas_reset,
                              confirm_entry=lambda test: confirm(
                                  "entry", _entry_text(mission, test),
                                  {"passed": bool(test.get("passed"))}))
    except KeyboardInterrupt:
        # Ctrl-C or SIGTERM: bring Spot out first (T16); a second interrupt
        # cancels the retreat and leaves Spot where it is.
        LOG.error("interrupted: bringing Spot out of the permit space before exiting "
                  "(interrupt again to leave it where it is)")
        try:
            if not mission.exit_safely("scripted mission interrupted"):
                LOG.error("Spot is still inside the permit space: operator attention required")
        except KeyboardInterrupt:
            mission.cancel("second interrupt: retreat cancelled")
            LOG.error("second interrupt: retreat cancelled; Spot stays where it is")
        return EXIT_FAILURE
    finally:
        mission.close()
    code = exit_code_for(summary)
    print(json.dumps({"outcome": summary["outcome"], "permit": summary["permit"],
                      "permit_id": summary.get("permit_id"), "reason": summary["reason"],
                      "storyline": bool(args.storyline),
                      "tests": [t["passed"] for t in summary.get("atmospheric_tests") or []],
                      "exit_code": code, "log": args.log}), flush=True)
    return code


def _raise_keyboard_interrupt(_signum, _frame):
    # SIGTERM (docker stop) takes the same safe path as Ctrl-C: unwind through
    # the motion loops' finally blocks (zero twist + stop), then cancel.
    raise KeyboardInterrupt


def main(argv=None):
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    # Event text in log lines cannot print a console marker or a chain head (T38).
    inert_console_markers()
    # SIGTERM (docker stop), a hang-up (SSM session lost) and Ctrl-C (even if
    # a launcher left SIGINT ignored) all take the safe path: bring Spot out.
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, _raise_keyboard_interrupt)
    # No unattributed record without saying so (T38): refused before anything
    # connects, unless the operator passed --unattributed-run.
    refusal = attribution_refusal(operator_identity(), args.unattributed_run, "run_mission.py")
    if refusal:
        LOG.error("%s", refusal)
        return EXIT_FAILURE
    if args.unattributed_run:
        LOG.warning("UNATTRIBUTED RUN (--unattributed-run): mission_start records that this run "
                    "does not name its operator")

    from robot_client import RobotClient  # rclpy is imported lazily in connect()

    robot = RobotClient(namespace=args.namespace)
    try:
        if not robot.connect(timeout=args.connect_timeout):
            LOG.error("could not connect to %s within %.0f s", args.namespace,
                      args.connect_timeout)
            return EXIT_FAILURE
        robot.wait_for_backend(timeout=min(args.connect_timeout, 10.0))
        return execute(args, robot)
    except KeyboardInterrupt:
        LOG.error("interrupted before the mission started")
        return EXIT_FAILURE
    finally:
        try:
            robot.send_velocity(0.0, 0.0, 0.0)
            robot.stop()
        except Exception:
            pass
        robot.close()


if __name__ == "__main__":
    sys.exit(main())
