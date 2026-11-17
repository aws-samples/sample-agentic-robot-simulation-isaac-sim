"""The command line of sim/isaac_app.py (pure stdlib: nothing here imports Isaac).

isaac_app.py builds its parser here, before SimulationApp starts, and parses
with ``parse_known_args`` so that an unknown argument is refused by
``facade_guard.unknown_args_error`` instead of reaching Kit (T44). The options,
defaults and help are documented in isaac_app.py's module docstring;
host/run_sim.sh's allowlist of them is kept equal to this parser by
tests/test_hardening_host.py, and tests/test_hardening_sim.py imports it::

    python3 -m unittest tests.test_hardening_sim
"""

import argparse


def build_parser():
    """The argument parser for sim/isaac_app.py, every option with its default and help."""
    parser = argparse.ArgumentParser(description="Spot confined-space demo — Isaac Sim 6.1 entrypoint")
    parser.add_argument("--headless", type=str, default="true", choices=("true", "false"),
                        help="Run without a viewport window (default: true)")
    parser.add_argument("--smoke", action="store_true",
                        help="Run ~600 physics steps, print odom + gas samples, exit 0")
    parser.add_argument("--cameras", type=str, default="frontleft,frontright",
                        help='Body cameras to publish: "all", "none" or a comma list of '
                             "frontleft,frontright,left,right,back")
    parser.add_argument("--no-cameras", action="store_true", help="Alias for --cameras none")
    parser.add_argument("--camera-hz", type=float, default=10.0,
                        help="Per-camera publish rate (Hz)")
    parser.add_argument("--scene", type=str, default=None, help="Optional USD environment override")
    parser.add_argument("--device", type=str, default="cpu", choices=("cpu", "cuda"),
                        help="PhysX device. cpu is faster for a single robot (no per-step GPU sync)")
    parser.add_argument("--render-hz", type=float, default=None,
                        help="Stage render rate in sim-time Hz (default 10)")
    parser.add_argument("--fabric", type=str, default="true", choices=("true", "false"),
                        help="PhysX->Fabric instead of USD write-back (large speedup)")
    parser.add_argument("--realtime", type=str, default="true", choices=("true", "false"),
                        help="Pace the sim to the wall clock (false = run as fast as possible)")
    parser.add_argument("--actuators", type=str, default="fast", choices=("newton", "fast"),
                        help="newton = stock explicit Newton actuators (12 per-DOF models, "
                             "~60 warp launches per physics step); fast = same gains/limits "
                             "on the PhysX joint drives (no per-step Python)")
    parser.add_argument("--gas-initial", type=str, default="raw", choices=("raw", "ventilated"),
                        help="raw = unventilated tank (storyline start); ventilated = start at the "
                             "forced-ventilation residual")
    parser.add_argument("--ui", type=str, default="presentation", choices=("presentation", "editor"),
                        help="GUI layout: presentation = fullscreen viewport only; editor = Isaac editor")
    parser.add_argument("--view", type=str, default="auto",
                        choices=("auto", "chase", "interior", "overhead", "door"),
                        help="Initial presenter camera for the GUI viewport")
    parser.add_argument("--gui-res", type=str, default="1280x720",
                        help="GUI viewport render resolution WxH (independent of the window size)")
    parser.add_argument("--battery-drain", type=str, default="demo", choices=("demo", "real", "off"),
                        help="Simulated battery: demo 25 %%/h floor 30 %%; real ~90 min; off = 100 %%")
    return parser
