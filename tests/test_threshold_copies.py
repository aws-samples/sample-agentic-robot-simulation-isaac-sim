"""The copies of the OSHA thresholds and the site layout stay equal (T33, M33).

Each container ships only its own package, so the OSHA 1910.146 Appendix E
limits and the demo site layout are copied on purpose:

* thresholds: sim/gas_field.py (the simulator's alarm flags, the source),
  agent/mission.py (the SafetyGuard, the entry gate and their text),
  agent/spot_agent.py (the agent's second check on the values),
  hud/ros_bridge.py (the HUD's threshold lines and its own alarm check) and
  hud/static/app.js (the page's fallback until the first snapshot);
* site layout: sim/build_scene.py (the source), agent/site_defaults.json
  (what the mission and the agent load), hud/ros_bridge.py BUILTIN_SITE,
  the tank and doorway defaults in sim/gas_field.py, and the doorway width
  in hud/static/app.js.

A pull request that changes one copy, for example the H2S alarm from 10 to
100 ppm in one module, fails here unless it changes every copy (T33's
"plausible PR"; review still has to catch a coordinated change). Constants
are read from the source text with ``ast``, so nothing that needs Isaac,
pxr, rclpy or the agent's packages is imported; the boundary tests import
only the pure modules.

Run from the repo root:

    python3 -m unittest tests.test_threshold_copies -v
"""

import ast
import importlib
import json
import math
import operator
import os
import re
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _path(*parts):
    return os.path.join(REPO, *parts)


def _read(*parts):
    with open(_path(*parts), encoding="utf-8") as handle:
        return handle.read()


_OPS = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
        ast.Div: operator.truediv}


def _value(node, env):
    """A constant expression: literals, names already in ``env``, math.pi,
    + - * / and unary minus. Anything else raises ValueError."""
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name):
        if node.id in env:
            return env[node.id]
        raise ValueError(node.id)
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
            and node.value.id == "math" and node.attr == "pi":
        return math.pi
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        value = _value(node.operand, env)
        return -value if isinstance(node.op, ast.USub) else value
    if isinstance(node, ast.BinOp) and type(node.op) in _OPS:
        return _OPS[type(node.op)](_value(node.left, env), _value(node.right, env))
    if isinstance(node, (ast.Tuple, ast.List)):
        return [_value(item, env) for item in node.elts]
    if isinstance(node, ast.Dict):
        return {_value(k, env): _value(v, env) for k, v in zip(node.keys, node.values)}
    raise ValueError(ast.dump(node))


def _constants(*parts):
    """Module-level ``NAME = <constant>`` and ``A, B = x, y`` assignments."""
    tree = ast.parse(_read(*parts))
    env = {}
    for node in tree.body:
        if not (isinstance(node, ast.Assign) and len(node.targets) == 1):
            continue
        target = node.targets[0]
        try:
            if isinstance(target, ast.Name):
                env[target.id] = _value(node.value, env)
            elif isinstance(target, ast.Tuple) and isinstance(node.value, ast.Tuple):
                for name, item in zip(target.elts, node.value.elts):
                    env[name.id] = _value(item, env)
        except (ValueError, TypeError, ZeroDivisionError, AttributeError):
            pass
    return env, tree


def _defaults(tree, cls, func="__init__"):
    """``{arg: default}`` of ``cls.func`` (constant defaults only)."""
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
    fn = next(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == func)
    args = fn.args.args[len(fn.args.args) - len(fn.args.defaults):]
    out = {}
    for arg, default in zip(args, fn.args.defaults):
        try:
            out[arg.arg] = _value(default, {})
        except ValueError:
            pass
    return out


def _scene_site():
    """sim/build_scene.py build_scene()'s returned layout, evaluated from its
    constant expressions (the scene code itself is never run)."""
    env, tree = _constants("sim", "build_scene.py")
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "build_scene")
    ret = [n for n in ast.walk(fn) if isinstance(n, ast.Return) and isinstance(n.value, ast.Dict)][-1]
    site = {}
    for key, value in zip(ret.value.keys, ret.value.values):
        try:
            site[key.value] = _value(value, env)
        except ValueError:
            pass   # camera_keepouts (a comprehension): GUI only, not a copy
    site["door_xy"] = site.pop("doorway_xy")
    return site, env


def _js_object(js, name):
    """``{key: {field: number}}`` from ``const NAME = { k: { f: n, ... }, ... };``."""
    body = re.search(r"const %s = \{(.*?)\};" % name, js, re.S).group(1)
    return {key: {f: float(v) for f, v in re.findall(r"(\w+): ([0-9.]+)", fields)}
            for key, fields in re.findall(r"(\w+): \{([^}]*)\}", body)}


def _numbers(text):
    return [float(n) for n in re.findall(r"\d+(?:\.\d+)?", text)]


class TestT33ThresholdCopies(unittest.TestCase):
    """T33 (M33): every copy of the OSHA App. E thresholds equals the
    simulator's (sim/gas_field.py)."""

    @classmethod
    def setUpClass(cls):
        sim, _tree = _constants("sim", "gas_field.py")
        cls.limits = {"o2_low": sim["O2_LOW_PCT"], "o2_high": sim["O2_HIGH_PCT"],
                      "lel": sim["LEL_ALARM_PCT"], "h2s": sim["H2S_ALARM_PPM"],
                      "co": sim["CO_ALARM_PPM"]}

    def test_the_source_values_are_osha_appendix_e(self):
        """T33: the simulator's copy itself is still OSHA 1910.146 App. E."""
        self.assertEqual(self.limits, {"o2_low": 19.5, "o2_high": 23.5, "lel": 10.0,
                                       "h2s": 10.0, "co": 35.0})

    def test_the_mission_copy(self):
        """T33: agent/mission.py (SafetyGuard, entry gate) equals the simulator's."""
        mission, _tree = _constants("agent", "mission.py")
        self.assertEqual({"o2_low": mission["O2_LOW_PCT"], "o2_high": mission["O2_HIGH_PCT"],
                          "lel": mission["LEL_ALARM_PCT"], "h2s": mission["H2S_ALARM_PPM"],
                          "co": mission["CO_ALARM_PPM"]}, self.limits)

    def test_the_mission_text_names_the_same_limits(self):
        """T33: the limits the mission writes into its records and speaks
        (GAS_LIMITS, _SPOKEN_LIMIT) carry the same numbers."""
        mission, _tree = _constants("agent", "mission.py")
        expect = {"o2": [self.limits["o2_low"], self.limits["o2_high"]],
                  "lel": [self.limits["lel"]], "h2s": [self.limits["h2s"]],
                  "co": [self.limits["co"]]}
        for table in ("GAS_LIMITS", "_SPOKEN_LIMIT"):
            self.assertEqual({gas: _numbers(text) for gas, text in mission[table].items()},
                             expect, table)

    def test_the_prompt_states_the_same_limits(self):
        """T33: the system prompt's acceptance sentence (agent/prompts.py), which the
        model repeats to the audience, carries the simulator's numbers."""
        with open(os.path.join(REPO, "agent", "prompts.py"), encoding="utf-8") as handle:
            text = " ".join(handle.read().replace("\\\n", " ").split())
        found = re.search(r"Acceptable: oxygen ([0-9.]+) to ([0-9.]+) percent, LEL below ([0-9.]+) "
                          r"percent, H2S below ([0-9.]+) ppm, CO below ([0-9.]+) ppm", text)
        self.assertIsNotNone(found, "the prompt's acceptance sentence changed shape")
        self.assertEqual(dict(zip(("o2_low", "o2_high", "lel", "h2s", "co"), map(float, found.groups()))),
                         self.limits)

    def test_the_agent_copy(self):
        """T33: agent/spot_agent.py's second check (the copy the threat
        model found untested) equals the simulator's."""
        agent, _tree = _constants("agent", "spot_agent.py")
        self.assertEqual({"o2_low": agent["O2_LOW_PCT"], "o2_high": agent["O2_HIGH_PCT"],
                          "lel": agent["LEL_LIMIT_PCT"], "h2s": agent["H2S_LIMIT_PPM"],
                          "co": agent["CO_LIMIT_PPM"]}, self.limits)

    def test_the_hud_copies(self):
        """T33: hud/ros_bridge.py OSHA_THRESHOLDS and the page's fallback in
        hud/static/app.js equal the simulator's."""
        hud, _tree = _constants("hud", "ros_bridge.py")
        th = hud["OSHA_THRESHOLDS"]
        self.assertEqual({"o2_low": th["o2"]["low"], "o2_high": th["o2"]["high"],
                          "lel": th["lel"]["alarm_at"], "h2s": th["h2s"]["alarm_at"],
                          "co": th["co"]["alarm_at"]}, self.limits)
        page = _js_object(_read("hud", "static", "app.js"), "FALLBACK_THRESHOLDS")
        self.assertEqual({"o2_low": page["o2"]["low"], "o2_high": page["o2"]["high"],
                          "lel": page["lel"]["alarm_at"], "h2s": page["h2s"]["alarm_at"],
                          "co": page["co"]["alarm_at"]}, self.limits)

    def test_every_copy_draws_the_line_at_the_same_boundary(self):
        """T33: at each limit and just either side of it, the simulator's alarm
        flags, the mission's verdict and the HUD's check agree (O2 at 19.5 or
        23.5 % passes; LEL, H2S and CO at the limit alarm)."""
        for directory in ("sim", "agent"):
            if _path(directory) not in sys.path:
                sys.path.insert(0, _path(directory))
        try:
            gas_field = importlib.import_module("gas_field")
        except ImportError as error:   # numpy missing
            self.skipTest("sim/gas_field.py not importable: %s" % error)
        mission = importlib.import_module("mission")
        from hud import ros_bridge
        base = {"o2_pct": 20.9, "lel_pct": 0.0, "h2s_ppm": 0.0, "co_ppm": 0.0}
        points = {"o2": (self.limits["o2_low"], self.limits["o2_high"]),
                  "lel": (self.limits["lel"],), "h2s": (self.limits["h2s"],),
                  "co": (self.limits["co"],)}
        for gas, limits in points.items():
            key = mission.GAS_KEYS[gas]
            for limit in limits:
                for value in (limit - 0.01, limit, limit + 0.01):
                    sample = dict(base, **{key: value})
                    sim = gas_field.GasField.alarms(sample)[gas]
                    self.assertEqual(not mission.gas_passes(gas, value), sim, (gas, value))
                    self.assertEqual(ros_bridge.gas_exceeds(gas, value), sim, (gas, value))
                    self.assertEqual(ros_bridge.compute_alarms(sample)[gas], sim, (gas, value))
        # The mission's exceedance text names the same O2 limits.
        self.assertIn("below %g" % self.limits["o2_low"], mission._exceedance_text("o2", 10.0))
        self.assertIn("above %g" % self.limits["o2_high"], mission._exceedance_text("o2", 30.0))


class TestT33SiteLayoutCopies(unittest.TestCase):
    """T33 (M33): every copy of the site layout equals sim/build_scene.py's."""

    @classmethod
    def setUpClass(cls):
        cls.scene, cls.scene_consts = _scene_site()
        mission, _tree = _constants("agent", "mission.py")
        cls.keys = tuple(mission["SITE_KEYS"])

    def assertLayoutEqual(self, site, label):
        self.assertEqual(set(site), set(self.keys), label)
        for key in self.keys:
            got, want = site[key], self.scene[key]
            if key == "interior_waypoints":
                self.assertEqual(len(got), len(want), (label, key))
                for a, b in zip(got, want):
                    self.assertEqual([round(float(v), 9) for v in a],
                                     [round(float(v), 9) for v in b], (label, key))
            elif isinstance(want, list):
                self.assertEqual([round(float(v), 9) for v in got],
                                 [round(float(v), 9) for v in want], (label, key))
            else:
                self.assertAlmostEqual(float(got), float(want), places=9, msg=(label, key))

    def test_the_mission_reads_every_layout_key(self):
        """T33: agent/mission.py's SITE_KEYS cover the layout build_scene publishes."""
        self.assertTrue(set(self.keys) <= set(self.scene), set(self.keys) - set(self.scene))

    def test_the_agents_site_defaults(self):
        """T33: agent/site_defaults.json (loaded by the mission and the agent)."""
        with open(_path("agent", "site_defaults.json"), encoding="utf-8") as handle:
            self.assertLayoutEqual(json.load(handle), "agent/site_defaults.json")

    def test_the_huds_builtin_site(self):
        """T33: hud/ros_bridge.py BUILTIN_SITE (the HUD's last-resort fallback)."""
        hud, _tree = _constants("hud", "ros_bridge.py")
        self.assertLayoutEqual(hud["BUILTIN_SITE"], "hud/ros_bridge.py BUILTIN_SITE")

    def test_the_simulators_tank_and_doorway_defaults(self):
        """T33: sim/gas_field.py's GasField and ProbeGeometry defaults (the
        probe's reach-through-the-doorway rule) and its door size."""
        consts, tree = _constants("sim", "gas_field.py")
        field = _defaults(tree, "GasField")
        self.assertEqual((list(field["center"]), field["radius"]),
                         (self.scene["tank_center_xy"], self.scene["tank_radius"]))
        probe = _defaults(tree, "ProbeGeometry")
        self.assertEqual(list(probe["tank_center"]), self.scene["tank_center_xy"])
        self.assertEqual(probe["tank_radius"], self.scene["tank_radius"])
        self.assertEqual(list(probe["door_xy"]), self.scene["door_xy"])
        self.assertAlmostEqual(probe["door_normal_yaw"], self.scene["door_normal_yaw"], places=9)
        self.assertEqual((consts["DOOR_WIDTH_M"], consts["DOOR_HEIGHT_M"]),
                         (self.scene_consts["DOOR_WIDTH"], self.scene_consts["DOOR_HEIGHT"]))

    def test_the_pages_doorway_width(self):
        """T33: hud/static/app.js draws the doorway at build_scene's DOOR_WIDTH."""
        width = re.search(r"const DOOR_WIDTH_M = ([0-9.]+);", _read("hud", "static", "app.js"))
        self.assertEqual(float(width.group(1)), self.scene_consts["DOOR_WIDTH"])


if __name__ == "__main__":
    unittest.main()
