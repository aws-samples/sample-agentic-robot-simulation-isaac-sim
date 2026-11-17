"""End-to-end ROS 2 check of the Isaac Sim Spot facade, acting like the agent.

Runs inside a ros:jazzy container on the sim host (see host/e2e_ros_test.sh) while
sim/isaac_app.py is running, on the test DDS domain host/e2e_ros_test.sh sets
(SPOT_TEST_DOMAIN_ID, default 43), so it reaches only the simulator that test
started. It talks to the spot_ros2-named interface under /demo_spot and drives
the simulated robot directly: it is a test of the simulator, never to be pointed
at a real robot (the sample supports the simulator only; see README.md).

Exit code 0 = every check passed; each check prints PASS/FAIL with evidence.
After the original 16 checks it adds: stand-up displacement (first stand),
three sit -> stand -> walk 1 m -> sit re-stand cycles (no fall: odometry z
> 0.4 m while standing; stand-up displacement < 0.15 m, yaw change < 0.15
rad), and the simulated gas controls (payload/gas ventilation block,
payload/gas/clear, payload/gas/probe round-trip, forced ventilation decay
and reset). Probe rules (extendable gas probe, simulated; arm-mounted on the
real robot): payload/site advertises them; from the door mouth the 2 depths
x 3 heights storyline points inside the opening are accepted; points beyond
1.1 m from the body's front and points behind the shell ("occluded by tank
wall") are rejected with an error and no samples.
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

NS = "/demo_spot"
RESULTS = []
STANDUP_MAX_DISP_M = 0.15
STANDUP_MAX_YAW_RAD = 0.15
UPRIGHT_MIN_Z_M = 0.40
# Storyline probe: ~0.5 m inside the cleanout door (wall line x = 6, door
# faces +X), at 3 heights; DOOR_PROBE_6 adds the 1.0 m-deep row. The facade
# only accepts points within 1.1 m (horizontal) of the body's front (nose,
# 0.55 m ahead of the body center) whose line from the body crosses the
# shell inside the 1.3 x 1.3 m doorway, so Spot stands at the door mouth.
DOOR_PROBE = [[5.5, 0.0, 0.2], [5.5, 0.0, 0.6], [5.5, 0.0, 1.0]]
DOOR_PROBE_6 = [[round(6.0 - d, 3), 0.0, z] for d in (0.5, 1.0) for z in (0.2, 0.6, 1.0)]
DOOR_MOUTH_XY = (6.58, 0.0)   # body center: nose ~at the door plane (x = 6)
DOOR_PRE_XY = (7.5, 0.0)      # on the door axis: line up (face -X) here, then walk straight in
FACE_TANK_YAW = math.pi       # door normal is +X, so face -X
PROBE_REACH_M = 1.1
BODY_FRONT_M = 0.55
# Inside the tank just past a jamb (r 5.87 m), behind the shell; y is
# mirrored to the nose's side so it stays in reach (tests/test_gas_field.py).
BEHIND_SHELL = [5.8, 0.9, 0.3]
VENT_KEYS = ("on", "t_on", "tau_s", "fraction_remaining")


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def check(name, ok, detail):
    RESULTS.append(ok)
    print(f"{'PASS' if ok else 'FAIL'} {name}: {detail}", flush=True)


class Client:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("e2e_client")
        self.odom = None
        self.odom_rx = []  # (wall_time, stamp_seconds)
        self.z_track = None  # list of odometry z while tracking, else None
        self.probe_results = {}
        self.site = None
        self.gas = None
        self.status = None
        self.node.create_subscription(Odometry, f"{NS}/odometry", self._on_odom, 10)
        self.node.create_subscription(String, f"{NS}/payload/gas", self._on_gas, 10)
        self.node.create_subscription(String, f"{NS}/payload/status", self._on_status, 10)
        self.pub_cmd = self.node.create_publisher(Twist, f"{NS}/cmd_vel", 10)
        self.pub_inject = self.node.create_publisher(String, f"{NS}/payload/gas/inject", 10)
        self.pub_clear = self.node.create_publisher(String, f"{NS}/payload/gas/clear", 10)
        self.pub_vent = self.node.create_publisher(String, f"{NS}/payload/gas/ventilation", 10)
        self.pub_probe = self.node.create_publisher(String, f"{NS}/payload/gas/probe", 10)
        self.node.create_subscription(String, f"{NS}/payload/gas/probe_result", self._on_probe, 10)
        site_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, f"{NS}/payload/site", self._on_site, site_qos)

    def _on_site(self, msg):
        self.site = json.loads(msg.data)

    def _on_odom(self, msg):
        self.odom = msg
        if self.z_track is not None:
            self.z_track.append(msg.pose.pose.position.z)
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_rx.append((time.monotonic(), st))

    def _on_gas(self, msg):
        self.gas = json.loads(msg.data)

    def _on_status(self, msg):
        self.status = json.loads(msg.data)

    def _on_probe(self, msg):
        d = json.loads(msg.data)
        self.probe_results[d.get("id")] = d

    def yaw(self):
        q = self.odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def wait_for(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            self.spin_for(0.1)
        return bool(pred())

    def publish_json(self, pub, obj, timeout=5.0):
        self.wait_for(lambda: pub.get_subscription_count() > 0, timeout)
        pub.publish(String(data=json.dumps(obj)))

    def probe(self, req_id, points, timeout=5.0):
        self.probe_results.pop(req_id, None)
        self.publish_json(self.pub_probe, {"id": req_id, "points": points})
        self.wait_for(lambda: req_id in self.probe_results, timeout)
        return self.probe_results.get(req_id)

    def stand_and_wait(self, timeout=8.0):
        """stand service, then wait for the policy handover (status)."""
        r = self.call("stand")
        ok = bool(r and r.success) and self.wait_for(
            lambda: bool(self.status and self.status.get("standing")
                         and not self.status.get("standing_up")), timeout)
        return ok, (r.message if r else "no response/timeout")

    def walk_toward(self, gx, gy, stop_dist, timeout=25.0):
        """Heading-controlled walk until within stop_dist of (gx, gy)."""
        end = time.monotonic() + timeout
        msg = Twist()
        while time.monotonic() < end:
            x, y, _ = self.pose()
            dist = math.hypot(gx - x, gy - y)
            if dist <= stop_dist:
                break
            err = wrap(math.atan2(gy - y, gx - x) - self.yaw())
            msg.angular.z = max(-0.8, min(0.8, 1.5 * err))
            msg.linear.x = 0.4 if abs(err) < 0.5 else 0.0
            self.pub_cmd.publish(msg)
            self.spin_for(0.05)
        self.spin_for(1.0)  # watchdog stops Spot
        x, y, _ = self.pose()
        return math.hypot(gx - x, gy - y)

    def approach_x(self, x_goal, timeout=20.0, speed=0.3):
        """Walk toward -X along the door axis (y = 0), holding heading pi with
        a small lateral correction, until body x <= x_goal. A goal-bearing
        walk swings the heading in the last decimeters (the bearing to a
        close goal is ill-conditioned). Returns the final x."""
        end = time.monotonic() + timeout
        msg = Twist()
        while time.monotonic() < end:
            x, y, _ = self.pose()
            if x <= x_goal:
                break
            want = math.pi + max(-0.2, min(0.2, 1.0 * y))
            msg.angular.z = max(-0.6, min(0.6, 1.5 * wrap(want - self.yaw())))
            msg.linear.x = speed
            self.pub_cmd.publish(msg)
            self.spin_for(0.05)
        self.spin_for(1.0)  # watchdog stops Spot
        return self.pose()[0]

    def turn_to(self, goal_yaw, tol=0.05, timeout=20.0):
        """Turn in place to goal_yaw (rad); returns the remaining error."""
        end = time.monotonic() + timeout
        msg = Twist()
        while time.monotonic() < end:
            err = wrap(goal_yaw - self.yaw())
            if abs(err) < tol:
                break
            wz = max(-0.8, min(0.8, 1.5 * err))
            msg.angular.z = math.copysign(max(abs(wz), 0.3), wz)  # the policy barely turns below ~0.25
            self.pub_cmd.publish(msg)
            self.spin_for(0.05)
        self.spin_for(1.0)  # watchdog stops Spot
        return wrap(goal_yaw - self.yaw())

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def call(self, name, timeout=20.0):
        cli = self.node.create_client(Trigger, f"{NS}/{name}")
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(Trigger.Request())
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def pose(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z

    def nose(self):
        """(x, y) of the body's front (BODY_FRONT_M ahead of the center)."""
        x, y, _ = self.pose()
        yaw = self.yaw()
        return x + BODY_FRONT_M * math.cos(yaw), y + BODY_FRONT_M * math.sin(yaw)

    def drive(self, vx, wz, seconds, hz=20.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub_cmd.publish(msg)
            self.spin_for(1.0 / hz)


def main():
    c = Client()

    # 1. Discovery: odometry flowing within 120 s of sim start.
    t0 = time.monotonic()
    while c.odom is None and time.monotonic() - t0 < 120:
        c.spin_for(0.5)
    check("odometry discovered", c.odom is not None, f"after {time.monotonic() - t0:.1f}s")
    if c.odom is None:
        return 1
    # The simulator, and only it, says so in payload/status ("simulated": true).
    # Before any command: never drive a backend that does not (T43).
    t1 = time.monotonic()
    while c.status is None and time.monotonic() - t1 < 15:
        c.spin_for(0.5)
    simulated = bool(c.status and c.status.get("simulated") is True)
    check("backend is the simulator", simulated,
          "payload/status simulated=true" if simulated else "no payload/status with simulated=true; "
          "nothing sent")
    if not simulated:
        return 1

    # 2. Real-time factor: payload/status carries the sim clock (odometry is
    #    stamped with the ROS wall clock, like the real driver, so it can't
    #    measure sim speed). Sample sim_time over ~10 s of wall time.
    t_end = time.monotonic() + 15
    while (c.status is None or "sim_time" not in c.status) and time.monotonic() < t_end:
        c.spin_for(0.25)
    if c.status and "sim_time" in c.status:
        w0, s0 = time.monotonic(), c.status["sim_time"]
        c.odom_rx.clear()
        c.spin_for(10.0)
        w1, s1 = time.monotonic(), c.status["sim_time"]
        rtf = (s1 - s0) / max(w1 - w0, 1e-6)
        hz = len(c.odom_rx) / max(w1 - w0, 1e-6)
        check("real-time factor >= 0.8", rtf >= 0.8, f"RTF={rtf:.2f}, odom {hz:.1f} Hz wall")
    else:
        check("real-time factor >= 0.8", False, "payload/status has no sim_time field")

    # 3. Lease/power/stand state machine via spot_ros2 service names.
    sx, sy, _ = c.pose()
    syaw = c.yaw()
    for svc in ("claim", "power_on", "stand"):
        r = c.call(svc)
        check(f"service {svc}", bool(r and r.success), (r.message if r else "no response/timeout"))
    c.spin_for(3.0)
    fx, fy, fz = c.pose()
    disp, dyaw = math.hypot(fx - sx, fy - sy), wrap(c.yaw() - syaw)
    check("first stand-up displacement", disp < STANDUP_MAX_DISP_M and abs(dyaw) < STANDUP_MAX_YAW_RAD,
          f"moved {disp:.3f} m, yaw change {dyaw:+.3f} rad, z={fz:.2f} "
          f"(limits {STANDUP_MAX_DISP_M} m / {STANDUP_MAX_YAW_RAD} rad)")

    # 4. Walk: 0.5 m/s forward for 6 s of wall time; expect real displacement.
    x0, y0, _ = c.pose()
    c.drive(0.5, 0.0, 6.0)
    c.spin_for(1.0)
    x1, y1, z1 = c.pose()
    moved = math.hypot(x1 - x0, y1 - y0)
    check("cmd_vel walk moves Spot", moved > 0.3, f"moved {moved:.2f} m, body z={z1:.2f}")
    check("Spot still upright after walk", z1 > 0.35, f"z={z1:.2f}")

    # 5. Watchdog: after commands stop, Spot must stop (no drift > 0.1 m over 3 s).
    xa, ya, _ = c.pose()
    c.spin_for(3.0)
    xb, yb, _ = c.pose()
    drift = math.hypot(xb - xa, yb - ya)
    check("cmd_vel watchdog stops Spot", drift < 0.15, f"drift {drift:.2f} m after commands stopped")

    # 6. Gas: clean baseline, then injected H2S spike at Spot's position -> alarm.
    c.spin_for(1.0)
    base = c.gas or {}
    check("gas baseline clean", base.get("alarm") is False, json.dumps(base)[:160])
    px, py, _ = c.pose()
    c.pub_inject.publish(String(data=json.dumps(
        {"gas": "h2s", "ppm": 80.0, "x": px, "y": py, "z": 0.3, "radius": 2.0})))
    alarm = False
    end = time.monotonic() + 10
    while time.monotonic() < end and not alarm:
        c.spin_for(0.25)
        alarm = bool(c.gas and c.gas.get("alarm") and c.gas.get("alarms", {}).get("h2s"))
    check("injected H2S spike raises alarm", alarm, json.dumps(c.gas)[:200])

    # 7. E-stop: hard stop refuses motion; release + power + stand restores.
    r = c.call("estop/hard")
    check("service estop/hard", bool(r and r.success), (r.message if r else "timeout"))
    c.spin_for(3.0)  # let the unpowered body settle before measuring motion
    xa, ya, _ = c.pose()
    c.drive(0.5, 0.0, 3.0)
    xb, yb, _ = c.pose()
    check("cmd_vel ignored while e-stopped", math.hypot(xb - xa, yb - ya) < 0.15,
          f"moved {math.hypot(xb - xa, yb - ya):.2f} m, status={c.status}")
    for svc in ("estop/release", "power_on", "stand"):
        r = c.call(svc)
        check(f"recover: {svc}", bool(r and r.success), (r.message if r else "timeout"))

    # 8. Sit, then report.
    r = c.call("sit")
    check("service sit", bool(r and r.success), (r.message if r else "timeout"))
    c.spin_for(3.0)

    # 9. Re-stand cycles (F1): sit -> stand -> hold -> walk 1 m -> sit, x3.
    for i in (1, 2, 3):
        x0, y0, _ = c.pose()
        yaw0 = c.yaw()
        ok, msg = c.stand_and_wait()
        c.z_track = []  # track z only once the policy has taken over
        check(f"re-stand {i}: stand reaches policy handover", ok, f"{msg}; status={c.status}")
        c.spin_for(2.5)
        x1, y1, z1 = c.pose()
        disp, dyaw = math.hypot(x1 - x0, y1 - y0), wrap(c.yaw() - yaw0)
        check(f"re-stand {i}: stand-up displacement",
              disp < STANDUP_MAX_DISP_M and abs(dyaw) < STANDUP_MAX_YAW_RAD,
              f"moved {disp:.3f} m, yaw change {dyaw:+.3f} rad, z={z1:.2f}")
        c.drive(0.5, 0.0, 2.0)
        c.spin_for(1.0)
        x2, y2, _ = c.pose()
        walked = math.hypot(x2 - x1, y2 - y1)
        zs, c.z_track = c.z_track, None
        zmin = min(zs) if zs else float("nan")
        check(f"re-stand {i}: upright while standing + walking",
              bool(zs) and zmin > UPRIGHT_MIN_Z_M,
              f"min odometry z {zmin:.3f} m over {len(zs)} samples (limit > {UPRIGHT_MIN_Z_M})")
        check(f"re-stand {i}: walk ~1 m", walked > 0.5, f"walked {walked:.2f} m")
        r = c.call("sit")
        check(f"re-stand {i}: sit", bool(r and r.success), (r.message if r else "timeout"))
        c.spin_for(2.5)

    # 10. Simulated gas controls: ventilation block, clear, probe, ventilation.
    g = c.gas or {}
    vent = g.get("ventilation") if isinstance(g.get("ventilation"), dict) else {}
    check("payload/gas carries ventilation block",
          all(k in vent for k in VENT_KEYS) and vent.get("on") is False
          and vent.get("fraction_remaining") == 1.0 and isinstance(g.get("injected_pockets"), int)
          and all(k in g for k in ("o2_pct", "lel_pct", "h2s_ppm", "co_ppm", "alarms", "alarm")),
          json.dumps(g)[:300])
    px, py, _ = c.pose()
    c.publish_json(c.pub_inject, {"gas": "h2s", "ppm": 80.0, "x": px, "y": py, "z": 0.3, "radius": 1.0})
    alarmed = c.wait_for(lambda: bool(c.gas and c.gas.get("alarms", {}).get("h2s")), 5.0)
    c.publish_json(c.pub_clear, {})
    cleared = c.wait_for(lambda: bool(c.gas and c.gas.get("injected_pockets") == 0
                                      and c.gas.get("alarm") is False), 5.0)
    check("payload/gas/clear removes injected pockets", alarmed and cleared,
          f"alarm before clear={alarmed}; after: {json.dumps(c.gas)[:200]}")

    site = c.site or {}
    lim = site.get("probe") if isinstance(site.get("probe"), dict) else {}
    check("payload/site advertises the probe rules",
          lim.get("max_reach_m") == PROBE_REACH_M and lim.get("reach_from") == "body_front"
          and lim.get("requires_pose") is True and "simulated" in str(lim.get("label"))
          and "arm-mounted on the real robot" in str(lim.get("label")),
          json.dumps(lim)[:300])

    ok, msg = c.stand_and_wait()
    # Like the mission: line up on the door axis facing the tank, then walk
    # straight in until the nose is at the opening (a goal-seeking walk from
    # off-axis arrives turned, and the 1.0 m-deep points are then out of reach).
    dist, yaw_err = float("nan"), float("nan")
    if ok:
        c.walk_toward(DOOR_PRE_XY[0], DOOR_PRE_XY[1], stop_dist=0.1)
        yaw_err = c.turn_to(FACE_TANK_YAW)
        c.approach_x(DOOR_MOUTH_XY[0] + 0.03)  # ~3 cm coast after the stop
        x, y, _ = c.pose()
        dist = math.hypot(x - DOOR_MOUTH_XY[0], y - DOOR_MOUTH_XY[1])
    nx, ny = c.nose()
    deep = math.hypot(DOOR_PROBE_6[-1][0] - nx, DOOR_PROBE_6[-1][1] - ny)
    check("walk to the door mouth (nose at the opening)",
          ok and dist < 0.2 and abs(wrap(c.yaw() - FACE_TANK_YAW)) < 0.25 and deep <= PROBE_REACH_M,
          f"{msg}; lined up (yaw error {yaw_err:+.2f} rad at {DOOR_PRE_XY}); "
          f"{dist:.2f} m from {DOOR_MOUTH_XY}, pose={tuple(round(v, 2) for v in c.pose())}, "
          f"yaw={c.yaw():+.2f}, nose=({nx:.2f}, {ny:.2f}), 1.0 m-deep points {deep:.2f} m from the nose")

    res = c.probe("e2e-raw", DOOR_PROBE) or {}
    smp = res.get("samples") or []
    h2s = [s.get("h2s_ppm", float("nan")) for s in smp]
    check("probe round-trip (unventilated): H2S alarm at 0.2 m, pooled low",
          res.get("id") == "e2e-raw" and len(smp) == 3 and "error" not in res
          and [s.get("point") for s in smp] == DOOR_PROBE
          and smp[0].get("alarms", {}).get("h2s") is True and h2s[0] > h2s[1] > h2s[2]
          and isinstance(res.get("sim_time"), (int, float)),
          json.dumps(res)[:400])
    res = c.probe("e2e-far", [[-4.8, 0.0, 0.2]]) or {}
    check("probe beyond reach is rejected with an error",
          res.get("id") == "e2e-far" and "reach" in str(res.get("error", "")) and res.get("samples") == [],
          json.dumps(res)[:200])
    res = c.probe("e2e-6pt", DOOR_PROBE_6) or {}
    smp = res.get("samples") or []
    check("probe accepted inside the opening: 0.5 + 1.0 m deep x 0.2/0.6/1.0 m",
          res.get("id") == "e2e-6pt" and "error" not in res and len(smp) == 6
          and [s.get("point") for s in smp] == DOOR_PROBE_6 and res.get("simulated") is True
          and "arm-mounted on the real robot" in str(res.get("probe"))
          and all(smp[i].get("alarms", {}).get("h2s") is True for i in (0, 3)),
          f"nose=({nx:.2f}, {ny:.2f}); " + json.dumps(res)[:400])
    yaw = c.yaw()
    beyond = [round(nx + 1.3 * math.cos(yaw), 3), round(ny + 1.3 * math.sin(yaw), 3), 0.6]
    res = c.probe("e2e-beyond", [DOOR_PROBE[0], beyond]) or {}
    check("probe 1.3 m ahead of the nose (straight through the door) is rejected: reach",
          res.get("id") == "e2e-beyond" and "reach" in str(res.get("error", ""))
          and "1.1" in str(res.get("error", "")) and res.get("samples") == []
          and "simulated" in str(res.get("probe")),
          f"point={beyond}; " + json.dumps(res)[:300])
    wall_pt = [BEHIND_SHELL[0], math.copysign(BEHIND_SHELL[1], ny), BEHIND_SHELL[2]]
    reach = math.hypot(wall_pt[0] - nx, wall_pt[1] - ny)
    res = c.probe("e2e-wall", [wall_pt]) or {}
    check("probe through the tank wall is rejected: occluded by tank wall",
          reach <= PROBE_REACH_M and res.get("id") == "e2e-wall"
          and "occluded by tank wall" in str(res.get("error", "")) and res.get("samples") == [],
          f"point={wall_pt} ({reach:.2f} m from the nose, in reach); " + json.dumps(res)[:300])

    c.publish_json(c.pub_vent, {"on": True, "tau_s": 2.0})
    c.spin_for(12.0)  # 6 tau
    vent = (c.gas or {}).get("ventilation") or {}
    res = c.probe("e2e-vented", DOOR_PROBE) or {}
    smp = res.get("samples") or []
    check("forced ventilation decays built-in gas (probe clean after 6 tau)",
          vent.get("on") is True and vent.get("tau_s") == 2.0
          and (vent.get("fraction_remaining") or 1.0) < 0.06 and len(smp) == 3
          and not any(s.get("alarms", {}).get("any") for s in smp),
          f"ventilation={vent}; H2S={[round(s.get('h2s_ppm', -1), 2) for s in smp]}")
    c.publish_json(c.pub_vent, {"reset": True})
    reset = c.wait_for(lambda: ((c.gas or {}).get("ventilation") or {}).get("fraction_remaining") == 1.0
                       and c.gas["ventilation"].get("on") is False, 5.0)
    res = c.probe("e2e-reset", DOOR_PROBE) or {}
    smp = res.get("samples") or []
    check("ventilation reset restores the unventilated tank",
          reset and len(smp) == 3 and smp[0].get("alarms", {}).get("h2s") is True,
          f"ventilation={(c.gas or {}).get('ventilation')}; H2S={[round(s.get('h2s_ppm', -1), 2) for s in smp]}")
    r = c.call("sit")
    check("final sit", bool(r and r.success), (r.message if r else "timeout"))
    c.spin_for(2.0)
    print(f"final status: {c.status}", flush=True)

    passed = sum(RESULTS)
    print(f"E2E {'OK' if all(RESULTS) else 'FAILED'}: {passed}/{len(RESULTS)} checks passed", flush=True)
    return 0 if all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
