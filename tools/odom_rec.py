"""Record /demo_spot odometry (10 Hz decimated) and payload/gas to JSONL until SIGTERM/timeout.

What it is:
    The recorder whose output tools/rh_analyze.py reads as ODOM.jsonl (was
    /opt/spot-demo/tools/odom_rec.py on the sim host, copied into the repo
    2026-09-27). Needs rclpy. tools/rehearse_rep.sh starts it for you in the
    spot-rec container, with the same DDS isolation (loopback profile, domain
    42) and unprivileged user as every other container on the host
    (host/sim_lib.sh), mounting only this file read-only and logs/. To run it
    by hand, copy that docker run from tools/rehearse_rep.sh rather than
    mounting the whole repository read-write as root.
    # stop early: docker stop spot-rec  (SIGTERM ends the recording cleanly)

Usage:
    python3 odom_rec.py OUT.jsonl [duration_s]      (default 600 s)

Records (one JSON object per line, "t" = unix wall time at receipt):
    {"t", "k": "odom", "x", "y", "z", "yaw"}   /demo_spot/odometry, >= 0.1 s apart
    {"t", "k": "gas", "g": <payload/gas dict>} every /demo_spot/payload/gas message
"""
import json, sys, time, signal, math
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
out = open(sys.argv[1], "w", buffering=1)
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 600
rclpy.init()
n = rclpy.create_node("odom_rec")
last = [0.0]
def on_odom(m):
    t = time.time()
    if t - last[0] < 0.1:
        return
    last[0] = t
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    out.write(json.dumps({"t": t, "k": "odom", "x": p.x, "y": p.y, "z": p.z, "yaw": yaw}) + "\n")
def on_gas(m):
    out.write(json.dumps({"t": time.time(), "k": "gas", "g": json.loads(m.data)}) + "\n")
n.create_subscription(Odometry, "/demo_spot/odometry", on_odom, 10)
n.create_subscription(String, "/demo_spot/payload/gas", on_gas, 10)
signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
end = time.time() + dur
try:
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
except KeyboardInterrupt:
    pass
out.close()
