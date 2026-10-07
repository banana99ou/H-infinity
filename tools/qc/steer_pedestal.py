"""Pedestal steering check (wheels OFF the ground) for the patched limo_base.

Does the chassis accept a serial steering command above the stock ceiling?
With the wheels in the air the IMU cannot measure the turn, so this reads what
the chassis REPORTS back (recovered from /wheel/odom) and leaves the physical
wheel angle to the person watching. Sequence (left, then right), wheel speed
0.1 m/s through cmd_vel_raw -> estop -> /cmd_vel:

  A  agilex  straight 2 s | stock FULL LOCK 5 s | straight 3 s
  B  direct  0.198 4 s | 0.30 4 s | 0.408 5 s | straight 3 s
  C  direct  -0.408 (right) 5 s | straight 2 s
then steering_mode back to agilex and zero command (also on error / Ctrl-C).
Watch: is the B 0.408 lock visibly further than the A stock lock?
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters, GetParameters

L, T, SCALE = 0.2, 0.172, 2.47
V = 0.1

rclpy.init()
n = Node("steer_pedestal")
pub = n.create_publisher(Twist, "cmd_vel_raw", 10)
odo = []
n.create_subscription(Odometry, "/wheel/odom", lambda m: odo.append(
    (time.monotonic(), math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), m.twist.twist.angular.z)), 50)
setp = n.create_client(SetParameters, "/limo_base_node/set_parameters")
getp = n.create_client(GetParameters, "/limo_base_node/get_parameters")


def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def call(cli, req):
    f = cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=3.0)
    return f.result()


def mode(m):
    r = call(setp, SetParameters.Request(parameters=[Parameter("steering_mode", value=m).to_parameter_msg()]))
    ok = bool(r and r.results and r.results[0].successful)
    g = call(getp, GetParameters.Request(names=["steering_mode"]))
    now = g.values[0].string_value if g and g.values else "?"
    print(f"  steering_mode -> {m}: {'ok' if ok and now == m else 'FAILED'} (reads {now})", flush=True)
    return ok and now == m


def hold(label, w, sec, m):
    cmd = Twist(); cmd.linear.x = V; cmd.angular.z = w
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        pub.publish(cmd); spin(0.05)
    t1 = time.monotonic()
    s = [(v, wz) for t, v, wz in odo if t1 - 1.5 <= t <= t1]
    if not s:
        print(f"  {label:28s} no odom", flush=True); return
    v = sorted(x[0] for x in s)[len(s) // 2]; wz = sorted(x[1] for x in s)[len(s) // 2]
    if v < 0.02:
        print(f"  {label:28s} reported v {v:.3f} (wheels not turning?)", flush=True); return
    if m == "direct":
        rep = math.atan(L * wz / v)
    else:
        rr = v / abs(wz) - T / 2 if abs(wz) > 1e-6 else float("inf")
        rep = math.copysign(math.atan(L / rr) / SCALE, wz) if rr > 0 and math.isfinite(rr) else 0.0
    print(f"  {label:28s} chassis reports {rep:+.3f} rad   (v {v:.3f})", flush=True)


def stop(sec=1.0):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        pub.publish(Twist()); spin(0.05)


rc = 0
try:
    t_disc = time.monotonic()
    while time.monotonic() - t_disc < 12.0 and not (odo and n.get_publishers_info_by_topic("/cmd_vel")):
        spin(0.2)                       # DDS discovery on the NUC can take seconds
    spin(0.5)
    others = [p.node_name for p in n.get_publishers_info_by_topic("/cmd_vel_raw") if p.node_name != "steer_pedestal"]
    if others or not n.get_publishers_info_by_topic("/cmd_vel") or not setp.wait_for_service(timeout_sec=3.0) \
            or not odo:
        print(f"REFUSING: other cmd_vel_raw pubs {others}, /cmd_vel pubs "
              f"{len(n.get_publishers_info_by_topic('/cmd_vel'))}, params svc {setp.service_is_ready()}, odom {bool(odo)}")
        rc = 2
    else:
        def tw(d):  # yaw rate that asks for bicycle angle d at V
            return V * math.tan(d) / L
        print("A agilex (stock)", flush=True)
        if not mode("agilex"): raise SystemExit(3)
        hold("straight", 0.0, 2.0, "agilex")
        hold("stock FULL LOCK left", tw(0.6), 5.0, "agilex")    # 0.6 rad asked -> clamps
        hold("straight", 0.0, 3.0, "agilex")
        print("B direct", flush=True)
        if not mode("direct"): raise SystemExit(3)
        hold("0.198 left (= stock max)", tw(0.198), 4.0, "direct")
        hold("0.300 left", tw(0.300), 4.0, "direct")
        hold("0.408 left", tw(0.408), 5.0, "direct")
        hold("straight", 0.0, 3.0, "direct")
        print("C direct right", flush=True)
        hold("0.408 right", tw(-0.408), 5.0, "direct")
        hold("straight", 0.0, 2.0, "direct")
finally:
    stop()
    try:
        mode("agilex")
    finally:
        stop(0.5)
        n.destroy_node(); rclpy.shutdown()
sys.exit(rc)
