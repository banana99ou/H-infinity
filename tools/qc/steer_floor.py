"""Floor steering sweep, stock vs patched limo_base, back to back (turning LEFT).

Pass 1 steering_mode=agilex  (stock):   ask 0.10, 0.198, 0.408 rad
Pass 2 steering_mode=direct  (patched): ask 0.10, 0.198, 0.25, 0.30, 0.35, 0.408 rad
3 s per step at 0.3 m/s, no stops inside a pass, 2 s stop between passes.
Measured per step over its last 1.5 s:
  driven = atan(L * w_imu / v_wheel)      what the robot actually turned
  report = the chassis' own steering report (recovered from /wheel/odom)
Footprint (sim, both passes, 75-100% of the reported angle): <= 2.7 m ahead,
<= 3.2 m left, nothing right/behind.

Publishes ONLY cmd_vel_raw (-> estop_cli -> /cmd_vel). Refuses unless base +
estop are up, estop is clear, chassis is in Ackermann mode, odom/IMU are live
and nobody else publishes cmd_vel_raw. Zero command + the steering_mode it found
restored on exit, Ctrl-C or any error.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from limo_msgs.msg import LimoStatus
from rcl_interfaces.srv import SetParameters, GetParameters

L, T, SCALE = 0.2, 0.172, 2.47
V, HOLD = 0.3, 3.0
PASSES = [("agilex", [0.10, 0.198, 0.408]),
          ("direct", [0.10, 0.198, 0.25, 0.30, 0.35, 0.408])]
_args = sys.argv[1:]
for a in [a for a in _args if "=" in a]:          # v=-0.15 (reverse)  hold=2.5
    k, val = a.split("=")
    if k == "v": V = float(val)
    elif k == "hold": HOLD = float(val)
    else: raise SystemExit(f"unknown arg {a}")
assert 0.08 <= abs(V) <= 0.3 and 1.5 <= HOLD <= 3.0, (V, HOLD)
_args = [a for a in _args if "=" not in a]
if _args:   # e.g.  direct:0.30,0.35  [agilex:0.408 ...]
    PASSES = [(a.split(":")[0], [float(x) for x in a.split(":")[1].split(",")]) for a in _args]
    assert all(m in ("agilex", "direct") and all(0 < d <= 0.45 for d in ds) for m, ds in PASSES)

rclpy.init()
n = Node("steer_floor")
pub = n.create_publisher(Twist, "cmd_vel_raw", 10)
imu, odo, est, stat = [], [], [], []
n.create_subscription(Imu, "/imu", lambda m: imu.append((time.monotonic(), m.angular_velocity.z)), 50)
n.create_subscription(Odometry, "/wheel/odom", lambda m: odo.append(
    (time.monotonic(), math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), m.twist.twist.angular.z)), 50)
latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(Bool, "/estop", lambda m: est.append(m.data), latched)
n.create_subscription(LimoStatus, "/limo_status", lambda m: stat.append((m.motion_mode, m.control_mode)), 10)
sent = []          # (t, v, w) we published
foreign = []       # cmd_vel_raw messages that were not ours


def _raw_cb(m):
    t = time.monotonic()
    mine = any(abs(m.linear.x - v) < 1e-6 and abs(m.angular.z - w) < 1e-6 for ts, v, w in sent if t - ts < 1.0)
    if not mine:
        foreign.append((t, m.linear.x, m.angular.z))


n.create_subscription(Twist, "cmd_vel_raw", _raw_cb, 50)
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


def get_mode():
    g = call(getp, GetParameters.Request(names=["steering_mode"]))
    return g.values[0].string_value if g and g.values else None


def set_mode(m):
    call(setp, SetParameters.Request(parameters=[Parameter("steering_mode", value=m).to_parameter_msg()]))
    return get_mode() == m


def send(cmd):
    sent.append((time.monotonic(), cmd.linear.x, cmd.angular.z))
    del sent[:-60]
    pub.publish(cmd)


def stop(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        send(Twist()); spin(0.05)


def med(xs):
    xs = sorted(xs); return xs[len(xs) // 2] if xs else float("nan")


rows = []
rc = 0
orig_mode = None   # steering_mode found before the first pass
try:
    t0 = time.monotonic()
    while time.monotonic() - t0 < 12.0 and not (odo and imu and est and stat
                                                 and n.get_publishers_info_by_topic("/cmd_vel")):
        spin(0.2)
    spin(0.5)
    problems = []
    others = [p.node_name for p in n.get_publishers_info_by_topic("/cmd_vel_raw")
              if p.node_name not in ("steer_floor", "rosbridge_websocket")]
    if others: problems.append(f"other cmd_vel_raw publishers {others}")
    foreign.clear(); spin(3.0)
    if foreign: problems.append(f"{len(foreign)} foreign cmd_vel_raw msgs in 3 s (battle-station teleop active?)")
    direct_pubs = [p.node_name for p in n.get_publishers_info_by_topic("/cmd_vel") if p.node_name != "limo_estop_cli"]
    if direct_pubs: problems.append(f"/cmd_vel published by {direct_pubs} besides the estop")
    if not n.get_publishers_info_by_topic("/cmd_vel"): problems.append("estop not relaying (/cmd_vel has no publisher)")
    if not est or est[-1]: problems.append(f"estop state {est[-1] if est else 'unknown'}")
    if not stat or stat[-1][0] != 1: problems.append(f"motion_mode {stat[-1][0] if stat else 'unknown'} (need 1 = Ackermann)")
    if not stat or stat[-1][1] != 1:
        problems.append(f"control_mode {stat[-1][1] if stat else 'unknown'} (need 1 = serial command; 3 = remote "
                        "control, serial commands are ignored)")
    now = time.monotonic()
    if not imu or now - imu[-1][0] > 0.5: problems.append("/imu not live")
    if not odo or now - odo[-1][0] > 0.5: problems.append("/wheel/odom not live")
    if not setp.wait_for_service(timeout_sec=3.0): problems.append("limo_base_node parameters unavailable")
    if problems:
        print("REFUSING: " + "; ".join(problems)); rc = 2
    else:
        # Restored on exit: the driver default is direct since 2026-10-07 and
        # run_executor refuses preflight on anything else.
        orig_mode = get_mode() or "direct"
        for pi, (mode, steps) in enumerate(PASSES):
            if pi:
                stop(2.0)
            if not set_mode(mode):
                print(f"could not set steering_mode={mode}; stopping"); rc = 3; break
            print(f"pass {pi + 1}: steering_mode={mode}  v={V}  hold={HOLD}s", flush=True)
            for d in steps:
                cmd = Twist(); cmd.linear.x = V; cmd.angular.z = V * math.tan(d) / L
                ts = time.monotonic()
                while time.monotonic() - ts < HOLD:
                    if est and est[-1]:
                        raise RuntimeError("E-STOP active")
                    if foreign:
                        raise RuntimeError(f"someone else commanded cmd_vel_raw {foreign[-1][1:]}")
                    if stat and stat[-1][1] != 1:
                        raise RuntimeError(f"chassis left command mode (control_mode {stat[-1][1]}): operator took over")
                    send(cmd); spin(0.05)
                te = time.monotonic()
                wi = med([w for t, w in imu if te - 1.5 <= t <= te])
                v = med([x[1] for x in odo if te - 1.5 <= x[0] <= te])
                wo = med([x[2] for x in odo if te - 1.5 <= x[0] <= te])
                driven = math.atan(L * abs(wi) / v) if v > 0.05 else float("nan")
                if mode == "direct":
                    rep = math.atan(L * abs(wo) / v) if v > 0.05 else float("nan")
                else:
                    rr = v / abs(wo) - T / 2 if abs(wo) > 1e-6 else float("inf")
                    rep = math.atan(L / rr) / SCALE if rr > 0 and math.isfinite(rr) else 0.0
                rows.append((mode, d, v, wi, rep, driven))
                print(f"  ask {d:.3f}  report {rep:.3f}  driven {driven:.3f} rad  R {L / math.tan(driven):.2f} m  "
                      f"(v {v:.2f}, w_imu {wi:+.3f})", flush=True)
finally:
    stop(1.0)
    try:
        if orig_mode is not None:   # only undo a change this script made
            print(f"restore steering_mode={orig_mode}:", "ok" if set_mode(orig_mode) else "FAILED")
    finally:
        stop(0.3)
        n.destroy_node(); rclpy.shutdown()

if rows:
    print(f"\n{'mode':7s} {'ask':>6s} {'report':>7s} {'driven':>7s} {'R driven':>8s} {'driven/report':>13s}")
    for mode, d, v, wi, rep, dr in rows:
        print(f"{mode:7s} {d:6.3f} {rep:7.3f} {dr:7.3f} {L / math.tan(dr):8.2f} {dr / rep if rep else float('nan'):13.2f}")
sys.exit(rc)
