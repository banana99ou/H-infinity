# -*- coding: utf-8 -*-
"""A minimal in-process stand-in for rclpy + the message packages, so a REAL
ROS node module (run_executor_node) can be imported and driven tick by tick on
a laptop with no ROS. Only what the executor touches is modelled: parameters,
publishers (recorded), subscriptions (a topic bus), timers (not auto-fired —
the test calls node._tick()), a set_parameters client that succeeds, the
logger, the clock and the on-set-parameters callback.

install() must run BEFORE importing the node module.
"""
import sys
import time
import types
from collections import defaultdict


class _Msg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


def _msgtype(name, **defaults):
    def __init__(self, **kw):
        for k, v in defaults.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)
    return type(name, (object,), {"__init__": __init__})


String = _msgtype("String", data="")
Bool = _msgtype("Bool", data=False)
Float64 = _msgtype("Float64", data=0.0)


class _Vec:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Vec()
        self.angular = _Vec()


Twist = _Twist


class _TwistWC:
    def __init__(self):
        self.twist = _Twist()


class Odometry:
    def __init__(self):
        self.twist = _TwistWC()


class _Status:
    def __init__(self):
        self.status = 0


class NavSatFix:
    def __init__(self, latitude=0.0, longitude=0.0, status=0):
        self.latitude = latitude
        self.longitude = longitude
        self.status = _Status()
        self.status.status = status


class Bus:
    """Topic bus shared by every fake node."""

    def __init__(self):
        self.subs = defaultdict(list)
        self.sent = defaultdict(list)

    def publish(self, topic, msg):
        self.sent[topic].append(msg)
        for cb in list(self.subs[topic]):
            cb(msg)


BUS = Bus()


class _Pub:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        BUS.publish(self.topic, msg)

    def get_subscription_count(self):
        return 1


class _Param:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class _Log:
    def __init__(self, sink):
        self.sink = sink

    def info(self, m, *a, **k):
        self.sink.append(("info", str(m)))

    def warn(self, m, *a, **k):
        self.sink.append(("warn", str(m)))

    warning = warn

    def error(self, m, *a, **k):
        self.sink.append(("error", str(m)))


class _Time:
    def __init__(self):
        self.nanoseconds = int(time.time() * 1e9)


class _Clock:
    def now(self):
        return _Time()


class _Future:
    def __init__(self, result):
        self._r = result

    def done(self):
        return True

    def result(self):
        return self._r


class _Client:
    def service_is_ready(self):
        return True

    def call_async(self, req):
        res = _Msg(results=[_Msg(successful=True, reason="")
                            for _ in getattr(req, "parameters", [])])
        return _Future(res)


class Node:
    # {node_name: {param: value}} applied before the node declares its params
    # (what `--ros-args -p` does).
    OVERRIDES = {}

    def __init__(self, name, **_kw):
        self._name = name
        self._params = dict(Node.OVERRIDES.get(name, {}))
        self._param_cbs = []
        self.log = []

    def declare_parameter(self, name, default=None, *a, **k):
        self._params.setdefault(name, default)
        return _Param(name, self._params[name])

    def get_parameter(self, name):
        return _Param(name, self._params.get(name))

    def set_param_live(self, name, value):
        """What `ros2 param set` does: run the callbacks, then store."""
        p = _Param(name, value)
        for cb in self._param_cbs:
            r = cb([p])
            if not r.successful:
                return r
        self._params[name] = value
        return _Msg(successful=True, reason="")

    def add_on_set_parameters_callback(self, cb):
        self._param_cbs.append(cb)
        return cb

    def create_publisher(self, _type, topic, _qos):
        return _Pub(topic)

    def create_subscription(self, _type, topic, cb, _qos):
        BUS.subs[topic].append(cb)
        return cb

    def create_timer(self, _period, _cb):
        return None

    def create_client(self, _type, _name):
        return _Client()

    def get_logger(self):
        return _Log(self.log)

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass


def install():
    """Register the fake modules in sys.modules."""
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class _Enum:
        def __getattr__(self, k):
            return k

    class QoSProfile:
        def __init__(self, **kw):
            self.kw = kw

    rclpy = mod("rclpy", init=lambda *a, **k: None, ok=lambda: False,
                shutdown=lambda *a, **k: None, spin=lambda *a, **k: None)
    mod("rclpy.node", Node=Node)
    mod("rclpy.executors", ExternalShutdownException=RuntimeError)
    mod("rclpy.qos", QoSProfile=QoSProfile, QoSDurabilityPolicy=_Enum(),
        QoSReliabilityPolicy=_Enum(), QoSHistoryPolicy=_Enum())
    rclpy.node = sys.modules["rclpy.node"]
    mod("rcl_interfaces")
    mod("rcl_interfaces.srv", SetParameters=_msgtype(
        "SetParameters", Request=lambda: _Msg(parameters=[])))
    sys.modules["rcl_interfaces.srv"].SetParameters.Request = (
        lambda: _Msg(parameters=[]))
    mod("rcl_interfaces.msg", ParameterValue=_Msg, ParameterType=_Enum(),
        Parameter=_Msg, SetParametersResult=_Msg)
    mod("std_msgs")
    mod("std_msgs.msg", String=String, Bool=Bool, Float64=Float64)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=Odometry)
    mod("sensor_msgs")
    mod("sensor_msgs.msg", NavSatFix=NavSatFix)
    mod("geometry_msgs")
    mod("geometry_msgs.msg", Twist=Twist)
