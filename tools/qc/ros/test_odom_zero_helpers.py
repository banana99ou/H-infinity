"""ros-sim tier: odom_zero_node pure helpers (NUC-run, laptop-skip).

The quaternion<->yaw helpers are the SE(2) re-anchoring building blocks (ROC
L3); a sign error in either corrupts /wheel/odom_zeroed and silently misaligns
all path tracking. These run on the sourced NUC via
``python3 tools/qc/run_qc.py ros-sim``; on the laptop they SKIP (no rclpy).

NOTE: the SE(2) transform itself is still inline in OdomZeroNode._odom_cb and is
NOT covered here — it needs to be extracted into a pure, rclpy-free module
inside the package before it can be unit-tested. See the ToDo test-coverage
backlog (Tier 3) for that follow-up.
"""

import math

import pytest

pytest.importorskip("rclpy")  # laptop has no ROS -> clean skip at collection

from limo_path_follower import odom_zero_node as oz  # noqa: E402


class _Quat:
    """Duck-typed geometry_msgs/Quaternion (only x,y,z,w are read)."""

    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


@pytest.mark.parametrize("yaw", [0.0, 0.5, -1.2, math.pi / 2, -math.pi / 2, 2.9])
def test_yaw_quaternion_round_trip(yaw):
    x, y, z, w = oz._quaternion_from_yaw(yaw)
    back = oz._yaw_from_quaternion(_Quat(x, y, z, w))
    assert oz._wrap(back - yaw) == pytest.approx(0.0, abs=1e-9)


def test_quaternion_from_yaw_is_planar_unit():
    x, y, z, w = oz._quaternion_from_yaw(1.234)
    assert (x, y) == (0.0, 0.0)  # planar: no roll/pitch
    assert x * x + y * y + z * z + w * w == pytest.approx(1.0, abs=1e-12)


def test_wrap_maps_into_half_open_pi_interval():
    assert oz._wrap(3 * math.pi) == pytest.approx(oz._wrap(math.pi), abs=1e-9)
    for a in (-10.0, -math.pi - 0.1, 0.0, math.pi, 10.0):
        assert -math.pi < oz._wrap(a) <= math.pi + 1e-12
