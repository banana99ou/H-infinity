"""Path setup for the ros-sim tier (run on the sourced NUC, skipped on laptop).

These tests import the ROS node modules, which import ``rclpy``. Each test
module guards itself with ``pytest.importorskip("rclpy")`` so collection is a
clean SKIP on a machine without ROS (e.g. the dev laptop). On the NUC the
installed ``limo_path_follower`` package is importable after sourcing; as a
fallback we also put the in-repo package source dir on the path so the tier can
run against source without a colcon install.
"""

from __future__ import annotations

import os
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG_SRC = os.path.join(_REPO_ROOT, "scalecar-vfg-h-infinite", "ros2_bridge")

if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)


@pytest.fixture(scope="session")
def ros_context():
    """Initialize one isolated ROS context for ros-sim tests.

    Keep this non-autouse so laptop collection can still skip cleanly before
    importing ROS packages.
    """
    os.environ.setdefault("ROS_DOMAIN_ID", "91")
    import rclpy

    did_init = False
    if not rclpy.ok():
        rclpy.init()
        did_init = True
    yield
    if did_init and rclpy.ok():
        rclpy.shutdown()
