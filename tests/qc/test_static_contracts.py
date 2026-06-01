import ast
import os
import re

import Data_Logger
from tools.qc.common import REPO_ROOT


CONTROLLER_FILES = [
    "scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/path_follower_node.py",
    "scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/reposition_node.py",
]


def _read(path):
    with open(os.path.join(REPO_ROOT, path), "r", encoding="utf-8") as f:
        return f.read()


def _published_topic_literals(src):
    """String-literal topics passed to ``*.create_publisher(msg, topic, ...)``.

    AST-based so it is immune to whitespace/quote-style (the old exact-substring
    check missed e.g. a missing space). Variable topics can't be resolved
    statically and are skipped — the positive cmd_vel_raw assertion still pins
    that the safe output path exists, since both controllers use a literal.
    """
    topics = []
    for node in ast.walk(ast.parse(src)):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "create_publisher"
                and len(node.args) >= 2
                and isinstance(node.args[1], ast.Constant)
                and isinstance(node.args[1].value, str)):
            topics.append(node.args[1].value)
    return topics


def test_controllers_publish_only_to_cmd_vel_raw():
    # Safety contract: the only controller output path is
    # cmd_vel_raw -> estop_cli.py -> /cmd_vel. A controller must never publish
    # to /cmd_vel directly, and must expose the cmd_vel_raw path.
    for rel in CONTROLLER_FILES:
        topics = _published_topic_literals(_read(rel))
        normalized = {t.lstrip("/") for t in topics}
        assert "cmd_vel" not in normalized, (
            f"{rel} publishes directly to cmd_vel (must go via estop_cli): {topics}")
        assert "cmd_vel_raw" in normalized, (
            f"{rel} does not publish to cmd_vel_raw: {topics}")


def test_data_logger_records_required_topics():
    required = {
        "/wheel/odom",
        "/wheel/odom_zeroed",
        "/cmd_vel",
        "/cmd_vel_raw",
        "/estop",
        "/reference_path",
        "/path_follower/status",
        "/path_follower/done",
        "/path_follower/timing",
        "/gps_rtk_f9p_helical/gps/fix",
        "/gps_rtk_f9p_helical/gps/nmea",
        "/gps_rtk_f9p_helical/gps/rtk_status",
        "/pixhawk/global_position/raw/fix",
        "/pixhawk/global_position/raw/satellites",
        "/pixhawk/gpsstatus/gps1/raw",
        "/imu",
    }
    missing = sorted(required - set(Data_Logger.TOPICS))
    assert not missing, f"Data_Logger.TOPICS missing required topics: {missing}"


def test_orchestrator_process_names_match_cli_facade():
    orch_src = _read("scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/orchestrator_node.py")
    tree = ast.parse(orch_src)
    proc_names = None
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "PROCS":
                    proc_names = set(ast.literal_eval(node.value).keys())
    assert proc_names is not None

    cli_src = _read("tools/ops/limo_ops.py")
    m = re.search(r"ORCH_NAMES\s*=\s*(\{.*?\})", cli_src, re.S)
    assert m, "tools/ops/limo_ops.py must define ORCH_NAMES"
    cli_names = set(ast.literal_eval(m.group(1)))
    assert proc_names == cli_names


def test_webui_uses_documented_control_topics():
    text = _read("tools/path_gen/interactive.html")
    for topic in (
        "/orchestrator/start",
        "/orchestrator/kill",
        "/orchestrator/status",
        "/experiment/cmd",
        "/experiment/status",
        "/estop_trigger",
    ):
        assert topic in text
