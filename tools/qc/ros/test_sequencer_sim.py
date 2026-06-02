"""ros-sim tier: experiment_sequencer_node autonomous-recovery behavior.

These are the Tier-B "does it survive unattended?" tests. They run the REAL
sequencer as a subprocess against a mock world (see sequencer_harness.py) on an
isolated ROS_DOMAIN_ID, and assert the recovery brain behaves — the layer that
killed every May-29 field attempt and that had ZERO coverage before this.

Run on the sourced NUC:  python3 tools/qc/run_qc.py ros-sim
On the laptop they SKIP at collection (no rclpy).

Each test is one injected fault. The happy-path test is the only slow one
(records 4 throwaway bags driving a full cell to DONE); the fault tests short-
circuit before/at the first leg and finish in a few seconds.
"""

import os

import pytest

pytest.importorskip("rclpy")  # laptop has no ROS -> clean SKIP at collection

import rclpy  # noqa: E402

from sequencer_harness import SimHarness  # noqa: E402


# Force an isolated domain BEFORE rclpy.init so the sim can never see — or
# command — a real orchestrator/estop on the default domain. This is a safety
# boundary, not just test hygiene.
@pytest.fixture(scope="session", autouse=True)
def _ros_context():
    # Force an isolated domain BEFORE any rclpy.init so the sim can never see —
    # or command — a real orchestrator/estop on the default domain. Guard the
    # init/shutdown: a shared ros_context fixture (conftest.py) or another
    # ros-sim module may have already initialized the context in this session,
    # and Context.init() must only be called once.
    os.environ["ROS_DOMAIN_ID"] = "91"
    did_init = False
    if not rclpy.ok():
        rclpy.init()
        did_init = True
    yield
    if did_init and rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def harness(tmp_path):
    made = {}

    def _make(**kw):
        h = SimHarness(tmp_path=tmp_path, **kw)
        made["h"] = h
        h.start()
        return h

    yield _make
    if "h" in made:
        made["h"].stop()


# ---------------------------------------------------------------------------
# Autonomy: the headline goal — a single cell completes hands-off.
# ---------------------------------------------------------------------------
def test_single_cell_runs_to_done_unattended(harness):
    h = harness(cells=1, preflight="pass")
    h.arm()
    h.wait_message_contains("starting preflight", timeout=10)
    h.wait_phase("done", timeout=150)
    done = [d for d in h.world.get_statuses() if d.get("phase") == "done"][-1]
    assert done["pass"] >= 1, f"expected >=1 headline pass, got {done}"
    assert done["fail"] == 0, f"unexpected failures: {done}"


# ---------------------------------------------------------------------------
# F2: persistent RTK-FIXED loss pauses the batch and AUTO-RESUMES on reacquire.
# This is the single most important unattended-survival behavior.
# ---------------------------------------------------------------------------
def test_rtk_loss_pauses_then_autoresumes(harness):
    # Preflight sleeps long enough that we sit in PREFLIGHT while RTK is lost,
    # so the F2 guard (which fires in PREFLIGHT/REPOSITION_GOTO) can trip.
    h = harness(preflight="pass", preflight_sleep=6.0, rtk_loss_wait_s=2.0)
    h.world.rtk_quality = 1  # not FIXED from the start
    h.arm()
    h.wait_phase("paused", timeout=20)
    paused = [d for d in h.world.get_statuses() if d.get("phase") == "paused"][-1]
    assert "RTK" in paused["message"], paused

    h.world.rtk_quality = 4  # reacquire
    h.wait_message_contains("resumed", timeout=20)


# ---------------------------------------------------------------------------
# Backlog P0#4: the EXACT reposition abort reason must reach /experiment/status,
# not the old opaque "R3 area/exclusion or unreachable".
# ---------------------------------------------------------------------------
def test_reposition_abort_reason_propagates(harness):
    h = harness(preflight="pass", circuit_breaker_k=5)
    h.world.repo_result = "aborted"
    h.world.repo_reason = "entry outside inset working area"
    h.world.repo_err_m = 1.23
    h.world.repo_err_deg = 42.0
    h.arm()
    h.wait_message_contains("entry outside inset working area", timeout=30)
    msgs = h.messages()
    assert "reposition aborted" in msgs, msgs
    assert "err_m=1.23" in msgs, msgs


# ---------------------------------------------------------------------------
# Backlog P0#5: a preflight failure must surface WHICH check failed, not just
# "preflight FAIL (exit 1)".
# ---------------------------------------------------------------------------
def test_preflight_fail_reason_propagates(harness):
    h = harness(preflight="fail", circuit_breaker_k=5)
    h.arm()
    h.wait_message_contains("preflight FAIL", timeout=25)
    msgs = h.messages()
    assert "RTK" in msgs, f"failed check name not surfaced: {msgs}"


# ---------------------------------------------------------------------------
# C6 (the single most important invariant per the node docstring): the follower
# and the reposition node are never both alive at the same instant.
# ---------------------------------------------------------------------------
def test_c6_movers_never_both_alive(harness):
    h = harness(preflight="pass")
    h.arm()
    # Run until the follower has come up at least once (i.e. we reached a leg).
    h.wait_until(lambda: any(f for _, f, _ in h.world.get_timeline()),
                 timeout=60, what="follower to come up")
    tl = h.world.get_timeline()
    assert any(r for _, _, r in tl), "reposition never came up — test would be vacuous"
    both = [(round(t, 2)) for (t, f, r) in tl if f and r]
    assert not both, f"C6 violated: follower & reposition both alive at t={both[:5]}"


# ---------------------------------------------------------------------------
# Backlog P0#10 + the fresh-session entry point: autostart=false stays idle
# (starts no movers) until an explicit /experiment/cmd start arrives.
# ---------------------------------------------------------------------------
def test_autostart_off_idles_until_armed(harness):
    import time

    h = harness(preflight="pass")
    time.sleep(4.0)
    assert h.world.get_start_names() == [], (
        f"movers started before any arm command: {h.world.get_start_names()}")
    h.arm()
    h.wait_until(lambda: "reposition" in h.world.get_start_names(),
                 timeout=25, what="reposition to start after explicit arm")


# ---------------------------------------------------------------------------
# F4 circuit breaker: a forced leg failure (k=1) pauses the batch. Documents
# that this pause needs an operator — it does NOT auto-resume like RTK loss.
# ---------------------------------------------------------------------------
def test_circuit_breaker_pauses_after_failure(harness):
    h = harness(cells=1, circuit_breaker_k=1, max_retries=0, preflight="pass")
    h.world.repo_result = "aborted"
    h.world.repo_reason = "unreachable"
    h.arm()
    h.wait_phase("paused", timeout=30)
    paused = [d for d in h.world.get_statuses() if d.get("phase") == "paused"][-1]
    assert "circuit breaker" in paused["message"], paused


# ---------------------------------------------------------------------------
# Silent follower / crash during RUN: the sequencer does NOT watch follower
# liveness in RUN, so a follower that goes silent is only caught by run_timeout.
# Assert the leg FAILS (does not hang forever) — and note the detection is
# timeout-bound (a real unattended-latency finding, not a bug fixed here).
# ---------------------------------------------------------------------------
def test_silent_follower_fails_leg_via_run_timeout(harness):
    h = harness(cells=1, circuit_breaker_k=5, max_retries=0,
                preflight="pass", run_timeout_s=3.0)
    h.world.run_outcome = "timeout"  # never emit /path_follower/done
    h.arm()
    h.wait_message_contains("AtoB fail", timeout=40)
