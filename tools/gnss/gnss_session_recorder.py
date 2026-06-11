#!/usr/bin/env python3
"""Session-long GNSS dataset bag (the GNSS sensor-comparison project).

Separate dataset from the H-inf experiment matrix: one continuous bag per
GNSS-stack session, capturing both receivers (F9P RTK + Pixhawk GPS) plus
enough motion context (odom, commanded velocity, reference paths) to see how
the LIMO moved while the receivers were being compared. Started/stopped by
the MAVROS+RTK launch file, so it runs whenever the GNSS stack runs — no
operator step, no interaction with the per-leg experiment bags.

Bags land under "Experiment Data/gnss_sessions/<stamp>_gnss_session/" and
ride the normal `tools/sync/sync.sh pull` to the laptop.
"""

import os
import signal
import sys
import time
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

import Data_Logger  # noqa: E402

GNSS_SESSION_TOPICS = [
    # Receiver under test 1: F9P helical RTK
    "/gps_rtk_f9p_helical/gps/fix",
    "/gps_rtk_f9p_helical/gps/nmea",
    "/gps_rtk_f9p_helical/gps/rtk_status",
    # Receiver under test 2: Pixhawk GPS
    "/pixhawk/global_position/raw/fix",
    "/pixhawk/global_position/raw/satellites",
    "/pixhawk/gpsstatus/gps1/raw",
    # Heading / compass
    "/heading/fused",
    "/heading/fused_status",
    "/pixhawk/global_position/compass_hdg",
    # Motion context: how the LIMO moved during the recording
    "/wheel/odom",
    "/wheel/odom_zeroed",
    "/imu",
    "/cmd_vel",
    "/reference_path",
    "/estop",
]


def main() -> int:
    stamp = datetime.now().strftime("%y_%m%d_%H%M")
    bag_path = os.path.join(
        _REPO, "Experiment Data", "gnss_sessions", f"{stamp}_gnss_session")

    rec = Data_Logger.BagRecorder(bag_path, topics=GNSS_SESSION_TOPICS)
    rec.start()
    print(f"[gnss_session] recording -> {bag_path}", flush=True)

    stopping = {"flag": False}

    def _stop(signum, frame):
        stopping["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    # The launch system owns our lifetime: idle until it signals us, then
    # stop the bag cleanly so metadata.yaml gets finalized.
    while not stopping["flag"]:
        if not rec.is_recording:
            print("[gnss_session] recorder subprocess died; exiting", flush=True)
            return 1
        time.sleep(0.5)

    info = rec.stop(timeout_s=15.0)
    print(f"[gnss_session] stopped after {info['duration_s']}s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
