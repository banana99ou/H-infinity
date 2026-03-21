#!/usr/bin/env python3
"""
ROS2 node that feeds RTCM to F9P-Helical and publishes GPS-RTK data.

This node:
- Connects to a TCP RTCM broadcaster
- Pumps RTCM3 bytes into F9P-Helical
- Reads NMEA from F9P-Helical and publishes RTK GPS data
- Publishes human-readable status information

Requires:
- F9P-Helical setup to take NMEA+UBX+RTCM3 in and NMEA+UBX out over USB.
    - MSG 
        - USB NMEA+UBX+RTCM3 in NMEA+UBX out
    - PRT
        - UBX-NAV-PVT       (position/vel/time)
        - UBX-NAV-RELPOSNED (RTK indicator: float/fix + baseline)
        - UBX-RXM-RTCM      (RTCM3 messages)
        - UBX-NAV-STATUS    (general fix flags / solution status)
        - UBX-NAV-SAT       (satellites in view)
        - UBX-NAV-SIG       (signal strength)
    - CFG
        - save to flash
    - refer to: https://docs.holybro.com/gps-and-rtk-system/zed-f9p-h-rtk-series/portable-rtk-base-station-setup
- TCP RTCM broadcaster running (e.g. rtcm_server.py)
- RTCM3 messages being sent to the broadcaster

Publishes:
- NavSatFix: RTK GPS data
- String: human-readable status
"""

import threading
import socket
import serial
import time
import pynmea2 # pyright: ignore[reportMissingImports]
from dataclasses import dataclass, field
from typing import Optional

import rclpy # pyright: ignore[reportMissingImports]
from rclpy.node import Node # pyright: ignore[reportMissingImports]
from sensor_msgs.msg import NavSatFix, NavSatStatus # pyright: ignore[reportMissingImports]
from std_msgs.msg import String # pyright: ignore[reportMissingImports]

# -------------- CONFIG --------------

# SERIAL_PORT = "/dev/ttyACM0"
SERIAL_PORT = "/dev/f9p_helical"
SERIAL_BAUD = 57600

TCP_HOST = "10.42.0.170"
# TCP_HOST = "10.0.0.42"
TCP_PORT = 2101

STATUS_PRINT_INTERVAL = 1.0   # seconds
RTCM_STALE_SECONDS = 5.0      # consider RTCM stale after this

# Debugging (set True temporarily when diagnosing RTCM "stale")
DEBUG_RTCM = False
DEBUG_RTCM_PRINT_INTERVAL = 2.0  # seconds (rate limit for debug prints)

# New: NMEA life sign print config
NMEA_LIFE_SIGN_INTERVAL = 5.0  # seconds between NMEA log prints

# -------------- STATE --------------

@dataclass
class GNSSStatus:
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None
    fix_quality: int = 0
    fix_desc: str = "NO FIX"
    num_sats: int = 0
    hdop: Optional[float] = None
    last_gps_time: float = field(default_factory=time.time)
    msg_count: int = 0

@dataclass
class RTCMStatus:
    total_bytes: int = 0
    # Updated after we successfully forward bytes into the F9P (post-serial write)
    last_rx_time: float = 0.0
    # Updated immediately after socket recv() returns bytes (pre-serial write)
    last_net_rx_time: float = 0.0
    # Debug counters
    socket_timeouts: int = 0
    zero_length_reads: int = 0
    reconnects: int = 0
    last_chunk_len: int = 0
    last_write_ms: float = 0.0
    _last_debug_print_time: float = 0.0

# -------------- HELPERS --------------

class SerialManager:
    """Thread-safe serial wrapper that auto-reopens on error."""

    def __init__(self, port: str, baud: int, timeout: float = 1.0):
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._lock = threading.Lock()
        self._ser: Optional[serial.Serial] = None
        self._open_serial(initial=True)

    def _open_serial(self, initial: bool = False):
        while True:
            try:
                print(f"[SERIAL] Opening {self._port} @ {self._baud} ...")
                self._ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
                print("[SERIAL] Serial opened.")
                return
            except serial.SerialException as e:
                phase = "initial open" if initial else "re-open"
                print(f"[SERIAL] {phase} failed: {e}. Retrying in 2s...")
                time.sleep(2.0)

    def _reopen_serial(self):
        with self._lock:
            try:
                if self._ser is not None:
                    self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._open_serial()

    def write(self, data: bytes):
        while True:
            try:
                with self._lock:
                    if self._ser is None:
                        raise serial.SerialException("Serial not open")
                    self._ser.write(data)
                    self._ser.flush()
                return
            except serial.SerialException as e:
                print(f"[SERIAL] Write error: {e}. Reopening port...")
                self._reopen_serial()

    def readline(self) -> bytes:
        while True:
            try:
                with self._lock:
                    if self._ser is None:
                        raise serial.SerialException("Serial not open")
                    return self._ser.readline()
            except serial.SerialException as e:
                print(f"[SERIAL] Read error: {e}. Reopening port...")
                self._reopen_serial()

    def close(self):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None


def fix_quality_to_desc(q: int) -> str:
    mapping = {
        0: "NO FIX",
        1: "GPS",
        2: "DGPS",
        3: "PPS",
        4: "RTK FIXED",
        5: "RTK FLOAT",
        6: "DEAD RECKONING",
        7: "MANUAL",
        8: "SIMULATION",
    }
    return mapping.get(q, f"UNKNOWN({q})")

class HelicalGpsNode(Node):
    """ROS2 node that feeds RTCM to F9P and publishes RTK GPS data."""

    def __init__(self):
        super().__init__("helical_gps_rtk_node")

        self.ser_mgr = SerialManager(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        self.gnss_status = GNSSStatus()
        self.rtcm_status = RTCMStatus()
        self._stop_event = threading.Event()

        self.last_msg_count = 0
        self.last_status_time = time.time()

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.nmea_pub = self.create_publisher(String, "gps/nmea", 10)
        self.status_pub = self.create_publisher(String, "gps/rtk_status", 10)

        # New: track time for NMEA life sign print
        self._last_nmea_life_sign_time = 0
        self._nmea_life_sign_interval = NMEA_LIFE_SIGN_INTERVAL

        # Start worker threads
        self._rtcm_thread = threading.Thread(
            target=rtcm_forwarder,
            args=(self.ser_mgr, self.rtcm_status, self._stop_event),
            daemon=True,
        )
        self._nmea_thread = threading.Thread(
            target=nmea_reader,
            args=(self.ser_mgr, self.gnss_status, self, self._stop_event),
            daemon=True,
        )
        # Optional console status printer (same as original script)
        self._status_thread = threading.Thread(
            target=status_printer,
            args=(self.gnss_status, self.rtcm_status, self._stop_event),
            daemon=True,
        )
        self._rtcm_thread.start()
        self._nmea_thread.start()
        self._status_thread.start()

        # Status + NavSatFix publisher timer
        self.create_timer(STATUS_PRINT_INTERVAL, self._status_timer_cb)

        self.get_logger().info("Helical GPS RTK node started.")

    def publish_nmea(self, line: str):
        """Publish raw NMEA sentence and occasionally print to logger for sign of life."""
        msg = String()
        msg.data = line
        self.nmea_pub.publish(msg)

        # NMEA sign of life logger
        t_now = time.time()
        if (t_now - getattr(self, "_last_nmea_life_sign_time", 0)) >= self._nmea_life_sign_interval:
            self._last_nmea_life_sign_time = t_now
            # Only log short (first 80 chars) to avoid flooding
            self.get_logger().info(f"F9P NMEA: {line[:80]}")
        

    def _status_timer_cb(self):
        """Publish NavSatFix and human-readable RTK status string."""
        now = self.get_clock().now().to_msg()
        gs = self.gnss_status
        rs = self.rtcm_status
        t_now = time.time()

        # RTCM status (forwarded-to-F9P time vs. network-received time)
        if rs.last_rx_time > 0:
            rtcm_age = t_now - rs.last_rx_time
            rtcm_active = rtcm_age < RTCM_STALE_SECONDS
        else:
            rtcm_age = float("inf")
            rtcm_active = False

        if getattr(rs, "last_net_rx_time", 0.0) > 0:
            rtcm_net_age = t_now - rs.last_net_rx_time
        else:
            rtcm_net_age = float("inf")

        # GNSS frequency calculation
        dt = t_now - self.last_status_time
        if dt > 0:
            hz = (gs.msg_count - self.last_msg_count) / dt
        else:
            hz = 0.0
        self.last_msg_count = gs.msg_count
        self.last_status_time = t_now

        # NavSatFix
        fix_msg = NavSatFix()
        fix_msg.header.stamp = now
        fix_msg.header.frame_id = "f9p_helical"

        fix_msg.latitude = gs.lat if gs.lat is not None else float("nan")
        fix_msg.longitude = gs.lon if gs.lon is not None else float("nan")
        fix_msg.altitude = gs.alt if gs.alt is not None else float("nan")

        status = NavSatStatus()
        status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_GALILEO
            | NavSatStatus.SERVICE_COMPASS
        )
        if gs.fix_quality == 0:
            status.status = NavSatStatus.STATUS_NO_FIX
        elif gs.fix_quality in (1, 2, 3):
            status.status = NavSatStatus.STATUS_FIX
        elif gs.fix_quality in (4, 5):
            # Treat RTK FIX/FLOAT as high-quality differential fix
            status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            status.status = NavSatStatus.STATUS_FIX
        fix_msg.status = status
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(fix_msg)

        # Human-readable status
        lat_str = f"{gs.lat:.8f}" if gs.lat is not None else "N/A"
        lon_str = f"{gs.lon:.8f}" if gs.lon is not None else "N/A"
        hdop_str = f"{gs.hdop:.2f}" if gs.hdop is not None else "N/A"

        status_str = (
            f"FIX: {gs.fix_desc} "
            f"(quality={gs.fix_quality}, sats={gs.num_sats}, HDOP={hdop_str}, rate={hz:.1f}Hz) | "
            f"Lat={lat_str}, Lon={lon_str} | "
            f"RTCM: {'ACTIVE' if rtcm_active else 'STALE'} "
            f"(bytes={rs.total_bytes}, fwd_age={rtcm_age:.1f}s, net_age={rtcm_net_age:.1f}s)"
        )

        s_msg = String()
        s_msg.data = status_str
        self.status_pub.publish(s_msg)

        # Also log occasionally
        self.get_logger().info(status_str)

    def destroy_node(self):
        self.get_logger().info("Shutting down Helical GPS RTK node...")
        self._stop_event.set()
        time.sleep(0.5)
        self.ser_mgr.close()
        super().destroy_node()


# -------------- THREADS --------------

def rtcm_forwarder(ser_mgr: SerialManager, rtcm_status: RTCMStatus, stop_event: threading.Event):
    """Connect to your TCP RTCM broadcaster and pump bytes into F9P."""
    while not stop_event.is_set():
        try:
            print(f"[RTCM] Connecting to {TCP_HOST}:{TCP_PORT} ...")
            with socket.create_connection((TCP_HOST, TCP_PORT), timeout=10) as sock:
                sock.settimeout(5.0)
                print("[RTCM] Connected. Receiving RTCM3 and forwarding to F9P...")
                rtcm_status.reconnects += 1

                while not stop_event.is_set():
                    try:
                        data = sock.recv(4096)
                        if not data:
                            print("[RTCM] Connection closed by remote.")
                            rtcm_status.zero_length_reads += 1
                            break
                        rtcm_status.last_net_rx_time = time.time()
                        rtcm_status.last_chunk_len = len(data)
                        # Feed F9P: this is where RTCM3 actually goes into the Helical
                        t0 = time.time()
                        ser_mgr.write(data)
                        rtcm_status.last_write_ms = (time.time() - t0) * 1000.0

                        rtcm_status.total_bytes += len(data)
                        rtcm_status.last_rx_time = time.time()

                        if DEBUG_RTCM:
                            now = time.time()
                            if (now - rtcm_status._last_debug_print_time) >= DEBUG_RTCM_PRINT_INTERVAL:
                                rtcm_status._last_debug_print_time = now
                                net_to_fwd_ms = (rtcm_status.last_rx_time - rtcm_status.last_net_rx_time) * 1000.0
                                print(
                                    "[RTCM][DBG] "
                                    f"chunk={rtcm_status.last_chunk_len}B "
                                    f"total={rtcm_status.total_bytes}B "
                                    f"write={rtcm_status.last_write_ms:.1f}ms "
                                    f"net→fwd={net_to_fwd_ms:.1f}ms "
                                    f"timeouts={rtcm_status.socket_timeouts} "
                                    f"zero_reads={rtcm_status.zero_length_reads} "
                                    f"reconnects={rtcm_status.reconnects}"
                                )
                    except socket.timeout:
                        # No data in this interval; just loop
                        rtcm_status.socket_timeouts += 1
                        if DEBUG_RTCM:
                            now = time.time()
                            if (now - rtcm_status._last_debug_print_time) >= DEBUG_RTCM_PRINT_INTERVAL:
                                rtcm_status._last_debug_print_time = now
                                age = (now - rtcm_status.last_rx_time) if rtcm_status.last_rx_time > 0 else float("inf")
                                net_age = (now - rtcm_status.last_net_rx_time) if rtcm_status.last_net_rx_time > 0 else float("inf")
                                print(
                                    "[RTCM][DBG] socket timeout "
                                    f"(fwd_age={age:.1f}s, net_age={net_age:.1f}s, "
                                    f"timeouts={rtcm_status.socket_timeouts})"
                                )
                        continue
        except (socket.error, OSError) as e:
            print(f"[RTCM] Connection error: {e}. Retrying in 5s...")
            time.sleep(5)


def nmea_reader(ser_mgr: SerialManager, gnss_status: GNSSStatus, node: HelicalGpsNode, stop_event: threading.Event):
    """Read NMEA from F9P, update GNSS status, and publish raw NMEA."""
    while not stop_event.is_set():
        try:
            line_bytes = ser_mgr.readline()
            # print(f'line_bytes: {line_bytes}')
            if not line_bytes:
                continue

            # NMEA sentences are ASCII starting with '$'
            line = line_bytes.decode("ascii", errors="ignore").strip()
            if not line.startswith("$"):
                # probably UBX or other binary, ignore
                continue

            # Publish raw NMEA
            if node is not None:
                try:
                    node.publish_nmea(line)
                except Exception as e:
                    node.get_logger().warn(f"Failed to publish NMEA: {e}")
            # (No further NMEA logging here; handled in publish_nmea.)

            try:
                msg = pynmea2.parse(line, check=True)
            except Exception:
                continue

            now = time.time()
            gnss_status.last_gps_time = now

            if isinstance(msg, pynmea2.types.talker.GGA):
                gnss_status.msg_count += 1
                gnss_status.lat = msg.latitude if msg.latitude != "" else None
                gnss_status.lon = msg.longitude if msg.longitude != "" else None
                try:
                    gnss_status.alt = float(msg.altitude) if msg.altitude not in ("", None) else None
                except (ValueError, TypeError, AttributeError):
                    gnss_status.alt = None

                try:
                    q = int(msg.gps_qual)
                except (ValueError, TypeError):
                    q = 0
                gnss_status.fix_quality = q
                gnss_status.fix_desc = fix_quality_to_desc(q)

                try:
                    gnss_status.num_sats = int(msg.num_sats) if msg.num_sats else 0
                except (ValueError, TypeError):
                    gnss_status.num_sats = 0

                try:
                    gnss_status.hdop = float(msg.horizontal_dil) if msg.horizontal_dil else None
                except (ValueError, TypeError):
                    gnss_status.hdop = None

        except Exception as e:
            # Catch-all to avoid killing the thread on unexpected errors
            if node is not None:
                node.get_logger().warn(f"[NMEA] Unexpected error: {e}")
            else:
                print(f"[NMEA] Unexpected error: {e}")
            time.sleep(1.0)


def status_printer(gnss_status: GNSSStatus, rtcm_status: RTCMStatus, stop_event: threading.Event):
    """Print GPS fix flag, RTCM status, and basic health."""
    while not stop_event.is_set():
        time.sleep(STATUS_PRINT_INTERVAL)
        now = time.time()

        # RTCM
        if rtcm_status.last_rx_time > 0:
            rtcm_age = now - rtcm_status.last_rx_time
            rtcm_active = rtcm_age < RTCM_STALE_SECONDS
        else:
            rtcm_age = float("inf")
            rtcm_active = False

        # GNSS
        lat = gnss_status.lat
        lon = gnss_status.lon
        lat_str = f"{lat:.8f}" if lat is not None else "N/A"
        lon_str = f"{lon:.8f}" if lon is not None else "N/A"
        hdop_str = f"{gnss_status.hdop:.2f}" if gnss_status.hdop is not None else "N/A"

        # print(
        #     f"[STATUS] FIX: {gnss_status.fix_desc} "
        #     f"(quality={gnss_status.fix_quality}, sats={gnss_status.num_sats}, HDOP={hdop_str}) | "
        #     f"Lat={lat_str}, Lon={lon_str} | "
        #     f"RTCM: {'ACTIVE' if rtcm_active else 'STALE'} "
        #     f"(bytes={rtcm_status.total_bytes}, age={rtcm_age:.1f}s)"
        # )

# -------------- MAIN --------------

def main():
    rclpy.init()
    node = HelicalGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
