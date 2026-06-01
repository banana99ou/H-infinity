#!/usr/bin/env python3
"""
RTCM3 TCP broadcaster for an RTK basestation.

Reads bytes from the base F9P over a serial port and re-streams them, unmodified,
to every connected TCP client. The intended client is the LIMO NUC running
`GPS-RTK_ROS2_pub_node.py`, which forwards them into the rover F9P over its own
serial link.

Designed to run as a systemd service on a Raspberry Pi (or any Linux SBC)
sitting next to the base antenna at the venue — see ``rtk-base.service`` and
``install.sh`` in this directory.

Differences from ``agile_ws/rtcm_server.py`` (the MacBook-bound original):

* Default serial port is ``/dev/f9p_base`` (a udev symlink, see
  ``99-f9p-base.rules``) — not ``/dev/tty.usbmodem101``.
* No ``SystemExit`` on serial silence; we log + let systemd ``Restart=always``
  handle the unrecoverable case if any.
* Serial port auto-reopens after a disconnect / read error (same pattern as the
  rover side's ``SerialManager``).
* ``print(...)`` → ``logging`` so the journal stays readable.

Usage (manual, on the Pi)::

    python3 rtcm_server.py --serial /dev/f9p_base --baud 115200 \
                           --host 0.0.0.0 --port 2101

Smoke test without a real F9P plugged in::

    python3 rtcm_server.py --demo

See README.md for the field deployment recipe.
"""

from __future__ import annotations

import argparse
import logging
import random
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

import serial  # pyserial  # pyright: ignore[reportMissingImports]

# ---------------- Defaults ----------------

SERIAL_PORT_DEFAULT = "/dev/f9p_base"  # udev symlink installed by 99-f9p-base.rules
BAUD_DEFAULT = 115200

TCP_HOST_DEFAULT = "0.0.0.0"
TCP_PORT_DEFAULT = 2101

SERIAL_REOPEN_BACKOFF_S = 2.0
RTCM_STALE_S = 5.0
STATUS_PRINT_INTERVAL_S = 5.0

SURVEY_IN_MIN_DURATION_S_DEFAULT = 60
SURVEY_IN_ACC_LIMIT_M_DEFAULT = 5.0

UBX_SYNC_1 = 0xB5
UBX_SYNC_2 = 0x62
UBX_CLASS_CFG = 0x06
UBX_ID_CFG_TMODE3 = 0x71
UBX_TMODE3_MODE_SURVEY_IN = 1

log = logging.getLogger("rtcm_server")


# ---------------- State ----------------

@dataclass
class BroadcastState:
    """Shared, lock-protected state for the broadcaster + status thread."""

    clients_lock: threading.Lock = field(default_factory=threading.Lock)
    clients: set = field(default_factory=set)

    status_lock: threading.Lock = field(default_factory=threading.Lock)
    last_data_time: float | None = None
    last_rtcm_time: float | None = None
    last_rtcm_types: set = field(default_factory=set)

    # NMEA-derived (when the base is configured to also emit NMEA — handy for
    # confirming it's seeing satellites, even though only RTCM gets forwarded).
    last_sats_in_view: int | None = None
    last_fix_quality: str | None = None
    last_sats_used: int | None = None
    last_nmea_time: float | None = None

    # Free-form buffer for the RTCM scanner.
    rtcm_buf: bytearray = field(default_factory=bytearray)
    nmea_buf: str = ""


# ---------------- RTCM helpers (CRC-24Q) ----------------

CRC24Q_POLY = 0x1864CFB


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= (b << 16)
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
            crc &= 0xFFFFFF
    return crc


def build_rtcm3_frame(msg_type: int, payload_len: int = 40) -> bytes:
    """Build a syntactically valid (but semantically empty) RTCM3 frame.

    Only used by --demo mode for end-to-end pipeline smoke tests.
    """
    payload_len = max(6, min(payload_len, 900))
    payload = bytearray(payload_len)
    payload[0] = (msg_type >> 4) & 0xFF
    payload[1] = ((msg_type & 0x0F) << 4) & 0xF0
    for i in range(2, payload_len):
        payload[i] = random.randrange(0, 256)

    hdr = bytearray(3)
    hdr[0] = 0xD3
    hdr[1] = (payload_len >> 8) & 0x03
    hdr[2] = payload_len & 0xFF

    frame_wo_crc = bytes(hdr + payload)
    crc = crc24q(frame_wo_crc)
    crc_bytes = bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])
    return frame_wo_crc + crc_bytes


def _find_rtcm_messages(buf: bytearray) -> tuple[list[tuple[int, int]], bytearray]:
    """Minimal RTCM3 frame scanner. Returns (msgs, leftover_buf).

    ``msgs`` is a list of ``(msg_type, payload_length)`` tuples for fully
    received frames. The leftover buffer contains any trailing bytes that
    didn't complete a frame yet.
    """
    i, n = 0, len(buf)
    results: list[tuple[int, int]] = []
    while i + 6 <= n:
        if buf[i] != 0xD3:
            i += 1
            continue
        length = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
        frame_len = 3 + length + 3
        if i + frame_len > n:
            break
        header0 = buf[i + 3]
        header1 = buf[i + 4]
        msg_type = ((header0 << 4) | (header1 >> 4)) & 0x0FFF
        results.append((msg_type, length))
        i += frame_len
    return results, buf[i:]


# ---------------- Serial ----------------

class SerialManager:
    """Thread-safe serial wrapper that auto-reopens on error.

    Same pattern as the rover-side ``GPS-RTK_ROS2_pub_node.py``: if a read or
    open fails, retry with a backoff. Callers see a blocking call that
    eventually returns bytes (or stays blocked indefinitely if the device
    never comes back; systemd will restart the unit in the worst case).
    """

    def __init__(self, port: str, baud: int, timeout_s: float = 1.0):
        self._port = port
        self._baud = baud
        self._timeout_s = timeout_s
        self._lock = threading.Lock()
        self._ser: serial.Serial | None = None
        self._open(initial=True)

    def _open(self, *, initial: bool) -> None:
        phase = "initial open" if initial else "re-open"
        while True:
            try:
                log.info("[serial] %s %s @ %d ...", phase, self._port, self._baud)
                self._ser = serial.Serial(self._port, self._baud, timeout=self._timeout_s)
                log.info("[serial] opened.")
                return
            except serial.SerialException as e:
                log.warning("[serial] %s failed: %s. retrying in %.1fs",
                            phase, e, SERIAL_REOPEN_BACKOFF_S)
                time.sleep(SERIAL_REOPEN_BACKOFF_S)

    def _reopen(self) -> None:
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001 — best-effort close
                    pass
                self._ser = None
        self._open(initial=False)

    def read(self, n: int = 4096) -> bytes:
        while True:
            try:
                with self._lock:
                    if self._ser is None:
                        raise serial.SerialException("serial not open")
                    return self._ser.read(n)
            except serial.SerialException as e:
                log.warning("[serial] read error: %s — reopening", e)
                self._reopen()

    def write(self, data: bytes) -> None:
        while True:
            try:
                with self._lock:
                    if self._ser is None:
                        raise serial.SerialException("serial not open")
                    self._ser.write(data)
                    self._ser.flush()
                return
            except serial.SerialException as e:
                log.warning("[serial] write error: %s — reopening", e)
                self._reopen()

    def reset_input_buffer(self) -> None:
        while True:
            try:
                with self._lock:
                    if self._ser is None:
                        raise serial.SerialException("serial not open")
                    self._ser.reset_input_buffer()
                return
            except serial.SerialException as e:
                log.warning("[serial] input-buffer reset error: %s — reopening", e)
                self._reopen()

    def close(self) -> None:
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self._ser = None


# ---------------- UBX config ----------------

def _ubx_checksum(data: bytes) -> bytes:
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


def _ubx_frame(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    header = struct.pack("<BBH", msg_class, msg_id, len(payload))
    return bytes((UBX_SYNC_1, UBX_SYNC_2)) + header + payload + _ubx_checksum(header + payload)


def build_tmode3_survey_in(min_duration_s: int, acc_limit_m: float) -> bytes:
    """Build UBX-CFG-TMODE3 that forces Survey-In in volatile receiver config."""
    acc_limit_0p1mm = int(round(acc_limit_m * 10000.0))
    payload = struct.pack(
        "<BBHiiibbbBIIIII",
        0,  # version
        0,  # reserved1
        UBX_TMODE3_MODE_SURVEY_IN,
        0, 0, 0,  # ECEF/LLH fields ignored in Survey-In mode
        0, 0, 0,  # high-precision coordinate bytes ignored in Survey-In mode
        0,  # reserved2
        0,  # fixed position accuracy ignored in Survey-In mode
        int(min_duration_s),
        acc_limit_0p1mm,
        0, 0,  # reserved3
    )
    return _ubx_frame(UBX_CLASS_CFG, UBX_ID_CFG_TMODE3, payload)


def force_survey_in_on_boot(ser: SerialManager, min_duration_s: int,
                            acc_limit_m: float) -> None:
    """Make this service startup a fresh base survey, even if F9P flash/BBR is stale."""
    log.info(
        "[ubx] forcing TMODE3 Survey-In on boot: min_duration=%ds acc_limit=%.2fm",
        min_duration_s, acc_limit_m,
    )
    ser.reset_input_buffer()
    ser.write(build_tmode3_survey_in(min_duration_s, acc_limit_m))
    time.sleep(0.2)
    ser.reset_input_buffer()


# ---------------- Threads ----------------

def serial_reader(ser: SerialManager, st: BroadcastState, stop: threading.Event) -> None:
    """Continuously read from the base F9P and broadcast to all TCP clients."""
    log.info("[reader] starting")
    while not stop.is_set():
        data = ser.read(4096)
        if not data:
            # Read timeout — no bytes this cycle, that's fine.
            continue
        _broadcast(data, st)


def demo_broadcaster(rate_hz: float, payload_len: int, also_nmea: bool,
                     st: BroadcastState, stop: threading.Event) -> None:
    """Generate synthetic RTCM3 frames at ``rate_hz`` and broadcast them.

    For end-to-end pipeline tests without a real base F9P plugged in.
    """
    log.info("[demo] %.1f Hz, payload_len=%d, also_nmea=%s",
             rate_hz, payload_len, also_nmea)
    demo_types = [1005, 1077, 1087, 1097, 1127, 1230]
    next_t = time.time()
    while not stop.is_set():
        now = time.time()
        if now < next_t:
            time.sleep(min(0.05, next_t - now))
            continue
        next_t += 1.0 / max(rate_hz, 0.1)

        mt = random.choice(demo_types)
        frame = build_rtcm3_frame(mt, payload_len=payload_len)
        _broadcast(frame, st)

        if also_nmea:
            nmea = (
                "$GNGGA,000000.00,3736.75000,N,12659.66000,E,1,12,0.9,100.0,M,18.0,M,,*00\r\n"
                "$GPGSV,1,1,12,01,40,100,30,02,50,110,35,03,60,120,40,04,30,130,25*00\r\n"
            ).encode("ascii", errors="ignore")
            _broadcast(nmea, st)


def _broadcast(data: bytes, st: BroadcastState) -> None:
    _update_state(data, st)
    with st.clients_lock:
        dead = []
        for c in st.clients:
            try:
                c.sendall(data)
            except Exception as e:  # noqa: BLE001 — any send failure = dead client
                log.debug("[bcast] dropping dead client: %s", e)
                dead.append(c)
        for d in dead:
            st.clients.discard(d)


def _update_state(data: bytes, st: BroadcastState) -> None:
    """Track liveness, RTCM message types seen, and (best-effort) NMEA fix info."""
    now = time.time()
    with st.status_lock:
        st.last_data_time = now
        st.rtcm_buf.extend(data)
        msgs, st.rtcm_buf = _find_rtcm_messages(st.rtcm_buf)
        if msgs:
            st.last_rtcm_time = now
            for mt, _ln in msgs:
                st.last_rtcm_types.add(mt)
    _update_nmea_status(data, st)


def _update_nmea_status(data: bytes, st: BroadcastState) -> None:
    try:
        text = data.decode("ascii", errors="ignore")
    except Exception:  # noqa: BLE001
        return
    if not text:
        return

    with st.status_lock:
        st.nmea_buf += text
        lines = st.nmea_buf.split("\r\n")
        st.nmea_buf = lines[-1]
        tail = lines[:-1]

    for line in tail:
        if not line.startswith("$"):
            continue
        if "*" in line:
            line = line.split("*", 1)[0]
        fields = line.split(",")
        talker = fields[0]
        now = time.time()
        if talker in ("$GPGSV", "$GLGSV", "$GAGSV", "$GBGSV", "$GQGSV", "$GNGSV"):
            if len(fields) >= 4 and fields[3].isdigit():
                with st.status_lock:
                    st.last_sats_in_view = int(fields[3])
                    st.last_nmea_time = now
        elif talker in ("$GPGGA", "$GNGGA"):
            if len(fields) >= 8:
                fix_q = fields[6]
                sats_used = fields[7]
                with st.status_lock:
                    st.last_fix_quality = fix_q if fix_q != "" else None
                    st.last_sats_used = int(sats_used) if sats_used.isdigit() else None
                    st.last_nmea_time = now


def status_printer(st: BroadcastState, stop: threading.Event) -> None:
    """Emit a one-line health summary periodically.

    Unlike the agile_ws original, this does **not** SystemExit on silence —
    we just log a WARNING and keep serving. Real recovery is handled by
    SerialManager (reopen) and systemd (restart) as a backstop.
    """
    log.info("[status] starting (%.1fs interval)", STATUS_PRINT_INTERVAL_S)
    was_alive: bool | None = None
    while not stop.is_set():
        time.sleep(STATUS_PRINT_INTERVAL_S)
        now = time.time()

        with st.status_lock:
            alive = (st.last_data_time is not None
                     and (now - st.last_data_time) < 3.0)
            rtcm_ready = (st.last_rtcm_time is not None
                          and (now - st.last_rtcm_time) < RTCM_STALE_S)
            rtcm_age = (now - st.last_rtcm_time) if st.last_rtcm_time else None
            rtcm_types = sorted(st.last_rtcm_types)

            sats_view = st.last_sats_in_view
            sats_used = st.last_sats_used
            fix_q = st.last_fix_quality
            have_nmea = (st.last_nmea_time is not None
                         and (now - st.last_nmea_time) < 5.0)

        with st.clients_lock:
            n_clients = len(st.clients)

        if have_nmea and fix_q is not None:
            if fix_q in ("4", "5"):
                fix_desc = "RTK"
            elif fix_q == "0":
                fix_desc = "no fix"
            else:
                fix_desc = f"fix_q={fix_q}"
        else:
            fix_desc = "unknown"

        if rtcm_ready:
            rtcm_desc = f"yes types={rtcm_types} age={rtcm_age:.1f}s"
        else:
            rtcm_desc = "no"

        log.info(
            "[status] alive=%s clients=%d sats(view/used)=%s/%s fix=%s rtcm=%s",
            alive, n_clients, sats_view, sats_used, fix_desc, rtcm_desc,
        )

        if was_alive is True and not alive:
            log.warning("[status] serial input went silent (no data for >3s) — "
                        "SerialManager will reopen on next read failure")
        was_alive = alive


# ---------------- TCP server ----------------

def handle_client(conn: socket.socket, addr: tuple, st: BroadcastState) -> None:
    log.info("[tcp] client connected: %s", addr)
    with st.clients_lock:
        st.clients.add(conn)
    try:
        while True:
            # Clients in this protocol don't send anything; recv() blocks until
            # the client disconnects. (NTRIP rovers do send a GET/auth header
            # in real NTRIP, but our F9P node just opens a raw TCP socket.)
            try:
                data = conn.recv(1024)
            except Exception as e:  # noqa: BLE001
                log.debug("[tcp] %s recv failed: %s", addr, e)
                break
            if not data:
                break
    finally:
        log.info("[tcp] client disconnected: %s", addr)
        with st.clients_lock:
            st.clients.discard(conn)
        try:
            conn.close()
        except Exception:  # noqa: BLE001
            pass


def serve_forever(host: str, port: int, st: BroadcastState,
                  stop: threading.Event) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    s.listen(8)
    s.settimeout(1.0)  # poll stop_event without hard-blocking accept()
    log.info("[tcp] listening on %s:%d", host, port)

    try:
        while not stop.is_set():
            try:
                conn, addr = s.accept()
            except socket.timeout:
                continue
            threading.Thread(
                target=handle_client, args=(conn, addr, st), daemon=True,
            ).start()
    finally:
        s.close()


# ---------------- Main ----------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--serial", default=SERIAL_PORT_DEFAULT,
                        help=f"serial device (default: {SERIAL_PORT_DEFAULT})")
    parser.add_argument("--baud", type=int, default=BAUD_DEFAULT,
                        help=f"serial baud (default: {BAUD_DEFAULT})")
    parser.add_argument("--host", default=TCP_HOST_DEFAULT,
                        help=f"bind host (default: {TCP_HOST_DEFAULT})")
    parser.add_argument("--port", type=int, default=TCP_PORT_DEFAULT,
                        help=f"bind port (default: {TCP_PORT_DEFAULT})")
    parser.add_argument("--demo", action="store_true",
                        help="run without a real base F9P; synthesize RTCM3 frames")
    parser.add_argument("--demo-rate", type=float, default=2.0,
                        help="demo: frames per second (default: 2.0)")
    parser.add_argument("--demo-len", type=int, default=80,
                        help="demo: payload length in bytes (default: 80)")
    parser.add_argument("--demo-no-nmea", action="store_true",
                        help="demo: don't also emit fake NMEA")
    parser.add_argument("--survey-min-duration-s", type=int,
                        default=SURVEY_IN_MIN_DURATION_S_DEFAULT,
                        help=("base boot Survey-In minimum duration in seconds "
                              f"(default: {SURVEY_IN_MIN_DURATION_S_DEFAULT})"))
    parser.add_argument("--survey-acc-limit-m", type=float,
                        default=SURVEY_IN_ACC_LIMIT_M_DEFAULT,
                        help=("base boot Survey-In accuracy limit in meters "
                              f"(default: {SURVEY_IN_ACC_LIMIT_M_DEFAULT})"))
    parser.add_argument("--no-force-survey-in", action="store_true",
                        help="do not force TMODE3 Survey-In on startup")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    stop = threading.Event()
    st = BroadcastState()

    if args.demo:
        threading.Thread(
            target=demo_broadcaster,
            args=(args.demo_rate, args.demo_len, not args.demo_no_nmea, st, stop),
            daemon=True,
        ).start()
    else:
        ser = SerialManager(args.serial, args.baud)
        if args.no_force_survey_in:
            log.warning("[ubx] not forcing Survey-In on boot (--no-force-survey-in)")
        else:
            force_survey_in_on_boot(
                ser,
                min_duration_s=args.survey_min_duration_s,
                acc_limit_m=args.survey_acc_limit_m,
            )
        threading.Thread(
            target=serial_reader, args=(ser, st, stop), daemon=True,
        ).start()

    threading.Thread(target=status_printer, args=(st, stop), daemon=True).start()

    try:
        serve_forever(args.host, args.port, st, stop)
    except KeyboardInterrupt:
        log.info("interrupt — shutting down")
        stop.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
