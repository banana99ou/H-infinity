#!/usr/bin/env python3
"""ntfy.sh push notifications + battery-alert helper for the experiment sequencer (T8).

Standard-library only (urllib, no requests). Every public function is written so it
can NEVER crash its caller: on an empty topic, a missing config, or any network/parse
error it logs and returns a safe value (False / defaults). The sequencer (T6) imports
this defensively, so a notification failure must never abort a batch.

Conventions (https://ntfy.sh):
  POST <server>/<topic>  body = message
  headers:  X-Title, X-Priority, X-Tags

CLI (manual testing, pure stdlib, hits no robot):
  python3 tools/notify/ntfy.py --message "hello" --topic my-topic
  python3 tools/notify/ntfy.py --message "hello" --dry-run        # print, don't send
"""

from __future__ import annotations

import argparse
import logging
import sys
import urllib.error
import urllib.request

log = logging.getLogger("ntfy")

DEFAULT_SERVER = "https://ntfy.sh"

# --- battery thresholds (VOLTS) -------------------------------------------------
# M2 in DOC/system_spec.md is stated as "alert at 30%, halt at 20%", but the only
# battery telemetry on the platform is /limo_status.battery_voltage (float64 VOLTS,
# observed ~12.0 V healthy). The %->V mapping for this LiPo pack is not yet
# characterised, so we gate on raw volts. Defaults below align with the spirit of
# tools/preflight/preflight.sh (which warns <10.8 V, fails <10.5 V); the brief
# specifies warn=11.0 / halt=10.5 as the conservative runtime defaults.
# TODO(hw-tune): resolve the %->V mapping (and reconcile preflight's 10.8 warn vs the
#   11.0 warn here) once a discharge curve for the pack is measured on the robot.
DEFAULT_BATT_WARN = 11.0
DEFAULT_BATT_HALT = 10.5


def notify(
    message,
    *,
    title=None,
    priority=None,
    tags=None,
    topic=None,
    server=DEFAULT_SERVER,
    timeout=5,
):
    """POST `message` to `{server}/{topic}` as an ntfy notification.

    Returns True on an HTTP 2xx response, False otherwise. Never raises: a falsy
    `topic`, a network error, or a bad response is logged and turns into False so
    the caller (sequencer) is never interrupted by a notification failure.

    `tags` may be a list/tuple of strings or a comma-joined string.
    """
    if not topic:
        log.info("ntfy disabled (no topic configured) — message not sent: %r", message)
        return False

    url = "{}/{}".format(server.rstrip("/"), topic)
    try:
        body = message.encode("utf-8") if isinstance(message, str) else bytes(message)
    except Exception as exc:  # noqa: BLE001 - never let encoding crash the caller
        log.warning("ntfy: could not encode message (%s) — not sent", exc)
        return False

    headers = {}
    if title is not None:
        headers["X-Title"] = str(title)
    if priority is not None:
        headers["X-Priority"] = str(priority)
    if tags is not None:
        if isinstance(tags, (list, tuple)):
            headers["X-Tags"] = ",".join(str(t) for t in tags)
        else:
            headers["X-Tags"] = str(tags)

    req = urllib.request.Request(url, data=body, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            status = getattr(resp, "status", None) or resp.getcode()
            if 200 <= int(status) < 300:
                log.info("ntfy: sent to %s (HTTP %s)", url, status)
                return True
            log.warning("ntfy: non-2xx from %s (HTTP %s)", url, status)
            return False
    except urllib.error.HTTPError as exc:
        log.warning("ntfy: HTTP error %s posting to %s", exc.code, url)
        return False
    except urllib.error.URLError as exc:
        log.warning("ntfy: network error posting to %s (%s)", url, exc.reason)
        return False
    except Exception as exc:  # noqa: BLE001 - last-resort guard; must not raise
        log.warning("ntfy: unexpected error posting to %s (%s)", url, exc)
        return False


def load_ntfy_config(yaml_path):
    """Read the `ntfy:` block from an experiment.yaml.

    Returns `(server, topic)`. Tolerates a missing file, missing/empty `ntfy:`
    block, or a parse error: in those cases returns `(DEFAULT_SERVER, "")` — an
    empty topic which `notify()` treats as "disabled". Never raises.

    The expected block is:
        ntfy:
          server: https://ntfy.sh
          topic: ""
    """
    try:
        import yaml  # PyYAML; available on the NUC. Imported lazily so the rest
        # of this module (notify / batt_alert_level) works stdlib-only on the laptop.
    except Exception as exc:  # noqa: BLE001
        log.warning("ntfy: PyYAML unavailable (%s) — using defaults", exc)
        return DEFAULT_SERVER, ""

    try:
        with open(yaml_path, "r") as fh:
            data = yaml.safe_load(fh)
    except FileNotFoundError:
        log.info("ntfy: config %s not found — using defaults (disabled)", yaml_path)
        return DEFAULT_SERVER, ""
    except Exception as exc:  # noqa: BLE001 - bad YAML must not crash the caller
        log.warning("ntfy: could not read %s (%s) — using defaults", yaml_path, exc)
        return DEFAULT_SERVER, ""

    if not isinstance(data, dict):
        return DEFAULT_SERVER, ""
    block = data.get("ntfy")
    if not isinstance(block, dict):
        return DEFAULT_SERVER, ""

    server = block.get("server") or DEFAULT_SERVER
    topic = block.get("topic") or ""
    return str(server), str(topic)


def batt_alert_level(volts, warn=DEFAULT_BATT_WARN, halt=DEFAULT_BATT_HALT):
    """Classify a battery voltage into 'ok' | 'warn' | 'halt'.

    Pure helper (no side effects) the sequencer (T6) calls for the battery monitor
    (M2). Thresholds are in VOLTS because /limo_status.battery_voltage is volts.
        volts <= halt  -> 'halt'
        volts <= warn  -> 'warn'
        else           -> 'ok'

    On a non-numeric `volts` (e.g. an empty topic read) returns 'ok' so a flaky
    telemetry read never spuriously halts the batch; the caller should treat a
    missing reading separately if it wants stricter behaviour.
    """
    try:
        v = float(volts)
    except (TypeError, ValueError):
        log.warning("batt_alert_level: non-numeric volts %r — treating as 'ok'", volts)
        return "ok"
    if v <= halt:
        return "halt"
    if v <= warn:
        return "warn"
    return "ok"


def _build_arg_parser():
    p = argparse.ArgumentParser(
        description="Send an ntfy.sh notification (manual testing for the sequencer)."
    )
    p.add_argument("--message", "-m", required=True, help="notification body")
    p.add_argument("--topic", "-t", default=None, help="ntfy topic (falsy = disabled)")
    p.add_argument("--server", default=DEFAULT_SERVER, help="ntfy server base URL")
    p.add_argument("--title", default=None, help="X-Title header")
    p.add_argument("--priority", default=None, help="X-Priority header (1-5)")
    p.add_argument(
        "--tags", default=None, help="comma-separated X-Tags (e.g. 'warning,battery')"
    )
    p.add_argument(
        "--config",
        default=None,
        help="experiment.yaml to read server/topic from (overridden by --topic/--server)",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="print what would be sent without POSTing",
    )
    return p


def main(argv=None):
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    args = _build_arg_parser().parse_args(argv)

    server, topic = args.server, args.topic
    if args.config is not None:
        cfg_server, cfg_topic = load_ntfy_config(args.config)
        if args.server == DEFAULT_SERVER:
            server = cfg_server
        if not topic:
            topic = cfg_topic

    tags = args.tags.split(",") if args.tags else None

    if args.dry_run:
        print("[dry-run] would POST to: {}/{}".format(server.rstrip("/"), topic or "<none>"))
        print("[dry-run] message : {}".format(args.message))
        print("[dry-run] title   : {}".format(args.title))
        print("[dry-run] priority: {}".format(args.priority))
        print("[dry-run] tags    : {}".format(tags))
        if not topic:
            print("[dry-run] note: topic is empty -> notify() would log 'ntfy disabled' and return False")
        return 0

    ok = notify(
        args.message,
        title=args.title,
        priority=args.priority,
        tags=tags,
        topic=topic,
        server=server,
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
