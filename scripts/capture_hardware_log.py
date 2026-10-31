#!/usr/bin/env python3
"""Capture one physical Smart Trolley serial session without simulator data."""

from __future__ import annotations

import argparse
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
HARDWARE_LOGS = ROOT / "hardware_logs"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def safe_label(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", value.strip()).strip("_-")
    return cleaned[:48] or "hardware_run"


def make_session(label: str) -> Path:
    HARDWARE_LOGS.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = HARDWARE_LOGS / f"{stamp}_{safe_label(label)}"
    session = base
    suffix = 2
    while session.exists():
        session = HARDWARE_LOGS / f"{base.name}_{suffix:02d}"
        suffix += 1
    session.mkdir()
    return session


def decode_line(raw: bytes) -> tuple[str, dict[str, Any] | None]:
    text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
    try:
        candidate = json.loads(text)
    except json.JSONDecodeError:
        return text, None
    return text, candidate if isinstance(candidate, dict) else None


def main() -> int:
    parser = argparse.ArgumentParser(description="Record ESP32 hardware serial output into an auditable session.")
    parser.add_argument("--port", required=True, help="serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--label", default="hardware_run", help="short experiment name")
    parser.add_argument("--command", action="append", default=[], help="line to send after the port opens; repeatable")
    args = parser.parse_args()

    try:
        import serial  # type: ignore[import-not-found]
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    session = make_session(args.label)
    manifest_path = session / "manifest.json"
    started = utc_now()
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "smart_trolley_hardware_session",
        "started_at": started,
        "port": args.port,
        "baud": args.baud,
        "commands": args.command,
        "ended_at": None,
        "received_lines": 0,
        "json_packets": 0,
        "non_json_lines": 0,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Recording hardware serial to {session}")

    try:
        with (
            serial.Serial(args.port, baudrate=args.baud, timeout=0.25, write_timeout=1) as device,
            (session / "raw_serial.log").open("w", encoding="utf-8", newline="\n") as raw_log,
            (session / "packets.jsonl").open("w", encoding="utf-8", newline="\n") as packets,
        ):
            for command in args.command:
                line = command.rstrip("\r\n") + "\n"
                device.write(line.encode("utf-8"))
                packets.write(json.dumps({"captured_at": utc_now(), "direction": "tx", "raw": command}) + "\n")
            device.flush()
            while True:
                incoming = device.readline()
                if not incoming:
                    continue
                captured_at = utc_now()
                raw_log.write(incoming.decode("utf-8", errors="replace"))
                raw_log.flush()
                text, packet = decode_line(incoming)
                entry: dict[str, Any] = {"captured_at": captured_at, "direction": "rx", "raw": text}
                if packet is not None:
                    entry["packet"] = packet
                    manifest["json_packets"] += 1
                else:
                    manifest["non_json_lines"] += 1
                manifest["received_lines"] += 1
                packets.write(json.dumps(entry, allow_nan=False) + "\n")
                packets.flush()
                print(text)
    except KeyboardInterrupt:
        print("\nCapture stopped by operator.")
        return_code = 0
    except Exception as exc:  # Preserve a usable partial session on serial failures.
        print(f"Capture stopped: {exc}", file=sys.stderr)
        return_code = 1
    else:
        return_code = 0
    finally:
        manifest["ended_at"] = utc_now()
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(f"Session saved: {session}")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
