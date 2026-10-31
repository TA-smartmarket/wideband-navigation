#!/usr/bin/env python3
"""Create a reproducible, conservative audit of a physical trolley log session."""

from __future__ import annotations

import argparse
import json
import statistics
from collections import Counter
from pathlib import Path
from typing import Any, Iterable


def get_path(document: dict[str, Any], *parts: str, default: Any = None) -> Any:
    value: Any = document
    for part in parts:
        if not isinstance(value, dict):
            return default
        value = value.get(part, default)
    return value


def iter_entries(path: Path) -> Iterable[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        for number, line in enumerate(handle, start=1):
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(item, dict):
                item["_line"] = number
                yield item


def packet_of(entry: dict[str, Any]) -> dict[str, Any] | None:
    packet = entry.get("packet", entry)
    return packet if isinstance(packet, dict) and "type" in packet else None


def write_report(target: Path, summary: dict[str, Any], findings: list[tuple[str, str]]) -> None:
    lines = ["# Hardware Log Audit", "", "Audit ini menganalisis telemetri serial yang direkam; ia tidak menggantikan inspeksi fisik atau verifikasi keselamatan.", "", "## Ringkasan", ""]
    for key, value in summary.items():
        if key != "states":
            lines.append(f"- {key.replace('_', ' ')}: `{value}`")
    lines.extend(["", "## State navigasi", ""])
    for state, count in summary["states"].items():
        lines.append(f"- `{state}`: {count} paket")
    lines.extend(["", "## Temuan", ""])
    if findings:
        lines.extend(f"- **{level}** — {message}" for level, message in findings)
    else:
        lines.append("- INFO — Tidak ada aturan otomatis yang terpicu. Tetap bandingkan perilaku fisik dengan log.")
    (target / "audit_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (target / "audit_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit packets.jsonl from one physical hardware session.")
    parser.add_argument("input", type=Path, help="session directory or a JSONL file")
    parser.add_argument("--output", type=Path, help="directory for report (default: input session directory)")
    args = parser.parse_args()
    source = args.input / "packets.jsonl" if args.input.is_dir() else args.input
    if not source.is_file():
        parser.error(f"cannot find JSONL input: {source}")
    output = args.output or (args.input if args.input.is_dir() else source.parent)
    output.mkdir(parents=True, exist_ok=True)

    entries = list(iter_entries(source))
    packets = [packet for entry in entries if (packet := packet_of(entry)) is not None]
    navigation = [p for p in packets if p.get("type") == "telemetry"]
    wheels = [p for p in packets if p.get("type") == "wheel_telemetry"]
    states = Counter(str(get_path(p, "navigation", "state", default="UNKNOWN")) for p in navigation)
    firmware_times = [p.get("timestamp_ms") for p in packets if isinstance(p.get("timestamp_ms"), int)]
    time_regressions = sum(b < a for a, b in zip(firmware_times, firmware_times[1:]))
    intervals = [b - a for a, b in zip(firmware_times, firmware_times[1:]) if b >= a]
    invalid = sum(not bool(p.get("position_valid")) for p in navigation)
    stale = sum(not bool(p.get("position_fresh")) for p in navigation)
    unhealthy = sum(not bool(p.get("encoder_healthy")) for p in wheels)
    tracking = sum(bool(get_path(p, side, "tracking_fault", default=False)) for p in wheels for side in ("left", "right"))
    saturated = sum(bool(get_path(p, side, "saturated", default=False)) for p in wheels for side in ("left", "right"))
    deadline_max = max((p.get("deadline_misses", 0) for p in wheels if isinstance(p.get("deadline_misses", 0), int)), default=0)
    summary: dict[str, Any] = {
        "source": str(source), "records": len(entries), "packets": len(packets),
        "navigation_packets": len(navigation), "wheel_packets": len(wheels),
        "states": dict(states), "position_invalid_packets": invalid,
        "position_stale_packets": stale, "encoder_unhealthy_packets": unhealthy,
        "wheel_tracking_fault_flags": tracking, "wheel_saturated_flags": saturated,
        "max_deadline_misses": deadline_max, "firmware_timestamp_regressions": time_regressions,
        "median_packet_interval_ms": round(statistics.median(intervals), 1) if intervals else None,
    }
    findings: list[tuple[str, str]] = []
    if not packets: findings.append(("ERROR", "Tidak ada paket JSON firmware. Periksa port, baud rate, dan kabel USB."))
    if not navigation: findings.append(("WARN", "Tidak ada telemetri navigasi; status/UWB/rute tidak bisa diaudit."))
    if not wheels: findings.append(("WARN", "Tidak ada telemetry roda; AS5600/PID/stepper tidak bisa diaudit (mungkin closed-loop tidak aktif)."))
    if invalid: findings.append(("WARN", f"{invalid} paket navigasi memiliki position_valid=false."))
    if stale: findings.append(("WARN", f"{stale} paket navigasi memiliki position_fresh=false; sumber posisi terlambat atau terputus."))
    if unhealthy: findings.append(("ERROR", f"{unhealthy} paket roda menunjukkan encoder_healthy=false; cek AS5600, magnet, I2C, dan konfigurasi arah."))
    if tracking: findings.append(("ERROR", f"{tracking} flag tracking_fault roda; motor tidak mengikuti target atau pembacaan encoder tidak masuk akal."))
    if deadline_max: findings.append(("WARN", f"Control loop mencatat hingga {deadline_max} deadline miss."))
    if time_regressions: findings.append(("WARN", f"Timestamp firmware mundur {time_regressions} kali (boot ulang atau stream tercampur)."))
    if "ERROR" in states or "EMERGENCY_STOP" in states: findings.append(("WARN", "Sesi memasuki state ERROR atau EMERGENCY_STOP; lihat raw_serial.log dan konteks fisik."))
    write_report(output, summary, findings)
    print(f"Audit written to {output / 'audit_report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
