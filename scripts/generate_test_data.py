#!/usr/bin/env python3
"""Generate the JSONL position-sample fixtures in test_data/.

The samples use the exact UWB contract, so they can be replayed against the
firmware (scripts/generate_mock_position.py), fed to the C++ parser tests, or
inspected by hand.  The generator is seeded, so the files are reproducible.

Usage
-----
    python scripts/generate_test_data.py
    python scripts/generate_test_data.py --check
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TEST_DATA = REPO_ROOT / "test_data"

FRAME_ID = "smart_market_map"
TROLLEY_ID = "TROLLEY_01"
SCHEMA_VERSION = 1

#: The default development route through the Smart Market graph (config/graph.json).
ROUTE = [(1.5, 2.0), (4.5, 2.0), (7.5, 2.0), (7.5, 4.0), (7.5, 6.0)]


def sample(x_m: float, y_m: float, quality: float, valid: bool, timestamp_ms: int) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "trolley_id": TROLLEY_ID,
        "frame_id": FRAME_ID,
        "timestamp_ms": timestamp_ms,
        "position": {"x_m": round(x_m, 4), "y_m": round(y_m, 4)},
        "quality": round(quality, 3),
        "valid": valid,
    }


def walk_route(rate_hz: float, speed_mps: float) -> list[tuple[float, float, int]]:
    """Points along ROUTE at the given sample rate and speed, with timestamps."""
    period_s = 1.0 / rate_hz
    step_m = speed_mps * period_s
    points: list[tuple[float, float, int]] = []
    timestamp_ms = 0
    for index in range(len(ROUTE) - 1):
        x0, y0 = ROUTE[index]
        x1, y1 = ROUTE[index + 1]
        segment = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(segment / step_m))
        for step in range(steps):
            t = step / steps
            points.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, timestamp_ms))
            timestamp_ms += int(period_s * 1000)
    points.append((ROUTE[-1][0], ROUTE[-1][1], timestamp_ms))
    return points


def build_valid(rate_hz: float = 10.0, speed_mps: float = 0.30) -> list[dict]:
    """Clean stream: low noise, always valid, high quality."""
    rng = random.Random(101)
    out = []
    for x, y, timestamp in walk_route(rate_hz, speed_mps):
        out.append(sample(x + rng.gauss(0.0, 0.02), y + rng.gauss(0.0, 0.02),
                          0.97 + rng.gauss(0.0, 0.005), True, timestamp))
    return out


def build_noise(rate_hz: float = 10.0, speed_mps: float = 0.30) -> list[dict]:
    """Moderate Gaussian noise (sigma = 0.10 m) with a matching quality drop."""
    rng = random.Random(202)
    out = []
    for x, y, timestamp in walk_route(rate_hz, speed_mps):
        out.append(sample(x + rng.gauss(0.0, 0.10), y + rng.gauss(0.0, 0.10),
                          0.82 + rng.gauss(0.0, 0.02), True, timestamp))
    return out


def build_dropout(rate_hz: float = 10.0, speed_mps: float = 0.30) -> list[dict]:
    """Sample stream with a 1.5 s gap in the middle (nothing is emitted)."""
    rng = random.Random(303)
    points = walk_route(rate_hz, speed_mps)
    out = []
    gap_start = int(len(points) * 0.4)
    gap_end = int(len(points) * 0.4) + int(1.5 * rate_hz)
    for index, (x, y, timestamp) in enumerate(points):
        if gap_start <= index < gap_end:
            continue  # dropout: no sample at all
        out.append(sample(x + rng.gauss(0.0, 0.05), y + rng.gauss(0.0, 0.05),
                          0.95 + rng.gauss(0.0, 0.01), True, timestamp))
    return out


def build_invalid(rate_hz: float = 10.0, speed_mps: float = 0.30) -> list[dict]:
    """Stream containing invalid samples and quality below the 0.60 threshold."""
    rng = random.Random(404)
    out = []
    for index, (x, y, timestamp) in enumerate(walk_route(rate_hz, speed_mps)):
        quality = 0.95
        valid = True
        if index % 17 == 5:
            valid = False           # positioning subsystem flags the sample
        if index % 23 == 7:
            quality = 0.35          # below position.minimum_quality
        out.append(sample(x + rng.gauss(0.0, 0.05), y + rng.gauss(0.0, 0.05),
                          quality, valid, timestamp))
    return out


def build_jump(rate_hz: float = 10.0, speed_mps: float = 0.30) -> list[dict]:
    """Stream where the measured position jumps 1.5 m for 2 s mid-route."""
    rng = random.Random(505)
    points = walk_route(rate_hz, speed_mps)
    out = []
    jump_start = int(len(points) * 0.5)
    jump_end = jump_start + int(2.0 * rate_hz)
    for index, (x, y, timestamp) in enumerate(points):
        offset_x = offset_y = 0.0
        if jump_start <= index < jump_end:
            offset_x, offset_y = 1.2, 0.9
        out.append(sample(x + rng.gauss(0.0, 0.05) + offset_x,
                          y + rng.gauss(0.0, 0.05) + offset_y,
                          0.95 + rng.gauss(0.0, 0.01), True, timestamp))
    return out


FILES = {
    "uwb_valid.jsonl": build_valid,
    "uwb_noise.jsonl": build_noise,
    "uwb_dropout.jsonl": build_dropout,
    "uwb_invalid.jsonl": build_invalid,
    "uwb_jump.jsonl": build_jump,
}


def build_all() -> dict[str, str]:
    documents: dict[str, str] = {}
    for name, builder in FILES.items():
        lines = [json.dumps(entry, separators=(",", ":")) for entry in builder()]
        documents[name] = "\n".join(lines) + "\n"
    return documents


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true",
                        help="do not write; fail when a file differs")
    args = parser.parse_args(argv)

    documents = build_all()
    failures = 0
    for name, text in documents.items():
        target = TEST_DATA / name
        if args.check:
            if not target.is_file() or target.read_text(encoding="utf-8") != text:
                print(f"[FAIL] {target} is out of date")
                failures += 1
            else:
                print(f"[ok]   {target.name}")
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding="utf-8")
        print(f"wrote {target} ({text.count(chr(10))} samples)")

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
