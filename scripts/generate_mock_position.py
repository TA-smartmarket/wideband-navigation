#!/usr/bin/env python3
"""Send mock UWB position JSON to the ESP32-S3 over a serial port.

This is the hardware-in-the-loop tool: it lets the real firmware be exercised
before the UWB positioning subsystem exists.  The output uses exactly the
documented contract, so nothing in the firmware changes when the real sensor
replaces this generator.

Examples
--------
    # Walk a straight line across the market, 10 Hz, COM5
    python scripts/generate_mock_position.py --port COM5 --trajectory line

    # Follow the planned route through the graph nodes
    python scripts/generate_mock_position.py --port COM5 --trajectory route

    # Replay a trajectory recorded by the simulator
    python scripts/generate_mock_position.py --port COM5 --csv logs/scenario_01_run001_*.csv

    # Add faults: 0.10 m noise, 5% dropout, 2% invalid samples
    python scripts/generate_mock_position.py --port COM5 --noise 0.10 --dropout 0.05 --invalid 0.02

    # Dry run: print what would be sent, no serial port needed
    python scripts/generate_mock_position.py --dry-run --duration 3

Windows note: use COM3 / COM4 / COM5, not /dev/ttyUSB0.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FRAME_ID = "smart_market_map"
SCHEMA_VERSION = 1
TROLLEY_ID = "TROLLEY_01"


def load_route(graph_path: Path, start_node: int, destination_node: int) -> list[tuple[float, float]]:
    """Shortest route through the graph, planned by the C++ core.

    The route comes from the same Dijkstra implementation the firmware runs, so
    the generated trajectory is exactly the one the device would plan itself.
    """
    sys.path.insert(0, str(REPO_ROOT / "simulator"))
    from smart_trolley_sim.native_core import NavigationCore  # local import: optional dependency

    graph_json = graph_path.read_text(encoding="utf-8")
    core = NavigationCore(graph_json, None)
    try:
        nodes, distance = core.plan_route(graph_json, start_node, destination_node)
        if not nodes:
            raise SystemExit(f"no route from node {start_node} to node {destination_node}")
        document = json.loads(graph_json)
        coordinates = {
            int(node["id"]): (float(node["x_m"]), float(node["y_m"]))
            for node in document["nodes"]
        }
        print(f"route {' -> '.join(str(n) for n in nodes)} ({distance:.2f} m)")
        return [coordinates[n] for n in nodes]
    finally:
        core.close()


def load_csv_trajectory(csv_path: Path) -> list[tuple[float, float]]:
    """Read true positions from a simulator log (logs/*.csv)."""
    points: list[tuple[float, float]] = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or "true_x_m" not in reader.fieldnames:
            raise SystemExit(f"{csv_path}: not a simulator telemetry log (no true_x_m column)")
        for row in reader:
            try:
                points.append((float(row["true_x_m"]), float(row["true_y_m"])))
            except (KeyError, ValueError):
                continue
    if not points:
        raise SystemExit(f"{csv_path}: no usable rows")
    return points


def build_trajectory(name: str, args: argparse.Namespace) -> list[tuple[float, float]]:
    if name == "line":
        steps = 200
        return [(0.8 + 10.0 * i / steps, 0.9 + 6.0 * i / steps) for i in range(steps + 1)]
    if name == "square":
        side = 4.0
        origin = (1.5, 2.0)
        points: list[tuple[float, float]] = []
        corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
        for index in range(len(corners) - 1):
            start = corners[index]
            end = corners[index + 1]
            for step in range(40):
                t = step / 40.0
                points.append((origin[0] + start[0] + (end[0] - start[0]) * t,
                               origin[1] + start[1] + (end[1] - start[1]) * t))
        return points
    if name == "route":
        graph_path = REPO_ROOT / args.graph
        return load_route(graph_path, args.start_node, args.destination)
    if name == "hold":
        return [(args.x, args.y)] * 200
    raise SystemExit(f"unknown trajectory '{name}'")


def densify(points: list[tuple[float, float]], spacing_m: float) -> list[tuple[float, float]]:
    """Interpolate a waypoint list into a smooth stream at `spacing_m` steps."""
    if len(points) < 2 or spacing_m <= 0.0:
        return points
    dense = [points[0]]
    for index in range(1, len(points)):
        x0, y0 = points[index - 1]
        x1, y1 = points[index]
        segment = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(segment / spacing_m))
        for step in range(1, steps + 1):
            t = step / steps
            dense.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return dense


def make_line(x_m: float, y_m: float, quality: float, valid: bool, timestamp_ms: int) -> str:
    """Serialise one position sample exactly as documented in docs/data_contract.md."""
    return json.dumps({
        "schema_version": SCHEMA_VERSION,
        "trolley_id": TROLLEY_ID,
        "frame_id": FRAME_ID,
        "timestamp_ms": timestamp_ms,
        "position": {"x_m": round(x_m, 4), "y_m": round(y_m, 4)},
        "quality": round(quality, 3),
        "valid": valid,
    }, separators=(",", ":"))


def open_port(port: str, baud: int):
    try:
        import serial  # pyserial
    except ImportError:
        raise SystemExit(
            "pyserial is required for serial output. Install it with:\n"
            "    python -m pip install pyserial\n"
            "or use --dry-run to preview the stream without a port."
        ) from None
    try:
        return serial.Serial(port, baud, timeout=1)
    except Exception as exc:  # noqa: BLE001 - surface the OS error verbatim
        raise SystemExit(f"cannot open {port}: {exc}") from None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default=None, help="serial port (Windows: COM3, COM4, COM5)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate (default 115200)")
    parser.add_argument("--trajectory", default="route",
                        choices=["route", "line", "square", "hold", "csv"],
                        help="trajectory generator (default: route)")
    parser.add_argument("--csv", default=None, help="CSV log to replay (with --trajectory csv)")
    parser.add_argument("--graph", default="config/graph.json", help="graph file for --trajectory route")
    parser.add_argument("--start-node", type=int, default=1, help="route start node")
    parser.add_argument("--destination", type=int, default=11, help="route destination node")
    parser.add_argument("--x", type=float, default=0.8, help="position for --trajectory hold")
    parser.add_argument("--y", type=float, default=0.9, help="position for --trajectory hold")

    parser.add_argument("--rate", type=float, default=10.0, help="sample rate in Hz (default 10)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="stop after N seconds (0 = until the trajectory ends)")
    parser.add_argument("--loop", action="store_true", help="repeat the trajectory forever")
    parser.add_argument("--speed", type=float, default=0.45,
                        help="assumed trolley speed in m/s, used to time the trajectory")
    parser.add_argument("--spacing", type=float, default=0.05,
                        help="interpolation step in meters (default 0.05)")

    parser.add_argument("--noise", type=float, default=0.0,
                        help="Gaussian position noise sigma in meters")
    parser.add_argument("--dropout", type=float, default=0.0,
                        help="probability of dropping a sample (0..1)")
    parser.add_argument("--dropout-burst", type=float, default=0.0,
                        help="after a dropped sample, suppress output for N seconds")
    parser.add_argument("--invalid", type=float, default=0.0,
                        help="probability of marking a sample valid=false (0..1)")
    parser.add_argument("--low-quality", type=float, default=0.0,
                        help="probability of reporting a quality below the threshold (0..1)")
    parser.add_argument("--quality", type=float, default=0.95, help="nominal quality value")
    parser.add_argument("--seed", type=int, default=12345, help="random seed (deterministic faults)")

    parser.add_argument("--command", default=None,
                        help="send a CMD: line first, e.g. --command 'destination 11'")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the JSON lines instead of writing to a serial port")
    parser.add_argument("--quiet", action="store_true", help="suppress progress output")
    args = parser.parse_args(argv)

    rng = random.Random(args.seed)

    if args.trajectory == "csv":
        if not args.csv:
            raise SystemExit("--trajectory csv requires --csv <file>")
        points = load_csv_trajectory(Path(args.csv))
    else:
        points = build_trajectory(args.trajectory, args)
    points = densify(points, args.spacing)
    if not points:
        raise SystemExit("empty trajectory")

    # The trajectory advances one point per sample, so the realised speed is
    # spacing * rate.  Report it: a mismatch with --speed is the most common
    # reason a hardware-in-the-loop run "moves too fast".
    period_s = 1.0 / max(args.rate, 0.1)
    implied_speed = args.spacing / period_s
    if not args.quiet:
        print(f"trajectory: {args.trajectory}, {len(points)} samples at {args.rate:.1f} Hz")
        print(f"implied speed: {implied_speed:.2f} m/s "
              f"(spacing {args.spacing:.3f} m, period {period_s * 1000:.0f} ms)")
        if args.duration > 0:
            print(f"duration: {args.duration:.1f} s")
        print(f"faults: noise={args.noise:.3f} m dropout={args.dropout:.3f} "
              f"invalid={args.invalid:.3f} low_quality={args.low_quality:.3f}")
        if abs(implied_speed - args.speed) > 0.05:
            print(f"note: --speed {args.speed:.2f} m/s differs from the realised "
                  f"{implied_speed:.2f} m/s; adjust --spacing or --rate to change it")

    port = None
    if not args.dry_run:
        if not args.port:
            raise SystemExit("--port is required unless --dry-run is used")
        port = open_port(args.port, args.baud)
        if not args.quiet:
            print(f"opened {args.port} @ {args.baud} baud")
        if args.command:
            port.write(f"CMD:{args.command}\n".encode("ascii"))
            port.flush()
            if not args.quiet:
                print(f"sent: CMD:{args.command}")
            time.sleep(0.3)

    started = time.monotonic()
    sent = 0
    dropped = 0
    invalid = 0
    low_quality = 0
    suppress_until = 0.0
    index = 0

    try:
        while True:
            now = time.monotonic()
            elapsed = now - started
            if args.duration > 0 and elapsed >= args.duration:
                break
            if index >= len(points):
                if not args.loop:
                    break
                index = 0

            if now < suppress_until:
                index += 1
                dropped += 1
            elif args.dropout > 0.0 and rng.random() < args.dropout:
                dropped += 1
                index += 1
                if args.dropout_burst > 0.0:
                    suppress_until = now + args.dropout_burst
            else:
                x_m, y_m = points[index]
                if args.noise > 0.0:
                    x_m += rng.gauss(0.0, args.noise)
                    y_m += rng.gauss(0.0, args.noise)

                quality = args.quality + rng.gauss(0.0, 0.01)
                if args.low_quality > 0.0 and rng.random() < args.low_quality:
                    quality = 0.30
                    low_quality += 1
                quality = max(0.0, min(1.0, quality))

                valid = True
                if args.invalid > 0.0 and rng.random() < args.invalid:
                    valid = False
                    invalid += 1

                timestamp_ms = int(elapsed * 1000.0)
                line = make_line(x_m, y_m, quality, valid, timestamp_ms)
                if port is not None:
                    port.write((line + "\n").encode("ascii"))
                elif not args.quiet or sent < 5:
                    print(line)
                sent += 1
                index += 1

            # Pace the loop against a monotonic deadline so drift cannot
            # accumulate over a long run.
            target = started + (sent + dropped) * period_s
            sleep_s = target - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        if not args.quiet:
            print("\ninterrupted")
    finally:
        if port is not None:
            port.close()

    if not args.quiet:
        print(f"sent {sent} samples, dropped {dropped}, invalid {invalid}, "
              f"low quality {low_quality}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
