#!/usr/bin/env python3
"""Run the reproducible software-readiness validation matrix.

This command deliberately distinguishes unavailable tooling from a passing test.
It rebuilds the C++ core, verifies its provenance, then exercises native tests,
the route oracle, dashboard models, simulator scenarios, and both firmware
images. Physical-board checks remain outside this command.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "build" / "validation_report.json"


def run(name: str, command: list[str], env: dict[str, str]) -> dict:
    started = time.time()
    result = subprocess.run(command, cwd=ROOT, env=env, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(f"[{'PASS' if result.returncode == 0 else 'FAIL'}] {name}")
    if result.returncode:
        print(result.stdout)
    return {"name": name, "command": subprocess.list2cmdline(command),
            "returncode": result.returncode, "duration_s": round(time.time() - started, 3),
            "output_tail": result.stdout[-4000:]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-firmware", action="store_true",
                        help="omit ESP32 compilation (not reported as a pass)")
    args = parser.parse_args()
    env = dict(os.environ)
    msys_bin = Path(r"C:\msys64\ucrt64\bin")
    if msys_bin.is_dir():
        env["PATH"] = str(msys_bin) + os.pathsep + env.get("PATH", "")
        env.setdefault("CXX", str(msys_bin / "g++.exe"))
    pio = shutil.which("pio", path=env["PATH"])
    results: list[dict] = []
    for name, command in [
        ("configuration", [sys.executable, "scripts/validate_config.py"]),
        ("native core fresh build", [sys.executable, "scripts/build_native_core.py", "--force"]),
        ("native core provenance", [sys.executable, "scripts/build_native_core.py", "--verify"]),
        ("route fixtures", [sys.executable, "scripts/generate_route_fixtures.py", "--check"]),
    ]:
        results.append(run(name, command, env))
    if pio:
        results.append(run("native Unity tests", [pio, "test", "-e", "native"], env))
    else:
        results.append({"name": "native Unity tests", "returncode": 127,
                        "duration_s": 0, "output_tail": "PlatformIO CLI unavailable"})
    for name, command in [
        ("Dijkstra Bellman-Ford oracle", [sys.executable, "scripts/benchmark_navigation.py", "--iterations", "100"]),
        ("dashboard model tests", [sys.executable, "-m", "unittest", "dashboard.test_models"]),
        ("dashboard JavaScript syntax", ["node", "--check", "dashboard/static/app.js"]),
        ("simulator scenarios", [sys.executable, "simulator/main.py", "--headless", "--all"]),
    ]:
        results.append(run(name, command, env))
    if not args.skip_firmware:
        if pio:
            results.append(run("ESP32-S3 production build", [pio, "run", "-e", "esp32-s3-devkitc-1"], env))
            results.append(run("ESP32-S3 mock build", [pio, "run", "-e", "esp32-s3-devkitc-1-mock"], env))
        else:
            results.append({"name": "firmware builds", "returncode": 127,
                            "duration_s": 0, "output_tail": "PlatformIO CLI unavailable"})
    failures = [result for result in results if result["returncode"] != 0]
    skipped = 2 if args.skip_firmware else 0
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                              text=True).stdout.strip()
    report = {"source_revision": revision, "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
              "python": sys.version, "passed": len(results) - len(failures), "failed": len(failures),
              "skipped": skipped, "results": results}
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"\nValidation: {report['passed']} passed, {report['failed']} failed, "
          f"{report['skipped']} skipped\nReport: {REPORT}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
