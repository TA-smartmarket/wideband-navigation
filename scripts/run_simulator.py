#!/usr/bin/env python3
"""Run a simulation scenario (convenience wrapper around simulator/main.py).

Kept as a separate entry point because the project structure documents
``scripts/run_simulator.py`` as the scenario runner, and because CI scripts want
a stable command that does not depend on simulator/main.py's argument layout.

Usage
-----
    python scripts/run_simulator.py --scenario 1
    python scripts/run_simulator.py --scenario 4 --headless
    python scripts/run_simulator.py --all --headless --json results/sweep.json
    python scripts/run_simulator.py --list
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MAIN = REPO_ROOT / "simulator" / "main.py"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", type=int, default=1, help="scenario index 1-12")
    parser.add_argument("--all", action="store_true", help="run every scenario (implies --headless)")
    parser.add_argument("--headless", action="store_true", help="no window")
    parser.add_argument("--json", default=None, help="write sweep results to this file")
    parser.add_argument("--list", action="store_true", help="list scenarios and exit")
    parser.add_argument("--seed", type=int, default=None, help="override the random seed")
    parser.add_argument("--speed", type=float, default=None, help="speed multiplier")
    parser.add_argument("--no-log", action="store_true", help="skip CSV/JSON logging")
    args = parser.parse_args(argv)

    if not MAIN.is_file():
        print(f"error: {MAIN} not found", file=sys.stderr)
        return 2

    command = [sys.executable, str(MAIN)]
    if args.list:
        command.append("--list")
    else:
        command += ["--scenario", str(args.scenario)]
    if args.all:
        command.append("--all")
    if args.headless or args.all:
        command.append("--headless")
    if args.json:
        command += ["--json", args.json]
    if args.seed is not None:
        command += ["--seed", str(args.seed)]
    if args.speed is not None:
        command += ["--speed", str(args.speed)]
    if args.no_log:
        command.append("--no-log")

    # Run from the repository root so relative config/log paths resolve.
    return subprocess.call(command, cwd=str(REPO_ROOT))


if __name__ == "__main__":
    raise SystemExit(main())
