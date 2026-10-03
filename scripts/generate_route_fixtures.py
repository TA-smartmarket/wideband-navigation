#!/usr/bin/env python3
"""Generate the shared test fixtures used by both the C++ and Python tests.

The C++ navigation core is the source of truth: this script asks it to plan every
route of interest and writes the results to ``test_data/route_fixtures.json``.
The C++ fixture test re-plans those routes and compares, and the Python simulator
tests read the same file, so the two languages cannot silently drift apart.

Usage
-----
    python scripts/generate_route_fixtures.py
    python scripts/generate_route_fixtures.py --check   # fail if out of date
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_PATH = REPO_ROOT / "test_data" / "route_fixtures.json"

#: (start, destination, description) - the routes worth pinning.
ROUTE_CASES = [
    (1, 11, "default development route: entry to the middle destination"),
    (1, 12, "long diagonal route across the whole market"),
    (9, 4, "top-left corner to the checkout row"),
    (5, 5, "start equals destination"),
    (1, 2, "adjacent nodes"),
    (1, 10, "entry to the left destination"),
    (12, 1, "reverse of the long diagonal"),
    (4, 9, "checkout row to the top-left corner"),
]


def build_fixture() -> dict:
    sys.path.insert(0, str(REPO_ROOT / "simulator"))
    from smart_trolley_sim.native_core import NavigationCore

    graph_path = REPO_ROOT / "config" / "graph.json"
    graph_json = graph_path.read_text(encoding="utf-8")

    core = NavigationCore(graph_json, None)
    try:
        node_count, edge_count = core.graph_counts(graph_json)
        fixtures = []
        for start_node, destination_node, description in ROUTE_CASES:
            route, distance = core.plan_route(graph_json, start_node, destination_node)
            fixtures.append({
                "start_node": start_node,
                "destination_node": destination_node,
                "description": description,
                "route": route if route else [],
                "distance_m": round(distance, 4),
                "reachable": route is not None,
            })
    finally:
        core.close()

    return {
        "generated_by": "scripts/generate_route_fixtures.py",
        "source_of_truth": "navigation_core (C++ Dijkstra)",
        "graph_path": "config/graph.json",
        "node_count": node_count,
        "edge_count": edge_count,
        "fixtures": fixtures,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true",
                        help="do not write; fail when the file on disk differs")
    parser.add_argument("--output", default=str(FIXTURE_PATH), help="output path")
    args = parser.parse_args(argv)

    document = build_fixture()
    text = json.dumps(document, indent=2) + "\n"
    target = Path(args.output)

    if args.check:
        if not target.is_file():
            print(f"[FAIL] {target} does not exist; run this script without --check")
            return 1
        if target.read_text(encoding="utf-8") != text:
            print(f"[FAIL] {target} is out of date; re-run this script")
            return 1
        print(f"[ok] {target} is up to date")
        return 0

    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    print(f"wrote {target}")
    for fixture in document["fixtures"]:
        route = " -> ".join(str(n) for n in fixture["route"]) or "(no route)"
        print(f"  {fixture['start_node']:>2} -> {fixture['destination_node']:<2} "
              f"{fixture['distance_m']:6.2f} m  {route}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
