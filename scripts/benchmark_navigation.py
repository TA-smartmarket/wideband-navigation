#!/usr/bin/env python3
"""Reproducible correctness and end-to-end timing benchmark for route planning.

The measured call is the public C ABI, including graph JSON parsing.  The
firmware's already-loaded graph is faster; its pure planning time is exposed as
``plan_time_us`` in telemetry.  Bellman-Ford below is only an independent test
oracle and is never used by the application.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.smart_trolley_sim.native_core import NavigationCore  # noqa: E402


def edge_table(graph: dict) -> dict[tuple[int, int], float]:
    positions = {int(node["id"]): (float(node["x_m"]), float(node["y_m"]))
                 for node in graph["nodes"]}
    edges: dict[tuple[int, int], float] = {}
    for edge in graph["edges"]:
        source, target = int(edge["from"]), int(edge["to"])
        weight = float(edge.get("weight_m", math.dist(positions[source], positions[target])))
        edges[source, target] = weight
        if edge.get("bidirectional", True):
            edges[target, source] = weight
    return edges


def bellman_ford_distance(nodes: list[int], edges: dict[tuple[int, int], float],
                          start: int, destination: int) -> float:
    distance = {node: math.inf for node in nodes}
    distance[start] = 0.0
    for _ in range(max(0, len(nodes) - 1)):
        changed = False
        for (source, target), weight in edges.items():
            candidate = distance[source] + weight
            if candidate < distance[target]:
                distance[target] = candidate
                changed = True
        if not changed:
            break
    return distance[destination]


def percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        return 0.0
    index = min(len(sorted_values) - 1, math.ceil(fraction * len(sorted_values)) - 1)
    return sorted_values[max(0, index)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graph", default="config/graph.json")
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--json", dest="json_output")
    args = parser.parse_args()
    if args.iterations <= 0:
        parser.error("--iterations must be greater than zero")

    graph_path = ROOT / args.graph
    graph_text = graph_path.read_text(encoding="utf-8")
    graph = json.loads(graph_text)
    nodes = [int(node["id"]) for node in graph["nodes"]]
    pairs = [(start, destination) for start in nodes for destination in nodes]
    edges = edge_table(graph)

    failures: list[str] = []
    samples_us: list[float] = []
    with NavigationCore(graph_text) as core:
        for start, destination in pairs:
            route, actual = core.plan_route(graph_text, start, destination)
            expected = bellman_ford_distance(nodes, edges, start, destination)
            if route is None or route[0] != start or route[-1] != destination:
                failures.append(f"{start}->{destination}: invalid route {route}")
            elif not math.isclose(actual, expected, rel_tol=1e-6, abs_tol=1e-5):
                failures.append(f"{start}->{destination}: {actual:.6f} != {expected:.6f}")

        for start, destination in pairs:  # warm cache and dynamic loader
            core.plan_route(graph_text, start, destination)
        for _ in range(args.iterations):
            for start, destination in pairs:
                began = time.perf_counter_ns()
                core.plan_route(graph_text, start, destination)
                samples_us.append((time.perf_counter_ns() - began) / 1_000.0)

    ordered = sorted(samples_us)
    report = {
        "graph": str(graph_path.relative_to(ROOT)),
        "nodes": len(nodes),
        "edges": len(graph["edges"]),
        "all_pairs_checked": len(pairs),
        "correctness_failures": failures,
        "queries": len(samples_us),
        "timing_scope": "C ABI end-to-end, including graph JSON parsing",
        "mean_us": round(statistics.fmean(samples_us), 3),
        "median_us": round(statistics.median(samples_us), 3),
        "p95_us": round(percentile(ordered, 0.95), 3),
        "p99_us": round(percentile(ordered, 0.99), 3),
        "worst_us": round(ordered[-1], 3),
        "throughput_queries_per_second": round(1_000_000 / statistics.fmean(samples_us)),
        "status": "PASS" if not failures else "FAIL",
    }
    print(json.dumps(report, indent=2))
    if args.json_output:
        output = ROOT / args.json_output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
