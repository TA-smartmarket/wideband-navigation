#!/usr/bin/env python3
"""Smart Trolley 2D navigation simulator (entry point).

Runs the desktop simulator: a differential-drive trolley follows a Dijkstra
route through the Smart Market graph while receiving noisy UWB position
samples.  All navigation decisions come from the shared C++ core (see
``smart_trolley_sim/native_core.py``); this program owns only the physics, the
sensor model and the rendering.

Usage
-----
    python simulator/main.py                       # interactive, default scenario
    python simulator/main.py --scenario 3           # start on scenario 3
    python simulator/main.py --scenario 5          # start on scenario 5
    python simulator/main.py --list                # list scenarios
    python simulator/main.py --headless --scenario 1
    python simulator/main.py --headless --all      # sweep every scenario
    python simulator/main.py --headless --all --json results/sweep.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Make the package importable when run as a script (python simulator/main.py).
sys.path.insert(0, str(Path(__file__).resolve().parent))

from smart_trolley_sim.config import load_simulator_config  # noqa: E402
from smart_trolley_sim.map_loader import load_market_map  # noqa: E402
from smart_trolley_sim.native_core import core_is_built  # noqa: E402
from smart_trolley_sim.scenario import (  # noqa: E402
    builtin_scenarios,
    find_scenario,
    scenario_by_index,
)
from smart_trolley_sim.simulation import SimulationEngine  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Smart Trolley autonomous navigation simulator (UWB + Dijkstra).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--scenario", type=int, default=1,
                        help="scenario index (default: 1); --list shows all")
    parser.add_argument("--all", action="store_true",
                        help="headless: run every scenario and print a summary table")
    parser.add_argument("--headless", action="store_true",
                        help="run without a window (metrics + CSV/JSON export only)")
    parser.add_argument("--json", type=str, default=None,
                        help="headless: write the sweep results to this JSON file")
    parser.add_argument("--list", action="store_true", help="list scenarios and exit")
    parser.add_argument("--list-nodes", action="store_true",
                        help="list graph nodes with their names and exit")
    parser.add_argument("--seed", type=int, default=None, help="override the random seed")
    parser.add_argument("--speed", type=float, default=None,
                        help="simulation speed multiplier (0.5, 1, 2, 5)")
    parser.add_argument("--no-log", action="store_true", help="do not write CSV/JSON logs")
    parser.add_argument("--no-build", action="store_true",
                        help="fail instead of compiling the native core if it is missing")
    # Custom destination selection, for scripted experiments and for driving any
    # point on the map without touching the scenario definitions.
    parser.add_argument("--destination", type=str, default=None,
                        help="destination: a graph node id (4) or a node name "
                             "(produk-susu); overrides the scenario. "
                             "--list-nodes shows the names")
    parser.add_argument("--destination-at", type=float, nargs=2, default=None,
                        metavar=("X_M", "Y_M"),
                        help="destination as map coordinates; the nearest graph node is used")
    parser.add_argument("--start-at", type=float, nargs=2, default=None,
                        metavar=("X_M", "Y_M"),
                        help="start position as map coordinates (overrides the scenario)")
    parser.add_argument("--scene", type=str, default=None,
                        help="saved GET /api/v1/scene response for obstacle-aware planning")
    parser.add_argument("--obstacle-clearance", type=float, default=None, metavar="METERS",
                        help="inflate each scene obstacle by this safety clearance")
    return parser.parse_args(argv)


def resolve_destination(engine, value: str) -> int | None:
    """Resolve a --destination value to a node id.

    Accepts a numeric id or a node name from graph.json (case-insensitive).
    Returns None and explains why when it cannot be resolved, so a typo is a
    clear message rather than a silent no-op.
    """
    text = str(value).strip()
    if not text:
        return None
    try:
        return int(text)
    except ValueError:
        pass
    node_id = engine.market_map.node_id_by_name(text)
    if node_id is None:
        known = ", ".join(sorted(engine.market_map.node_names)) or "none defined"
        print(f'error: unknown destination "{text}". Known names: {known}',
              file=sys.stderr)
        return None
    return node_id


def apply_destination_overrides(engine, args) -> bool:
    """Apply --start-at / --destination / --destination-at to a fresh engine.

    Returns False when a destination was requested but could not be resolved, so
    the caller can abort instead of silently running the scenario default.
    """
    if args.start_at is not None:
        engine.set_start_position(args.start_at[0], args.start_at[1])
        engine.state.notes.append(
            f"start overridden to ({args.start_at[0]:.2f}, {args.start_at[1]:.2f})")
    if args.destination is not None:
        node_id = resolve_destination(engine, args.destination)
        if node_id is None:
            return False
        if engine.set_destination(node_id):
            engine.state.notes.append(
                f"destination overridden to {engine.market_map.node_label(node_id)} "
                f"(node {node_id})")
        else:
            print(f"error: cannot set destination to node {node_id}", file=sys.stderr)
            return False
    elif args.destination_at is not None:
        engine.pick_destination_at(args.destination_at[0], args.destination_at[1])
    return True


def print_nodes(market_map) -> None:
    """List graph nodes with their names, for --list-nodes."""
    print(f"{len(market_map.nodes)} nodes in {market_map.map_id}:")
    for node in sorted(market_map.nodes.values(), key=lambda n: n.node_id):
        label = node.name or "(unnamed)"
        print(f"  {node.node_id:3d}  {label:20s} {node.type:13s} "
              f"({node.x_m:5.1f}, {node.y_m:5.1f}) m")


def print_scenarios() -> None:
    print("Available scenarios:\n")
    for index, scenario in enumerate(builtin_scenarios(), start=1):
        print(f"  {index:2d}. {scenario.name}")
        print(f"      {scenario.description}")
        print(f"      start ({scenario.start_x_m:.1f}, {scenario.start_y_m:.1f}) -> "
              f"node {scenario.destination_node}   noise sigma={scenario.noise_std_m:.2f} m")
        if scenario.expectation:
            print(f"      expected: {scenario.expectation}")
    print("\nRun:  python simulator/main.py --scenario 3")


def run_headless(args: argparse.Namespace) -> int:
    config = load_simulator_config()
    if args.scene is not None:
        config.scene_path = args.scene
    if args.obstacle_clearance is not None:
        config.obstacle_clearance_m = args.obstacle_clearance
    if args.seed is not None:
        config.simulation.random_seed = args.seed
    if args.no_log:
        config.logging.enabled = False

    scenarios = builtin_scenarios() if args.all else [scenario_by_index(args.scenario)]
    scenarios = [s for s in scenarios if s is not None]
    if not scenarios:
        print(f"error: no scenario for index {args.scenario}", file=sys.stderr)
        return 2

    results = []
    failures = 0
    for scenario in scenarios:
        engine = SimulationEngine(scenario, config)
        try:
            if not args.all:
                # Overrides are for a single targeted run; a full sweep keeps each
                # scenario's own start and destination.
                if not apply_destination_overrides(engine, args):
                    return 2
            metrics = engine.run_to_completion()
        finally:
            engine.close()
        results.append({
            "scenario": scenario.name,
            "description": scenario.description,
            "expectation": scenario.expectation,
            "expected_success": scenario.expect_success,
            **metrics.as_dict(),
        })
        if metrics.success:
            status = "SUCCESS"
        elif not metrics.completed:
            status = "TIMEOUT"      # ran out of simulated time without arriving
        else:
            status = "FAILED"
        efficiency = (f"{metrics.path_efficiency_percent:5.1f}%"
                      if metrics.completed else "  n/a")
        print(f"[{status:8s}] {scenario.name:38s} "
              f"state={metrics.final_state:11s} "
              f"time={metrics.completion_time_s:6.1f}s "
              f"planned={metrics.planned_distance_m:6.2f}m "
              f"travelled={metrics.ground_truth_distance_m:6.2f}m "
              f"excess={metrics.excess_distance_m:+5.2f}m "
              f"eff={efficiency} "
              f"xte={metrics.mean_cross_track_error_m:5.2f}/{metrics.max_cross_track_error_m:5.2f}m "
              f"dest_err={metrics.destination_error_m:5.2f}m "
              f"replans={metrics.replan_count} "
              f"losses={metrics.position_loss_events} "
              f"[{metrics.benchmark_overall}]")
        # A scenario marked expect_success=False is a deliberate failure drill;
        # not arriving is the correct outcome for it.  A benchmark breach is
        # reported but never counted as a failure: it is an experimental metric.
        if scenario.expect_success and not metrics.success:
            failures += 1
        if not scenario.expect_success and metrics.final_state not in ("ERROR",):
            failures += 1

    if args.json:
        target = Path(args.json)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", encoding="utf-8") as handle:
            json.dump(results, handle, indent=2)
            handle.write("\n")
        print(f"\nwrote {target}")

    print(f"\n{len(results) - failures}/{len(results)} scenarios met their expectation")
    return 1 if failures else 0


def run_interactive(args: argparse.Namespace) -> int:
    import pygame

    from smart_trolley_sim.renderer import Renderer

    config = load_simulator_config()
    if args.scene is not None:
        config.scene_path = args.scene
    if args.obstacle_clearance is not None:
        config.obstacle_clearance_m = args.obstacle_clearance
    if args.seed is not None:
        config.simulation.random_seed = args.seed
    if args.speed is not None:
        config.simulation.speed_multiplier = args.speed
    if args.no_log:
        config.logging.enabled = False

    scenario = scenario_by_index(args.scenario) or builtin_scenarios()[0]
    engine = SimulationEngine(scenario, config)
    if not apply_destination_overrides(engine, args):
        engine.close()
        return 2
    renderer = Renderer(config.window, config.render, engine.market_map)

    running = True
    accumulator = 0.0
    fps = 0.0
    aborted = False
    try:
        while running:
            frame_dt = renderer.tick(60)
            fps = 0.9 * fps + 0.1 * (1.0 / frame_dt if frame_dt > 0 else 0.0)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    running = handle_key(event.key, engine, config, pygame)
                elif event.type == pygame.MOUSEMOTION:
                    # Highlight the node a click would select, so the operator can
                    # aim before committing.
                    if renderer.is_in_map_area(event.pos):
                        world = renderer.screen_to_world(event.pos)
                        found = engine.nearest_node_to(*world)
                        renderer.pick_hover_node = found[0] if found else None
                    else:
                        renderer.pick_hover_node = None
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if renderer.is_in_map_area(event.pos):
                        world = renderer.screen_to_world(event.pos)
                        if event.button == 1:
                            # Left click: retarget to the nearest graph node.
                            engine.pick_destination_at(*world)
                        elif event.button == 3:
                            # Right click: start / resume navigation.
                            engine.start_navigation()
                        elif event.button == 2:
                            # Middle click: cancel navigation.
                            engine.cancel()

            # Fixed-step simulation with an accumulator: physics never depends on
            # the render frame rate, so a slow frame cannot change the trajectory.
            if not engine.state.paused and not engine.state.finished:
                accumulator += frame_dt * config.simulation.speed_multiplier
                steps = 0
                while accumulator >= config.simulation.dt_s and steps < 200:
                    engine.step()
                    accumulator -= config.simulation.dt_s
                    steps += 1
                    if engine.state.finished:
                        break

            renderer.draw(engine, fps)
            if engine.state.paused:
                renderer.draw_overlay_message("PAUSED  (SPACE to resume)")
            elif engine.state.finished:
                verdict = "ARRIVED" if engine.metrics.success else engine.metrics.final_state
                renderer.draw_overlay_message(
                    f"{verdict}  (R to reset, ESC to quit)",
                    color=(110, 210, 140) if engine.metrics.success else (235, 190, 90),
                )
            pygame.display.flip()
    except BaseException:
        # A fault in the simulator itself (renderer, input handling) is NOT a
        # navigation failure.  Do not export a summary that would claim the run
        # "FAILED" with zero distance: that misrepresents the experiment.  The
        # exception is re-raised so the cause stays visible.
        aborted = True
        raise
    finally:
        # ORDER MATTERS: finalize() reads the navigation session (it queries the
        # core for the final state), so the native session must still be open when
        # it runs.  Closing first made a normal window-close raise
        # "nav_get_status failed".
        try:
            if not aborted:
                metrics = engine.finalize()
                print()
                print(metrics.as_report())
                print()
                print(metrics.benchmark_report())
            else:
                print("\nSimulator aborted before completion; no summary written.",
                      file=sys.stderr)
        finally:
            engine.close()
            pygame.quit()
    return 0


def handle_key(key: int, engine: SimulationEngine, config, pygame) -> bool:
    """Handle one key press.  Returns False when the window should close."""
    if key == pygame.K_ESCAPE:
        return False

    if pygame.K_1 <= key <= pygame.K_9:
        index = key - pygame.K_1 + 1
        scenario = scenario_by_index(index)
        if scenario is not None:
            engine.__exit__()  # release the native session before replacing it
            new_engine = SimulationEngine(scenario, config)
            # Copy the fresh engine state into the live object so the renderer
            # keeps working with the same reference.
            engine.__dict__.update(new_engine.__dict__)
            engine.state.notes.append(f"scenario switched to {scenario.name}")
    elif key == pygame.K_SPACE:
        engine.toggle_pause()
    elif key == pygame.K_r:
        engine.core.reset()
        engine._configure_from_scenario()
        engine.events = engine.scenario.resolve_events()
        engine.metrics_accumulator.reset()
        engine.logger.records.clear()
        engine.uwb.reset(now_ms=0)
        engine.state = type(engine.state)(current_scenario=engine.scenario.name)
    elif key == pygame.K_p:
        engine.replan()
    elif key == pygame.K_z:
        # Cycle the destination through the selectable graph nodes: this is how
        # an arbitrary route is driven without editing configuration or scripts.
        engine.cycle_destination(-1)
    elif key == pygame.K_x:
        engine.cycle_destination(+1)
    elif key in (pygame.K_RETURN, pygame.K_KP_ENTER):
        # Explicit start: required after an emergency stop, and it re-validates
        # the position and route before the trolley may move again.
        engine.start_navigation()
    elif key == pygame.K_n:
        engine.toggle_noise()
    elif key == pygame.K_d:
        engine.toggle_dropout()
    elif key == pygame.K_j:
        engine.trigger_position_jump()
    elif key == pygame.K_i:
        engine.toggle_invalid()
    elif key == pygame.K_q:
        engine.toggle_low_quality()
    elif key == pygame.K_f:
        engine.toggle_frozen()
    elif key == pygame.K_e:
        engine.emergency_stop()
    elif key == pygame.K_c:
        # Clear ONLY.  Resuming is a separate, explicit action (ENTER) because
        # the emergency stop must never transition straight back into motion:
        # the position and the route have to be re-validated first.
        engine.clear_emergency_stop()
    elif key == pygame.K_g:
        config.render.show_graph_labels = not config.render.show_graph_labels
    elif key == pygame.K_t:
        config.render.show_trajectory = not config.render.show_trajectory
    elif key == pygame.K_h:
        config.render.show_debug_overlay = not config.render.show_debug_overlay
    elif key == pygame.K_s:
        engine.start_navigation()
    return True


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    if args.list_nodes:
        config = load_simulator_config()
        print_nodes(load_market_map(config.map_path, config.graph_path))
        return 0
    if args.list:
        print_scenarios()
        return 0

    if not core_is_built():
        print("Native navigation core not found; building it (first run only)...")
        from smart_trolley_sim.native_core import build_native_core

        path = build_native_core(verbose=False)
        print(f"built {path}")

    if args.headless:
        return run_headless(args)
    return run_interactive(args)


if __name__ == "__main__":
    raise SystemExit(main())
