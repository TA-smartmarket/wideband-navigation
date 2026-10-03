"""Pygame rendering for the Smart Trolley simulator.

The renderer is a pure observer: it reads simulation state and never writes back
into navigation.  World coordinates are meters in the ``smart_market_map`` frame
(origin bottom-left, +X right, +Y up); screen coordinates are pixels with the
origin top-left, so the vertical axis is flipped here and *only* here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import pygame

from .config import RenderConfig, WindowConfig
from .map_loader import MarketMap
from .native_core import NavState
from .simulation import SimulationEngine

# --- palette ---------------------------------------------------------------
COLOR_BACKGROUND = (18, 20, 24)
COLOR_PANEL = (26, 29, 35)
COLOR_GRID = (38, 42, 50)
COLOR_MAP_BORDER = (90, 98, 112)
COLOR_EDGE = (78, 86, 100)
COLOR_NODE = (150, 160, 178)
COLOR_NODE_LABEL = (196, 204, 216)
COLOR_DESTINATION = (86, 196, 128)
COLOR_ENTRY = (90, 160, 230)
COLOR_PATH = (232, 178, 64)
COLOR_PATH_COMPLETED = (120, 132, 150)
COLOR_ACTIVE_WAYPOINT = (255, 226, 120)
COLOR_PICK = (255, 120, 220)
COLOR_TRAJECTORY = (90, 190, 220)
COLOR_TROLLEY = (240, 244, 250)
COLOR_TROLLEY_MEASURED = (232, 96, 96)
COLOR_TROLLEY_NAV = (96, 200, 232)
COLOR_TEXT = (222, 228, 238)
COLOR_TEXT_DIM = (150, 158, 172)
COLOR_OK = (110, 210, 140)
COLOR_WARN = (235, 190, 90)
COLOR_ERROR = (235, 96, 96)

NODE_COLORS = {
    "destination": COLOR_DESTINATION,
    "entry": COLOR_ENTRY,
    "parking": (150, 130, 220),
    "checkout": (220, 150, 90),
    "exit": (200, 120, 180),
}


@dataclass
class Viewport:
    """Maps the map rectangle onto the pygame surface."""

    width_px: int
    height_px: int
    margin_px: int
    map_min_x: float
    map_min_y: float
    map_max_x: float
    map_max_y: float

    @classmethod
    def fit(cls, window: WindowConfig, market_map: MarketMap) -> "Viewport":
        min_x, min_y, max_x, max_y = market_map.bounds()
        return cls(
            width_px=window.width_px - window.panel_width_px,
            height_px=window.height_px,
            margin_px=window.margin_px,
            map_min_x=min_x,
            map_min_y=min_y,
            map_max_x=max_x,
            map_max_y=max_y,
        )

    @property
    def scale_px_per_m(self) -> float:
        usable_w = max(1, self.width_px - 2 * self.margin_px)
        usable_h = max(1, self.height_px - 2 * self.margin_px)
        span_x = max(1e-6, self.map_max_x - self.map_min_x)
        span_y = max(1e-6, self.map_max_y - self.map_min_y)
        return min(usable_w / span_x, usable_h / span_y)

    def to_screen(self, x_m: float, y_m: float) -> tuple[int, int]:
        scale = self.scale_px_per_m
        # Centre the scaled map inside the viewport.
        drawn_w = (self.map_max_x - self.map_min_x) * scale
        drawn_h = (self.map_max_y - self.map_min_y) * scale
        offset_x = self.margin_px + 0.5 * ((self.width_px - 2 * self.margin_px) - drawn_w)
        offset_y = self.margin_px + 0.5 * ((self.height_px - 2 * self.margin_px) - drawn_h)
        screen_x = offset_x + (x_m - self.map_min_x) * scale
        # Flip Y: world +Y is up, screen +Y is down.
        screen_y = offset_y + (self.map_max_y - y_m) * scale
        return int(round(screen_x)), int(round(screen_y))

    def to_world(self, screen_x: int, screen_y: int) -> tuple[float, float]:
        scale = self.scale_px_per_m
        drawn_w = (self.map_max_x - self.map_min_x) * scale
        drawn_h = (self.map_max_y - self.map_min_y) * scale
        offset_x = self.margin_px + 0.5 * ((self.width_px - 2 * self.margin_px) - drawn_w)
        offset_y = self.margin_px + 0.5 * ((self.height_px - 2 * self.margin_px) - drawn_h)
        x_m = self.map_min_x + (screen_x - offset_x) / scale
        y_m = self.map_max_y - (screen_y - offset_y) / scale
        return x_m, y_m

    def meters_to_px(self, meters: float) -> int:
        return max(1, int(round(meters * self.scale_px_per_m)))


class Renderer:
    """Draws the map, the planned route, the trolley and the status panels."""

    def __init__(self, window: WindowConfig, render: RenderConfig, market_map: MarketMap):
        self.window = window
        self.config = render
        self.market_map = market_map
        self.viewport = Viewport.fit(window, market_map)
        # display.set_mode() initialises the display; the font module must be
        # initialised separately (it is not implied) before SysFont is usable.
        if not pygame.get_init():
            pygame.init()
        if not pygame.font.get_init():
            pygame.font.init()
        self.screen = pygame.display.set_mode((window.width_px, window.height_px))
        pygame.display.set_caption(window.caption)
        # Monospace keeps the numeric columns of the status panel aligned.
        # SysFont falls back to the pygame default when none of the names match.
        self.font_small = pygame.font.SysFont("consolas,couriernew,monospace", 13)
        self.font = pygame.font.SysFont("consolas,couriernew,monospace", 15)
        self.font_bold = pygame.font.SysFont("consolas,couriernew,monospace", 16, bold=True)
        self.font_title = pygame.font.SysFont("consolas,couriernew,monospace", 18, bold=True)
        self.clock = pygame.time.Clock()
        #: Node a click would select, or None.  Set by the interactive loop.
        self.pick_hover_node: int | None = None
        #: Layout diagnostics: how many panel lines were drawn / dropped.
        self.panel_lines_drawn = 0
        self.panel_lines_skipped = 0

    # -- top level ---------------------------------------------------------
    def draw(self, engine: SimulationEngine, fps: float) -> None:
        self.screen.fill(COLOR_BACKGROUND)
        self._draw_map_area(engine)
        self._draw_panel(engine, fps)
        pygame.display.flip()

    def tick(self, fps_cap: int = 60) -> float:
        return self.clock.tick(fps_cap) / 1000.0

    def is_in_map_area(self, position: tuple[int, int]) -> bool:
        """True when a screen position is inside the map viewport (not the panel)."""
        return 0 <= position[0] < self.viewport.width_px

    def screen_to_world(self, position: tuple[int, int]) -> tuple[float, float]:
        """Convert a screen position to meters in the map frame."""
        return self.viewport.to_world(position[0], position[1])

    # -- map ---------------------------------------------------------------
    def _draw_map_area(self, engine: SimulationEngine) -> None:
        viewport = self.viewport
        # Map rectangle.
        min_x, min_y, max_x, max_y = self.market_map.bounds()
        top_left = viewport.to_screen(min_x, max_y)
        bottom_right = viewport.to_screen(max_x, min_y)
        rect = pygame.Rect(
            top_left[0], top_left[1],
            bottom_right[0] - top_left[0], bottom_right[1] - top_left[1],
        )
        pygame.draw.rect(self.screen, COLOR_PANEL, rect)
        self._draw_grid(rect)

        self._draw_edges()
        self._draw_route(engine)
        if self.config.show_trajectory:
            self._draw_trajectory(engine)
        self._draw_nodes(self.pick_hover_node)
        self._draw_trolley(engine)
        self._draw_node_labels()
        self._draw_map_caption(rect)

    def _draw_grid(self, rect: pygame.Rect) -> None:
        """One meter reference grid (purely visual, never used by navigation)."""
        scale = self.viewport.scale_px_per_m
        if scale < 8:
            return
        min_x, min_y, max_x, max_y = self.market_map.bounds()
        x = math.ceil(min_x)
        while x <= max_x:
            p0 = self.viewport.to_screen(x, min_y)
            p1 = self.viewport.to_screen(x, max_y)
            pygame.draw.line(self.screen, COLOR_GRID, p0, p1, 1)
            x += 1
        y = math.ceil(min_y)
        while y <= max_y:
            p0 = self.viewport.to_screen(min_x, y)
            p1 = self.viewport.to_screen(max_x, y)
            pygame.draw.line(self.screen, COLOR_GRID, p0, p1, 1)
            y += 1
        pygame.draw.rect(self.screen, COLOR_MAP_BORDER, rect, 2)

    def _draw_edges(self) -> None:
        for edge in self.market_map.edges:
            polyline = self.market_map.edge_polyline(edge)
            if polyline is None:
                continue
            a, b = polyline
            pygame.draw.line(
                self.screen, COLOR_EDGE,
                self.viewport.to_screen(*a), self.viewport.to_screen(*b), 2,
            )

    def _draw_route(self, engine: SimulationEngine) -> None:
        route = engine.planned_route()
        if len(route) < 2:
            return
        points = [self.viewport.to_screen(*p) for p in self.market_map.route_polyline(route)]
        # Completed portion: from the route start up to the active waypoint.
        snapshot = engine.core.status()
        active_index = max(0, snapshot.waypoint_index)
        # The route list includes the snapped start node, so the completed part is
        # everything before the leg currently being driven.
        completed_end = min(len(points) - 1, active_index)
        if completed_end >= 1:
            pygame.draw.lines(self.screen, COLOR_PATH_COMPLETED, False,
                              points[: completed_end + 1], 3)
        if completed_end < len(points) - 1:
            pygame.draw.lines(self.screen, COLOR_PATH, False, points[completed_end:], 3)
        # Destination marker.
        destination = points[-1]
        pygame.draw.circle(self.screen, COLOR_DESTINATION, destination, 9, 2)
        # Active waypoint marker.
        if snapshot.active_waypoint_node >= 0:
            position = self.market_map.node_position(snapshot.active_waypoint_node)
            if position is not None:
                pygame.draw.circle(
                    self.screen, COLOR_ACTIVE_WAYPOINT,
                    self.viewport.to_screen(*position), 11, 2,
                )

    def _draw_trajectory(self, engine: SimulationEngine) -> None:
        trajectory = engine.trajectory()
        if len(trajectory) < 2:
            return
        if len(trajectory) > self.config.trajectory_max_points:
            step = len(trajectory) // self.config.trajectory_max_points + 1
            trajectory = trajectory[::step]
        points = [self.viewport.to_screen(*p) for p in trajectory]
        pygame.draw.lines(self.screen, COLOR_TRAJECTORY, False, points, 1)

    def _draw_nodes(self, pick_hover_node: int | None = None) -> None:
        for node in self.market_map.nodes.values():
            position = self.viewport.to_screen(node.x_m, node.y_m)
            color = NODE_COLORS.get(node.type, COLOR_NODE)
            radius = 6 if node.type in NODE_COLORS else 4
            if pick_hover_node is not None and node.node_id == pick_hover_node:
                # Show which node a click would select.
                pygame.draw.circle(self.screen, COLOR_PICK, position, radius + 7, 2)
                color = COLOR_PICK
            pygame.draw.circle(self.screen, color, position, radius)
            pygame.draw.circle(self.screen, COLOR_BACKGROUND, position, radius, 1)
    def _draw_node_labels(self) -> None:
        """Draw node labels on top of the trolley.

        Called after _draw_trolley() so the trolley body cannot hide the name of
        the node it is standing on.
        """
        if not self.config.show_graph_labels:
            return
        map_right = self.window.width_px - self.window.panel_width_px
        for node in self.market_map.nodes.values():
            position = self.viewport.to_screen(node.x_m, node.y_m)
            text = f"{node.node_id} {node.name}" if node.name else str(node.node_id)
            label = self.font_small.render(text, True, COLOR_NODE_LABEL)
            # Flip the label to the left of the node when it would run past the
            # map's right edge, so a node on the boundary (e.g. the 10.5 m
            # column) keeps its full label visible.
            label_x = position[0] + 7
            if label_x + label.get_width() > map_right - 2:
                label_x = position[0] - 7 - label.get_width()
            label_y = position[1] - 16
            if label_y < 2:
                label_y = position[1] + 8
            # Outline keeps the text readable over the route line and the trolley.
            outline = self.font_small.render(text, True, COLOR_BACKGROUND)
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                self.screen.blit(outline, (label_x + dx, label_y + dy))
            self.screen.blit(label, (label_x, label_y))

    def _draw_trolley(self, engine: SimulationEngine) -> None:
        viewport = self.viewport
        true_x, true_y, theta = engine.true_pose
        snapshot = engine.core.status()

        # UWB measurement (what the sensor reported).
        if engine.state.measured is not None:
            measured = viewport.to_screen(*engine.state.measured)
            pygame.draw.circle(self.screen, COLOR_TROLLEY_MEASURED, measured, 4)
            pygame.draw.line(self.screen, COLOR_TROLLEY_MEASURED, measured,
                             viewport.to_screen(true_x, true_y), 1)

        # Ground truth body: a rectangle aligned with the true heading.
        center = viewport.to_screen(true_x, true_y)
        length_px = max(10, viewport.meters_to_px(0.45))
        width_px = max(8, viewport.meters_to_px(0.32))
        forward = (math.cos(theta), math.sin(theta))
        left = (-math.sin(theta), math.cos(theta))
        half_len = 0.5 * length_px
        half_wid = 0.5 * width_px
        corners = []
        for along, across in ((half_len, half_wid), (half_len, -half_wid),
                              (-half_len, -half_wid), (-half_len, half_wid)):
            # Screen Y is flipped, so negate the Y component.
            cx = center[0] + along * forward[0] + across * left[0]
            cy = center[1] - (along * forward[1] + across * left[1])
            corners.append((cx, cy))
        pygame.draw.polygon(self.screen, COLOR_TROLLEY, corners)
        pygame.draw.polygon(self.screen, (30, 34, 40), corners, 2)
        # Heading indicator (nose).
        nose = (center[0] + forward[0] * half_len * 1.6, center[1] - forward[1] * half_len * 1.6)
        pygame.draw.line(self.screen, (40, 44, 52), center, nose, 3)

        # Navigation input point (the filtered position the core is using).
        nav_point = viewport.to_screen(snapshot.x_m, snapshot.y_m)
        pygame.draw.circle(self.screen, COLOR_TROLLEY_NAV, nav_point, 4, 1)

    def _draw_map_caption(self, rect: pygame.Rect) -> None:
        text = (
            f"{self.market_map.map_id} v{self.market_map.map_version}  "
            f"{self.market_map.width_m:.1f} x {self.market_map.height_m:.1f} m  "
            f"frame={self.market_map.frame_id}"
        )
        label = self.font_small.render(text, True, COLOR_TEXT_DIM)
        self.screen.blit(label, (rect.x + 6, rect.y + 4))

    # -- side panel --------------------------------------------------------
    def _draw_panel(self, engine: SimulationEngine, fps: float) -> None:
        panel_x = self.window.width_px - self.window.panel_width_px
        panel_rect = pygame.Rect(panel_x, 0, self.window.panel_width_px, self.window.height_px)
        pygame.draw.rect(self.screen, COLOR_PANEL, panel_rect)
        pygame.draw.line(self.screen, COLOR_MAP_BORDER, (panel_x, 0),
                         (panel_x, self.window.height_px), 2)

        snapshot = engine.core.status()
        x = panel_x + 14
        y = 12

        # Reserve a band at the bottom for the event log so a growing list of
        # sections can never overdraw it.
        events_lines = 4
        events_line_height = self.font_small.get_linesize() + 1
        events_top = self.window.height_px - 10 - (events_lines + 1) * events_line_height

        self.panel_lines_drawn = 0
        self.panel_lines_skipped = 0

        max_text_width = self.window.panel_width_px - 2 * (x - panel_x)

        def line(text: str, color=COLOR_TEXT, font=None) -> None:
            nonlocal y
            chosen = font or self.font_small
            if y + chosen.get_linesize() > events_top:
                # No room: skip rather than collide with the event log.  Sections
                # are drawn in priority order, so only the least important lines
                # are ever dropped.
                self.panel_lines_skipped += 1
                return
            # Clip over-long text (a route of long node names) so it cannot spill
            # over the map area.  Truncation keeps the beginning, which is the
            # part that identifies the line.
            if chosen.size(text)[0] > max_text_width:
                ellipsis = "..."
                trimmed = text
                while trimmed and chosen.size(trimmed + ellipsis)[0] > max_text_width:
                    trimmed = trimmed[:-1]
                text = trimmed + ellipsis
            self.screen.blit(chosen.render(text, True, color), (x, y))
            self.panel_lines_drawn += 1
            y += chosen.get_linesize() + 1

        def gap(px: int = 4) -> None:
            nonlocal y
            y += px

        line("NAVIGATION", COLOR_TEXT_DIM, self.font_bold)
        state_color = {
            NavState.ARRIVED: COLOR_OK,
            NavState.ERROR: COLOR_ERROR,
            NavState.EMERGENCY_STOP: COLOR_ERROR,
            NavState.POSITION_LOST: COLOR_WARN,
        }.get(snapshot.state, COLOR_TEXT)
        line(f"State        {snapshot.state_name}", state_color, self.font_bold)
        line(f"Scenario     {engine.state.current_scenario}", font=self.font_small)
        line(f"Route        {self._format_route(snapshot)}", font=self.font_small)
        if snapshot.destination_node >= 0:
            destination_text = (f"{self.market_map.node_label(snapshot.destination_node)} "
                                f"(#{snapshot.destination_node})")
        else:
            destination_text = "none"
        line(f"Dest         {destination_text}  Z/X change", font=self.font_small)
        line(f"Waypoint     {snapshot.active_waypoint_node} "
             f"({snapshot.waypoint_index + 1}/{max(1, snapshot.waypoint_count)})",
             font=self.font_small)
        line(f"Dist WP/dst  {snapshot.distance_to_waypoint_m:5.2f} / "
             f"{snapshot.distance_to_destination_m:5.2f} m", font=self.font_small)
        line(f"Planned/left {snapshot.planned_distance_m:5.2f} / "
             f"{snapshot.remaining_distance_m:5.2f} m", font=self.font_small)
        line(f"Replan/loss  {snapshot.replan_count} / {snapshot.position_loss_events}",
             font=self.font_small)
        line(f"Invalid smp  {snapshot.invalid_sample_count}   "
             f"Dijkstra {snapshot.plan_time_us} us", font=self.font_small)
        gap(4)

        line("POSE", COLOR_TEXT_DIM, self.font_bold)
        line(f"True   ({engine.true_pose[0]:6.2f},{engine.true_pose[1]:6.2f}) "
             f"{math.degrees(engine.true_pose[2]):7.1f}deg", font=self.font_small)
        if engine.state.measured is not None:
            line(f"UWB    ({engine.state.measured[0]:6.2f},{engine.state.measured[1]:6.2f})",
                 font=self.font_small)
        else:
            line("UWB     no sample", COLOR_TEXT_DIM, self.font_small)
        line(f"Nav    ({snapshot.x_m:6.2f},{snapshot.y_m:6.2f}) "
             f"{math.degrees(snapshot.heading_rad):7.1f}deg"
             f"{'' if snapshot.heading_valid else ' est'}", font=self.font_small)
        line(f"Quality      {snapshot.position_quality:5.2f} "
             f"{'valid' if snapshot.position_valid else 'INVALID'}",
             COLOR_TEXT if snapshot.position_valid else COLOR_ERROR, self.font_small)
        gap(4)

        line("CONTROL", COLOR_TEXT_DIM, self.font_bold)
        line(f"v / omega    {snapshot.linear_velocity_mps:6.3f} m/s  "
             f"{snapshot.angular_velocity_radps:6.3f} rad/s", font=self.font_small)
        line(f"Left / right {snapshot.left_motor:6.3f}  {snapshot.right_motor:6.3f}",
             font=self.font_small)
        line(f"Head. error  {math.degrees(snapshot.heading_error_rad):7.1f} deg"
             f"{'  (rotating)' if snapshot.rotate_in_place else ''}", font=self.font_small)
        line(f"Cross track  {snapshot.cross_track_error_m:6.3f} m", font=self.font_small)
        gap(4)

        line("UWB / FAULTS", COLOR_TEXT_DIM, self.font_bold)
        faults = engine.state.active_faults
        line("faults: " + (", ".join(faults) if faults else "none"),
             COLOR_WARN if faults else COLOR_OK, self.font_small)
        gap(4)

        metrics = engine.metrics
        line("METRICS (live)", COLOR_TEXT_DIM, self.font_bold)
        line(f"Mean/max XTE {metrics.mean_cross_track_error_m:5.3f} / "
             f"{metrics.max_cross_track_error_m:5.3f} m", font=self.font_small)
        line(f"RMSE UWB/nav {metrics.uwb_rmse_m:5.3f} / {metrics.nav_input_rmse_m:5.3f} m",
             font=self.font_small)
        line(f"Travelled    {metrics.ground_truth_distance_m:6.2f} m  "
             f"(odo {metrics.wheel_odometry_distance_m:6.2f} m)", font=self.font_small)
        if metrics.completed:
            line(f"Efficiency   {metrics.path_efficiency_percent:6.1f} %  "
                 f"excess {metrics.excess_distance_m:+5.2f} m", font=self.font_small)
        else:
            # Planned-vs-travelled is only meaningful once the route is finished;
            # mid-run it would show a large, meaningless percentage.
            line("Efficiency   n/a (route in progress)", font=self.font_small)
        line(f"Samples      {metrics.samples_valid}/{metrics.samples_received} valid, "
             f"{metrics.samples_invalid} invalid, {metrics.samples_dropped} dropped",
             font=self.font_small)
        line(f"Validation   {metrics.benchmark_overall}",
             COLOR_OK if metrics.benchmark_overall == "PASS" else COLOR_WARN,
             self.font_small)
        gap(4)

        line("CONTROLS", COLOR_TEXT_DIM, self.font_bold)
        for text in (
            "Click map: set destination",
            "Right-click: start   Middle: cancel",
            "1-9 scenario   SPACE run/pause",
            "Z/X dest       ENTER start",
            "R reset        P replan",
            "N/D/J/I/Q/F: UWB faults",
            "E E-stop       C clear",
            "G/T/H: labels/trail/overlay",
            "ESC quit   (full list: docs/simulator.md)",
        ):
            line(text, COLOR_TEXT_DIM, self.font_small)

        if self.config.show_debug_overlay:
            line("DEBUG", COLOR_TEXT_DIM, self.font_bold)
            line(f"FPS {fps:5.1f}  sim {engine.state.sim_time_s:7.2f} s  "
                 f"steps {engine.state.steps}", font=self.font_small)
            gap(4)

        # Recent notes, drawn inside the reserved band.
        event_y = events_top
        self.screen.blit(self.font_small.render("RECENT EVENTS", True, COLOR_TEXT_DIM), (x, event_y))
        event_y += events_line_height
        notes = engine.state.notes[-3:] if engine.state.notes else ["(none yet)"]
        for note in notes:
            if event_y + events_line_height > self.window.height_px - 4:
                break
            self.screen.blit(self.font_small.render(note[:48], True, COLOR_TEXT), (x, event_y))
            event_y += events_line_height

    def _format_route(self, snapshot) -> str:
        """Compact "masuk>...>produk-roti" rendering of the planned route.

        Node names are used where defined because they are what the operator
        typed; the numeric id stays visible on the map label and in the
        Dest select line.
        """
        if not snapshot.route:
            return "n/a"
        labels = [self.market_map.node_label(node) for node in snapshot.route]
        if len(labels) > 2:
            # First hop ... final destination.  Intermediate nodes are visible on
            # the map, and two long names already fill the panel width.
            return f"{labels[0]}>...>{labels[-1]}"
        return ">".join(labels)

    # -- overlays ----------------------------------------------------------
    def draw_overlay_message(self, message: str, color=COLOR_WARN) -> None:
        """Centred banner used for paused / finished / E-stop notifications."""
        text = self.font_title.render(message, True, color)
        rect = text.get_rect(center=(self.viewport.width_px // 2, 28))
        background = rect.inflate(24, 12)
        pygame.draw.rect(self.screen, COLOR_PANEL, background)
        pygame.draw.rect(self.screen, color, background, 2)
        self.screen.blit(text, rect)
