"""Supermarket floor-plan layer for the simulator renderer.

Provider of a `draw_floorplan(surface, viewport)` function that draws a clean,
non-"AI slop" top-down store layout under the navigation graph: outer walls,
three shelf racks per aisle row, a checkout counter, the entrance and the
parking zone.

The layout is derived from the 12 x 8 m Smart Market map (config/map.json) and
the 12-node graph (config/graph.json).  All dimensions are in the
``smart_market_map`` frame (meters, origin bottom-left, +X right, +Y up), so the
renderer's existing ``Viewport`` maps them to screen pixels.

Nothing here changes navigation: it is a pure presentation helper.
"""

from __future__ import annotations

import math

import pygame

# --- palette -----------------------------------------------------------------
# Deliberately muted so the route/trolley/telem overlay stays the hero.  These
# are graph-neutral "building" colours, not the node/trajectory palette in
# renderer.py.
COLOR_FLOOR = (34, 38, 46)          # market floor fill
COLOR_WALL = (96, 104, 120)         # perimeter wall
COLOR_SHELF = (72, 80, 96)          # rack body
COLOR_SHELF_EDGE = (108, 118, 136)  # rack outline
COLOR_CHECKOUT = (96, 92, 110)      # checkout counter body
COLOR_CHECKOUT_EDGE = (132, 128, 148)
COLOR_ENTRANCE = (58, 88, 84)       # entrance door fill
COLOR_PARKING = (52, 62, 78)        # parking bay fill
COLOR_PARKING_EDGE = (90, 102, 120)

#: Shelf rack half-extents (meters).  Racks run vertically along an aisle.
#: The aisle corridors sit at y = 2, 4, 6 m (2 m pitch) and x = 1.5, 4.5, 7.5,
#: 10.5 m, so a rack must stay ~0.6 m tall to leave room to walk (and for the
#: trolley to drive) around it.  0.65 m each side keeps every rack clear of the
#: corridors and of the map walls.
SHELF_HALF_LEN = 0.65   # along Y (rack half-height)
SHELF_HALF_WID = 0.28   # along X (rack half-width)


def _rect_corners(cx: float, cy: float, sx: float, sy: float) -> list[tuple[float, float]]:
    """Four corners (CCW) of a centered, axis-aligned rectangle in meters."""
    return [(cx - sx, cy - sy), (cx + sx, cy - sy),
            (cx + sx, cy + sy), (cx - sx, cy + sy)]


def _to_screen(viewport, x_m: float, y_m: float) -> tuple[int, int]:
    return viewport.to_screen(x_m, y_m)


def _draw_rect(viewport, surface, color, cx: float, cy: float,
               half_x: float, half_y: float, width: int = 0) -> pygame.Rect:
    pts = [_to_screen(viewport, *c) for c in _rect_corners(cx, cy, half_x, half_y)]
    rect = pygame.Rect(min(p[0] for p in pts), min(p[1] for p in pts),
                       max(p[0] for p in pts) - min(p[0] for p in pts),
                       max(p[1] for p in pts) - min(p[1] for p in pts))
    pygame.draw.rect(surface, color, rect, width)
    return rect


def _draw_line(viewport, surface, color, a: tuple[float, float],
               b: tuple[float, float], width: int = 1) -> None:
    pa = _to_screen(viewport, *a)
    pb = _to_screen(viewport, *b)
    pygame.draw.line(surface, color, pa, pb, width)


def _draw_shelf_rack(viewport, surface, center_x: float, center_y: float) -> None:
    """One vertical product rack with a subtle inner divider stripe.

    A grocery rack is not a solid slab: the divider hint makes it read as a
    stocked bay instead of a wall, while keeping the geometry dead simple.
    """
    _draw_rect(viewport, surface, COLOR_SHELF,
               center_x, center_y, SHELF_HALF_WID, SHELF_HALF_LEN)
    _draw_rect(viewport, surface, COLOR_SHELF_EDGE,
               center_x, center_y, SHELF_HALF_WID, SHELF_HALF_LEN, 2)
    # Centre divider (aisle marker on the rack).
    _draw_line(viewport, surface, (86, 94, 108),
               (center_x, center_y - SHELF_HALF_LEN + 0.2),
               (center_x, center_y + SHELF_HALF_LEN - 0.2), 2)


def draw_floorplan(surface: pygame.Surface, viewport) -> None:
    """Draw the supermarket floor plan under the navigation overlays."""
    min_x, min_y, max_x, max_y = viewport.map_min_x, viewport.map_min_y, \
        viewport.map_max_x, viewport.map_max_y

    # 1. Floor and perimeter wall (the store shell).
    corners = [_to_screen(viewport, x, y) for x, y in
               ((min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y))]
    pygame.draw.polygon(surface, COLOR_FLOOR, corners)
    # Outer wall: a double outline sells the "building" read cheaply.
    rect = pygame.Rect(min(p[0] for p in corners), min(p[1] for p in corners),
                       max(p[0] for p in corners) - min(p[0] for p in corners),
                       max(p[1] for p in corners) - min(p[1] for p in corners))
    pygame.draw.rect(surface, COLOR_WALL, rect, 4)
    pygame.draw.rect(surface, (58, 64, 74), rect, 1)

    # 2. Entrance (bottom-left, gap in the bottom wall) + door fill.
    #    Entrance sits beside node "masuk" (1.5, 2.0); the store entry is below
    #    it along the bottom wall.
    entrance_cx, entrance_cy = 1.5, min_y + 0.35
    _draw_rect(viewport, surface, COLOR_ENTRANCE, entrance_cx, entrance_cy, 0.7, 0.35)
    _draw_rect(viewport, surface, COLOR_WALL, entrance_cx, entrance_cy, 0.7, 0.35, 2)

    # 3. Three rows of 3 shelf racks, matching the aisle grid:
    #    aisle row A (y=2.0) between the lowest racks; row B (y=4.0) middle;
    #    row C (y=6.0) top.  Racks sit *between* aisle corridors so the graph
    #    lines run through the walkable gaps.
    rack_centers_x = (3.0, 6.0, 9.0)
    # Bottom rack row (products around y=1.0, aisle corridor at y=2.0).
    for cx in rack_centers_x:
        _draw_shelf_rack(viewport, surface, cx, 1.0)
    # Middle rack row (around y=3.0, between aisle y=2 and y=4).
    for cx in rack_centers_x:
        _draw_shelf_rack(viewport, surface, cx, 3.0)
    # Top rack row (around y=5.0, between aisle y=4 and y=6).
    for cx in rack_centers_x:
        _draw_shelf_rack(viewport, surface, cx, 5.0)

    # 4. Checkout counters along the top-right (kasir node 4 at 10.5, 2.0).
    #    A single wide register band next to the entrance aisle.
    checkout_cx, checkout_cy = 10.5, 2.0
    _draw_rect(viewport, surface, COLOR_CHECKOUT, checkout_cx, checkout_cy, 1.1, 0.45)
    _draw_rect(viewport, surface, COLOR_CHECKOUT_EDGE, checkout_cx, checkout_cy, 1.1, 0.45, 2)
    # Register stripe hints.
    for offset in (-0.5, 0.0, 0.5):
        _draw_line(viewport, surface, (120, 116, 136),
                   (checkout_cx + offset, checkout_cy - 0.45),
                   (checkout_cx + offset, checkout_cy + 0.45), 2)

    # 5. Parking bay along the right edge (parkir node 12 at 10.5, 6.0).
    park_cx, park_cy = 10.5, 6.0
    _draw_rect(viewport, surface, COLOR_PARKING, park_cx, park_cy, 1.2, 0.9)
    _draw_rect(viewport, surface, COLOR_PARKING_EDGE, park_cx, park_cy, 1.2, 0.9, 2)
    # Parking slot separators.
    for yy in (park_cy - 0.3, park_cy + 0.3):
        _draw_line(viewport, surface, COLOR_PARKING_EDGE,
                   (park_cx - 1.2, yy), (park_cx + 1.2, yy), 1)