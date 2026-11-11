# SPDX-License-Identifier: Apache-2.0
"""Static travel graph, clearance, and Dijkstra."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

COORD_EPS = 1e-9
PARENT_DIST_EPS = 1e-15
ROBOT_RADIUS_M = 0.25
V_CRUISE = 0.60
WORLD_X = (0.0, 20.0)
WORLD_Y = (0.0, 16.0)
INSET = (ROBOT_RADIUS_M, WORLD_X[1] - ROBOT_RADIUS_M, ROBOT_RADIUS_M, WORLD_Y[1] - ROBOT_RADIUS_M)

OBSTACLES: list[tuple[float, float, float, float]] = [
    (7.0, 9.0, 0.0, 6.5),
    (7.0, 9.0, 9.5, 16.0),
    (12.5, 14.5, 5.0, 11.0),
]

WAYPOINTS: list[tuple[float, float]] = [
    (8.0, 8.0),
    (5.0, 2.0),
    (5.0, 14.0),
    (11.0, 8.0),
    (16.0, 8.0),
    (16.0, 4.0),
    (16.0, 12.0),
]

DEFAULT_ZONES: list[dict[str, Any]] = [
    {"id": "Z0", "rect": [1.0, 3.0, 1.0, 4.0], "centroid": (2.0, 2.5), "area_m2": 6.0, "work_s": 12.0},
    {"id": "Z1", "rect": [1.0, 3.0, 12.0, 15.0], "centroid": (2.0, 13.5), "area_m2": 6.0, "work_s": 12.0},
    {"id": "Z2", "rect": [4.0, 6.0, 6.0, 9.0], "centroid": (5.0, 7.5), "area_m2": 6.0, "work_s": 12.0},
    {"id": "Z3", "rect": [10.0, 12.0, 1.0, 4.0], "centroid": (11.0, 2.5), "area_m2": 6.0, "work_s": 12.0},
    {"id": "Z4", "rect": [10.0, 12.0, 12.0, 15.0], "centroid": (11.0, 13.5), "area_m2": 6.0, "work_s": 12.0},
    {"id": "Z5", "rect": [15.0, 18.0, 1.0, 3.0], "centroid": (16.5, 2.0), "area_m2": 6.0, "work_s": 12.0},
    {"id": "Z6", "rect": [15.0, 18.0, 13.0, 15.0], "centroid": (16.5, 14.0), "area_m2": 6.0, "work_s": 12.0},
    {"id": "Z7", "rect": [16.0, 19.0, 6.0, 9.0], "centroid": (17.5, 7.5), "area_m2": 9.0, "work_s": 18.0},
]

DOCK_POSES = ((4.0, 1.0), (4.0, 15.0))
CHARGER_POSES = ((18.0, 4.0), (18.0, 12.0))

WORK_MIN_S = 10.0
KAPPA = 2.0
DOCK_SERVICE_S = 8.0


@dataclass
class Scenario:
    """Frozen world instance for one family and seed."""

    family: str
    seed: int
    mutant_seed: int | None
    schema_version: str = "mrta-world-v1"
    robot_radius_m: float = ROBOT_RADIUS_M
    v_cruise: float = V_CRUISE
    T_horizon: float = 150.0
    dock_after_zone: bool = False
    dock_service_s: float = DOCK_SERVICE_S
    obstacles: list[tuple[float, float, float, float]] = field(default_factory=lambda: list(OBSTACLES))
    waypoints: list[tuple[float, float]] = field(default_factory=lambda: list(WAYPOINTS))
    zones: list[dict[str, Any]] = field(default_factory=lambda: [dict(z) for z in DEFAULT_ZONES])
    docks: list[dict[str, Any]] = field(default_factory=list)
    chargers: list[dict[str, Any]] = field(default_factory=list)
    robots: list[dict[str, Any]] = field(default_factory=list)
    battery: dict[str, float] = field(default_factory=dict)
    node_coords: list[tuple[float, float]] = field(default_factory=list)
    node_aliases: list[list[str]] = field(default_factory=list)
    dist_m: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))
    parent: np.ndarray = field(default_factory=lambda: np.zeros((0, 0), dtype=np.int32))
    pose_to_node: dict[tuple[float, float], int] = field(default_factory=dict)
    site_to_node: dict[str, int] = field(default_factory=dict)


class InvalidScenario(ValueError):
    """Raised when geometry or poses are illegal."""


def round_coord(x: float) -> float:
    return round(float(x), 9)


def coord_key(x: float, y: float) -> tuple[float, float]:
    return (round_coord(x), round_coord(y))


def point_in_inset(x: float, y: float) -> bool:
    return INSET[0] <= x <= INSET[1] and INSET[2] <= y <= INSET[3]


def point_clear_of_obstacles(x: float, y: float, radius: float) -> bool:
    for ox0, ox1, oy0, oy1 in OBSTACLES:
        cx = max(ox0, min(x, ox1))
        cy = max(oy0, min(y, oy1))
        if (x - cx) ** 2 + (y - cy) ** 2 < radius * radius - 1e-12:
            return False
    return True


def pose_legal(x: float, y: float, radius: float = ROBOT_RADIUS_M) -> bool:
    return point_in_inset(x, y) and point_clear_of_obstacles(x, y, radius)


def _dist_point_aabb(px: float, py: float, ox0: float, ox1: float, oy0: float, oy1: float) -> float:
    cx = max(ox0, min(px, ox1))
    cy = max(oy0, min(py, oy1))
    return math.hypot(px - cx, py - cy)


def _dist_seg_point(
    ax: float, ay: float, bx: float, by: float, px: float, py: float
) -> float:
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-18:
        return math.hypot(apx, apy)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    qx, qy = ax + t * abx, ay + t * aby
    return math.hypot(px - qx, py - qy)


def _point_in_rect(px: float, py: float, ox0: float, ox1: float, oy0: float, oy1: float) -> bool:
    return ox0 <= px <= ox1 and oy0 <= py <= oy1


def _segments_intersect(
    ax: float, ay: float, bx: float, by: float, cx: float, cy: float, dx: float, dy: float
) -> bool:
    def orient(px: float, py: float, qx: float, qy: float, rx: float, ry: float) -> float:
        return (qy - py) * (rx - qx) - (qx - px) * (ry - qy)

    o1 = orient(ax, ay, bx, by, cx, cy)
    o2 = orient(ax, ay, bx, by, dx, dy)
    o3 = orient(cx, cy, dx, dy, ax, ay)
    o4 = orient(cx, cy, dx, dy, bx, by)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return True
    if abs(o1) < 1e-15 and _point_on_segment(cx, cy, ax, ay, bx, by):
        return True
    if abs(o2) < 1e-15 and _point_on_segment(dx, dy, ax, ay, bx, by):
        return True
    if abs(o3) < 1e-15 and _point_on_segment(ax, ay, cx, cy, dx, dy):
        return True
    if abs(o4) < 1e-15 and _point_on_segment(bx, by, cx, cy, dx, dy):
        return True
    return False


def _point_on_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> bool:
    return (
        min(ax, bx) - 1e-12 <= px <= max(ax, bx) + 1e-12
        and min(ay, by) - 1e-12 <= py <= max(ay, by) + 1e-12
    )


def _rect_intersects_segment(
    ax: float, ay: float, bx: float, by: float, ox0: float, ox1: float, oy0: float, oy1: float
) -> bool:
    if _point_in_rect(ax, ay, ox0, ox1, oy0, oy1) or _point_in_rect(bx, by, ox0, ox1, oy0, oy1):
        return True
    edges = (
        (ox0, oy0, ox1, oy0),
        (ox1, oy0, ox1, oy1),
        (ox1, oy1, ox0, oy1),
        (ox0, oy1, ox0, oy0),
    )
    for ex0, ey0, ex1, ey1 in edges:
        if _segments_intersect(ax, ay, bx, by, ex0, ey0, ex1, ey1):
            return True
    return False


def segment_aabb_clearance(
    ax: float, ay: float, bx: float, by: float, ox0: float, ox1: float, oy0: float, oy1: float
) -> float:
    """Minimum distance from closed segment to closed AABB."""
    if _rect_intersects_segment(ax, ay, bx, by, ox0, ox1, oy0, oy1):
        return 0.0
    d = min(
        _dist_point_aabb(ax, ay, ox0, ox1, oy0, oy1),
        _dist_point_aabb(bx, by, ox0, ox1, oy0, oy1),
    )
    for vx, vy in (
        (ox0, oy0),
        (ox1, oy0),
        (ox1, oy1),
        (ox0, oy1),
    ):
        d = min(d, _dist_seg_point(ax, ay, bx, by, vx, vy))
    return d


def segment_clearance(ax: float, ay: float, bx: float, by: float) -> float:
    """Min clearance to all obstacles; segment must stay in world."""
    if not (0.0 <= ax <= WORLD_X[1] and 0.0 <= bx <= WORLD_X[1]):
        return -1.0
    if not (0.0 <= ay <= WORLD_Y[1] and 0.0 <= by <= WORLD_Y[1]):
        return -1.0
    m = float("inf")
    for ox0, ox1, oy0, oy1 in OBSTACLES:
        m = min(m, segment_aabb_clearance(ax, ay, bx, by, ox0, ox1, oy0, oy1))
    return m


def segment_legal(ax: float, ay: float, bx: float, by: float, radius: float = ROBOT_RADIUS_M) -> bool:
    if ax == bx and ay == by:
        return False
    if not pose_legal(ax, ay, radius) or not pose_legal(bx, by, radius):
        return False
    return segment_clearance(ax, ay, bx, by) + 1e-12 >= radius


def euclidean(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def build_graph(scenario: Scenario) -> None:
    """Dedupe nodes and run all-pairs Dijkstra."""
    coords: list[tuple[float, float]] = []
    aliases: list[list[str]] = []
    pose_to_node: dict[tuple[float, float], int] = {}
    site_to_node: dict[str, int] = {}

    def add_site(key: tuple[float, float], alias: str) -> int:
        if key in pose_to_node:
            nid = pose_to_node[key]
            if alias not in aliases[nid]:
                aliases[nid].append(alias)
            site_to_node[alias] = nid
            return nid
        nid = len(coords)
        coords.append(key)
        aliases.append([alias])
        pose_to_node[key] = nid
        site_to_node[alias] = nid
        return nid

    for wp in scenario.waypoints:
        add_site(coord_key(wp[0], wp[1]), f"W{wp}")

    for z in scenario.zones:
        cx, cy = z["centroid"]
        add_site(coord_key(cx, cy), z["id"])

    for i, d in enumerate(scenario.docks):
        px, py = d["pose"]
        add_site(coord_key(px, py), d["id"])

    for i, c in enumerate(scenario.chargers):
        px, py = c["pose"]
        add_site(coord_key(px, py), c["id"])

    for r in scenario.robots:
        sx, sy = r["start"]
        add_site(coord_key(sx, sy), r["id"])

    n = len(coords)
    scenario.node_coords = coords
    scenario.node_aliases = aliases
    scenario.pose_to_node = pose_to_node
    scenario.site_to_node = site_to_node

    dist = np.full((n, n), np.inf)
    parent = np.full((n, n), -1, dtype=np.int32)
    for i in range(n):
        dist[i, i] = 0.0
        parent[i, i] = -1

    adj: list[list[tuple[int, float]]] = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if segment_legal(coords[i][0], coords[i][1], coords[j][0], coords[j][1], scenario.robot_radius_m):
                length = euclidean(coords[i], coords[j])
                adj[i].append((j, length))
                adj[j].append((i, length))

    for src in range(n):
        d = dist[src].copy()
        p = parent[src].copy()
        d[src] = 0.0
        p[src] = -1
        visited = np.zeros(n, dtype=bool)
        for _ in range(n):
            u = -1
            best = np.inf
            for v in range(n):
                if not visited[v] and d[v] < best:
                    best = d[v]
                    u = v
            if u < 0 or not np.isfinite(best):
                break
            visited[u] = True
            for v, w in adj[u]:
                nd = d[u] + w
                if nd < d[v] - PARENT_DIST_EPS:
                    d[v] = nd
                    p[v] = u
                elif abs(nd - d[v]) <= PARENT_DIST_EPS:
                    if p[v] == -1 or u < p[v]:
                        p[v] = u
        dist[src] = d
        parent[src] = p

    scenario.dist_m = dist
    scenario.parent = parent


def travel_time(scenario: Scenario, a: int, b: int) -> float:
    d = float(scenario.dist_m[a, b])
    if not np.isfinite(d):
        return float("inf")
    return d / scenario.v_cruise


def travel_distance(scenario: Scenario, a: int, b: int) -> float:
    d = float(scenario.dist_m[a, b])
    if not np.isfinite(d):
        return float("inf")
    return d


def reconstruct_path(parent_row: np.ndarray, src: int, dst: int) -> list[int]:
    if src == dst:
        return [src]
    if parent_row[dst] < 0:
        return []
    path: list[int] = []
    cur = dst
    seen: set[int] = set()
    while cur != src:
        if cur in seen:
            return []
        seen.add(cur)
        path.append(cur)
        cur = int(parent_row[cur])
        if cur < 0:
            return []
    path.append(src)
    path.reverse()
    return path


def scenario_canonical_dict(scenario: Scenario) -> dict[str, Any]:
    return {
        "battery": {k: round_coord(v) for k, v in sorted(scenario.battery.items())},
        "chargers": scenario.chargers,
        "dock_after_zone": scenario.dock_after_zone,
        "dock_service_s": round_coord(scenario.dock_service_s),
        "docks": scenario.docks,
        "family": scenario.family,
        "obstacles": [list(o) for o in scenario.obstacles],
        "robot_radius_m": round_coord(scenario.robot_radius_m),
        "robots": scenario.robots,
        "schema_version": scenario.schema_version,
        "seed": scenario.seed,
        "T_horizon": round_coord(scenario.T_horizon),
        "v_cruise": round_coord(scenario.v_cruise),
        "waypoints": [list(w) for w in scenario.waypoints],
        "zones": scenario.zones,
    }


def world_sha256(scenario: Scenario) -> str:
    import hashlib

    payload = json.dumps(
        scenario_canonical_dict(scenario),
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def node_for_pose(scenario: Scenario, pose: tuple[float, float]) -> int:
    return scenario.pose_to_node[coord_key(pose[0], pose[1])]
