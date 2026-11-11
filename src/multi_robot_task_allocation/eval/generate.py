# SPDX-License-Identifier: Apache-2.0
"""Scenario generator."""
from __future__ import annotations

import copy
from typing import Any

import numpy as np

from multi_robot_task_allocation.world import (
    CHARGER_POSES,
    DEFAULT_ZONES,
    DOCK_POSES,
    DOCK_SERVICE_S,
    KAPPA,
    WORK_MIN_S,
    InvalidScenario,
    Scenario,
    build_graph,
    pose_legal,
    world_sha256,
)

FAMILY_HORIZONS = {"s1": 150.0, "s2": 200.0, "s3": 300.0, "s4": 200.0}
FAMILY_HORIZONS_UPPER = {"S1": 150.0, "S2": 200.0, "S3": 300.0, "S4": 200.0}

START_POOL = [
    (2.0, 8.0),
    (11.0, 8.0),
    (16.0, 8.0),
    (2.0, 2.5),
    (2.0, 13.5),
    (16.5, 2.0),
]

DEFAULT_BATTERY = {
    "drain_move": 0.004,
    "drain_work": 0.003,
    "drain_idle": 0.0008,
    "charge_rate": 0.025,
    "min_reserve": 0.15,
}


def _family_key(family: str) -> str:
    return family.upper()


def generate(family: str, seed: int) -> Scenario:
    fam = _family_key(family)
    if fam not in FAMILY_HORIZONS_UPPER:
        raise ValueError(f"unknown family {family}")

    rng = np.random.Generator(np.random.PCG64(seed))
    zones = copy.deepcopy(DEFAULT_ZONES)
    for z in zones:
        area = z["area_m2"]
        z["work_s"] = max(WORK_MIN_S, KAPPA * area)

    dock_after_zone = fam == "S3"
    T_horizon = FAMILY_HORIZONS_UPPER[fam]

    drain_move = 0.008 if fam == "S2" else 0.004
    start_soc = 0.35 if fam == "S2" else 1.0

    battery = dict(DEFAULT_BATTERY)
    battery["drain_move"] = drain_move

    if fam == "S4":
        zone_ids = [z["id"] for z in zones]
        rng.shuffle(zone_ids)
        long_set = set(zone_ids[:3])
        for z in zones:
            if z["id"] in long_set:
                z["work_s"] = 40.0

    pool = list(START_POOL)
    rng.shuffle(pool)
    starts: list[tuple[float, float]] = []
    for p in pool:
        if p not in starts:
            starts.append(p)
        if len(starts) >= 4:
            break

    if len(starts) < 4:
        raise InvalidScenario("not enough distinct starts")

    for sx, sy in starts:
        if not pose_legal(sx, sy):
            raise InvalidScenario(f"illegal start {sx},{sy}")

    docks: list[dict[str, Any]] = []
    if fam == "S3":
        docks.append({"id": "D0", "pose": DOCK_POSES[0], "capacity": 1})
    else:
        docks.append({"id": "D0", "pose": DOCK_POSES[0], "capacity": 1})
        docks.append({"id": "D1", "pose": DOCK_POSES[1], "capacity": 1})

    chargers = [
        {"id": "C0", "pose": CHARGER_POSES[0], "capacity": 1},
        {"id": "C1", "pose": CHARGER_POSES[1], "capacity": 1},
    ]

    robots = [
        {"id": f"R{i}", "start": starts[i], "start_soc": start_soc}
        for i in range(4)
    ]

    scenario = Scenario(
        family=fam,
        seed=seed,
        mutant_seed=None,
        T_horizon=T_horizon,
        dock_after_zone=dock_after_zone,
        dock_service_s=DOCK_SERVICE_S,
        zones=zones,
        docks=docks,
        chargers=chargers,
        robots=robots,
        battery=battery,
    )
    build_graph(scenario)
    _validate_reachability(scenario)
    return scenario


def _validate_reachability(scenario: Scenario) -> None:
    import math

    from multi_robot_task_allocation.world import node_for_pose, travel_time

    for r in scenario.robots:
        rn = node_for_pose(scenario, tuple(r["start"]))
        for z in scenario.zones:
            cn = scenario.site_to_node[z["id"]]
            if not math.isfinite(travel_time(scenario, rn, cn)):
                raise InvalidScenario(f"robot {r['id']} cannot reach {z['id']}")
        for c in scenario.chargers:
            cn = scenario.site_to_node[c["id"]]
            if not math.isfinite(travel_time(scenario, rn, cn)):
                raise InvalidScenario(f"robot {r['id']} cannot reach {c['id']}")
        for d in scenario.docks:
            dn = scenario.site_to_node[d["id"]]
            if not math.isfinite(travel_time(scenario, rn, dn)):
                raise InvalidScenario(f"robot {r['id']} cannot reach {d['id']}")


def scenario_to_json(scenario: Scenario) -> dict[str, Any]:
    from multi_robot_task_allocation.world import scenario_canonical_dict

    return scenario_canonical_dict(scenario)


def write_world_sha(scenario: Scenario) -> str:
    return world_sha256(scenario)
