# SPDX-License-Identifier: Apache-2.0
"""Gate recomputation from RunLogs."""
from __future__ import annotations

import math
from typing import Any

from multi_robot_task_allocation.sim import RunLog

EVAL_SEEDS = {
    "S1": [20261111, 20261112, 20261113],
    "S2": [20261121, 20261122, 20261123],
    "S3": [20261131, 20261132, 20261133],
    "S4": [20261141, 20261142, 20261143, 20261144],
}

HORIZONS = {"S1": 150.0, "S2": 200.0, "S3": 300.0, "S4": 200.0}

MANIFEST: list[tuple[str, str, int, str]] = []
for alloc in ("greedy", "hungarian", "auction"):
    for fam, seeds in EVAL_SEEDS.items():
        for seed in seeds:
            MANIFEST.append((alloc, fam, seed, "pass"))
for fam, seeds in EVAL_SEEDS.items():
    for seed in seeds:
        MANIFEST.append(("random", fam, seed, "fail"))
for seed in EVAL_SEEDS["S2"]:
    MANIFEST.append(("never_charge", "S2", seed, "fail"))


def count_slot_conflicts(log: RunLog) -> int:
    conflicts = 0
    by_site: dict[str, list[tuple[float, float]]] = {}
    for iv in log.occupy:
        by_site.setdefault(iv["site_id"], []).append((iv["start"], iv["end"]))
    for intervals in by_site.values():
        intervals.sort()
        for i in range(len(intervals)):
            for j in range(i + 1, len(intervals)):
                a0, a1 = intervals[i]
                b0, b1 = intervals[j]
                if a0 < b1 and b0 < a1 and not (a1 <= b0 or b1 <= a0):
                    if a1 > b0 and b1 > a0:
                        conflicts += 1
    return conflicts


def metrics_from_log(log: RunLog) -> dict[str, Any]:
    required = set(log.zones_required)
    done = set(log.zones_done)
    coverage_frac = len(done & required) / len(required) if required else 0.0
    coverage_complete = coverage_frac >= 1.0 - 1e-12
    deaths = sum(1 for r in log.robots.values() if r.get("dead"))
    meta = {}
    for ev in log.events:
        if ev.get("kind") == "summary" and "meta" in ev:
            meta = ev["meta"]
    dock_wait = float(meta.get("dock_wait_s", 0.0))
    charge_wait = float(meta.get("charge_wait_s", 0.0))
    travel_m = float(meta.get("total_travel_m", 0.0))
    conflicts = count_slot_conflicts(log)
    makespan = float(log.makespan_s)
    if not math.isfinite(makespan):
        makespan = float("nan")
    hor = HORIZONS.get(log.scenario_family, log.T_horizon)
    mission_pass = (
        coverage_complete
        and deaths == 0
        and conflicts == 0
        and log.complete
        and makespan <= hor + 1e-6
    )
    return {
        "coverage_complete": coverage_complete,
        "deaths": deaths,
        "makespan_s": makespan,
        "complete": log.complete,
        "total_travel_m": travel_m,
        "dock_wait_s": dock_wait,
        "charge_wait_s": charge_wait,
        "slot_conflicts": conflicts,
        "pass": mission_pass,
    }


def evaluate_row(
    allocator: str,
    family: str,
    seed: int,
    expected: str,
    log: RunLog,
) -> dict[str, Any]:
    m = metrics_from_log(log)
    if allocator == "never_charge" and expected == "fail":
        mutant_ok = m["deaths"] >= 1 and not m["coverage_complete"]
        row_pass = mutant_ok
        observed = "fail" if mutant_ok else "pass"
    elif allocator == "random" and expected == "fail":
        mutant_ok = not m["pass"]
        row_pass = mutant_ok
        observed = "fail" if mutant_ok else "pass"
    else:
        row_pass = m["pass"]
        observed = "pass" if m["pass"] else "fail"
    return {
        "allocator": allocator,
        "family": family,
        "seed": seed,
        "world_sha": log.world_sha,
        "coverage_complete": m["coverage_complete"],
        "deaths": m["deaths"],
        "makespan_s": m["makespan_s"],
        "complete": m["complete"],
        "total_travel_m": m["total_travel_m"],
        "dock_wait_s": m["dock_wait_s"],
        "charge_wait_s": m["charge_wait_s"],
        "slot_conflicts": m["slot_conflicts"],
        "expected": expected,
        "observed": observed,
        "pass": row_pass,
    }
