# SPDX-License-Identifier: Apache-2.0
"""Myopic feasibility wrapper for coverage assignment."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from multi_robot_task_allocation.battery import (
    MIN_RESERVE,
    apply_drain,
    charge_duration_to_full,
    soc_after_idle,
)
from multi_robot_task_allocation.slots import SlotCalendar
from multi_robot_task_allocation.world import Scenario, node_for_pose, travel_distance, travel_time


@dataclass
class PlanStep:
    kind: str
    site_id: str | None = None
    travel_s: float = 0.0
    travel_m: float = 0.0
    work_s: float = 0.0
    service_s: float = 0.0
    charger_id: str | None = None
    dock_id: str | None = None


@dataclass
class Candidate:
    robot_id: str
    zone_id: str
    cost: float
    finish_time: float
    travel_m: float
    steps: list[PlanStep] = field(default_factory=list)
    charge_inserted: bool = False


@dataclass
class RobotState:
    robot_id: str
    pose: tuple[float, float]
    soc: float
    ready_time: float
    dead: bool = False


NEXT_FIT_ITER_CAP = 200


def feasible_plan(
    scenario: Scenario,
    robot: RobotState,
    zone_index: int,
    calendar: SlotCalendar,
    now: float,
    *,
    allow_charge: bool = True,
    require_witness: bool = True,
    allow_mid_plan_death: bool = False,
) -> Candidate | None:
    """Build a residual plan for (robot, zone) at time now."""
    zone = scenario.zones[zone_index]
    zone_id = zone["id"]
    if robot.dead:
        return None

    t0 = max(now, robot.ready_time)
    soc0 = robot.soc

    def try_residual(soc_start: float, ready: float, prefix: list[PlanStep]) -> Candidate | None:
        steps: list[PlanStep] = list(prefix)
        soc = soc_start
        t = ready
        travel_m = sum(s.travel_m for s in steps)

        rn = node_for_pose(scenario, robot.pose)
        zn = scenario.site_to_node[zone_id]
        leg_s = travel_time(scenario, rn, zn)
        leg_m = travel_distance(scenario, rn, zn)
        if not math.isfinite(leg_s):
            return None

        soc, death_t, dead = apply_drain(soc, scenario.battery["drain_move"], leg_s)
        if dead:
            if allow_mid_plan_death and death_t is not None:
                finish = ready + death_t
                steps.append(PlanStep("travel_zone", zone_id, death_t, leg_m * (death_t / leg_s)))
                return Candidate(
                    robot_id=robot.robot_id,
                    zone_id=zone_id,
                    cost=finish,
                    finish_time=finish,
                    travel_m=travel_m + leg_m * (death_t / leg_s),
                    steps=steps,
                )
            return None
        t += leg_s
        travel_m += leg_m
        steps.append(PlanStep("travel_zone", zone_id, leg_s, leg_m))

        work_s = float(zone["work_s"])
        soc, death_t, dead = apply_drain(soc, scenario.battery["drain_work"], work_s)
        if dead:
            if allow_mid_plan_death and death_t is not None:
                finish = t + death_t
                steps.append(PlanStep("work", zone_id, work_s=death_t))
                return Candidate(
                    robot_id=robot.robot_id,
                    zone_id=zone_id,
                    cost=finish,
                    finish_time=finish,
                    travel_m=travel_m,
                    steps=steps,
                )
            return None
        t += work_s
        steps.append(PlanStep("work", zone_id, work_s=work_s))

        finish = t
        if scenario.dock_after_zone:
            best_dock: tuple[str, float, float] | None = None
            for d in scenario.docks:
                did = d["id"]
                dn = scenario.site_to_node[did]
                ds = travel_time(scenario, zn, dn)
                dm = travel_distance(scenario, zn, dn)
                if not math.isfinite(ds):
                    continue
                soc_d, _, dead_d = apply_drain(soc, scenario.battery["drain_move"], ds)
                if dead_d:
                    continue
                soc_d, _, dead_d = apply_drain(soc_d, scenario.battery["drain_idle"], 0.0)
                arr = t + ds
                svc = scenario.dock_service_s
                slot_t = calendar.next_fit(did, arr, svc)
                if slot_t is None:
                    continue
                wait = slot_t - arr
                soc_w = soc_after_idle(soc_d, scenario.battery["drain_idle"], wait)
                if soc_w <= 0.0:
                    continue
                finish_d = slot_t + svc
                cand = (did, finish_d, dm + ds)
                if best_dock is None or finish_d < best_dock[1]:
                    best_dock = cand
            if best_dock is None:
                return None
            did, finish, _ = best_dock
            dn = scenario.site_to_node[did]
            ds = travel_time(scenario, zn, dn)
            dm = travel_distance(scenario, zn, dn)
            arr = t + ds
            svc = scenario.dock_service_s
            slot_t = calendar.next_fit(did, arr, svc)
            if slot_t is None:
                return None
            wait = slot_t - arr
            soc_mid = soc
            soc_mid, _, dead = apply_drain(soc_mid, scenario.battery["drain_move"], ds)
            if dead:
                return None
            soc_mid = soc_after_idle(soc_mid, scenario.battery["drain_idle"], wait)
            if soc_mid <= 0.0:
                return None
            soc = soc_mid
            soc, _, dead = apply_drain(soc, scenario.battery["drain_idle"], svc)
            if dead:
                return None
            t = slot_t + svc
            travel_m += dm
            steps.append(PlanStep("travel_dock", did, ds, dm, dock_id=did))
            steps.append(PlanStep("dock", did, service_s=svc, dock_id=did))
            finish = t

        if require_witness:
            witness_ok = False
            after_pose = zone["centroid"] if not scenario.dock_after_zone else None
            if scenario.dock_after_zone and steps:
                for d in scenario.docks:
                    if steps[-1].dock_id == d["id"]:
                        after_pose = d["pose"]
                        break
            if after_pose is None:
                after_pose = zone["centroid"]
            from_n = node_for_pose(scenario, after_pose)

            for c in scenario.chargers:
                cid = c["id"]
                cn = scenario.site_to_node[cid]
                ts = travel_time(scenario, from_n, cn)
                if not math.isfinite(ts):
                    continue
                soc_arr, _, dead = apply_drain(soc, scenario.battery["drain_move"], ts)
                if dead:
                    continue
                if soc_arr >= MIN_RESERVE:
                    witness_ok = True
                    break
            if not witness_ok:
                return None

        return Candidate(
            robot_id=robot.robot_id,
            zone_id=zone_id,
            cost=finish,
            finish_time=finish,
            travel_m=travel_m,
            steps=steps,
            charge_inserted=bool(prefix),
        )

    direct = try_residual(soc0, t0, [])
    if direct is not None:
        return direct
    if not allow_charge:
        return None

    best: Candidate | None = None
    rn = node_for_pose(scenario, robot.pose)

    for c in scenario.chargers:
        cid = c["id"]
        cn = scenario.site_to_node[cid]
        to_chg_s = travel_time(scenario, rn, cn)
        to_chg_m = travel_distance(scenario, rn, cn)
        if not math.isfinite(to_chg_s):
            continue

        arr0 = t0 + to_chg_s
        soc_arr, _, dead = apply_drain(soc0, scenario.battery["drain_move"], to_chg_s)
        if dead:
            continue

        t_slot = arr0
        for _ in range(NEXT_FIT_ITER_CAP):
            wait = max(0.0, t_slot - arr0)
            soc_at_service = soc_after_idle(soc_arr, scenario.battery["drain_idle"], wait)
            if soc_at_service <= 0.0:
                break
            fill_s = charge_duration_to_full(soc_at_service, scenario.battery["charge_rate"])
            gap = calendar.next_fit(cid, t_slot, fill_s)
            if gap is None:
                break
            if abs(gap - t_slot) <= 1e-9:
                ready_after = gap + fill_s
                soc_full = 1.0
                prefix = [
                    PlanStep("travel_charge", cid, to_chg_s, to_chg_m, charger_id=cid),
                    PlanStep(
                        "charge",
                        cid,
                        service_s=fill_s,
                        charger_id=cid,
                    ),
                ]
                cand = try_residual(soc_full, ready_after, prefix)
                if cand is not None:
                    cand.charge_inserted = True
                    if best is None or cand.cost < best.cost:
                        best = cand
                break
            t_slot = gap
        else:
            continue

    return best


def build_cost_matrix(
    scenario: Scenario,
    robots: list[RobotState],
    pending_zone_indices: list[int],
    calendar: SlotCalendar,
    now: float,
    *,
    allow_charge: bool = True,
    require_witness: bool = True,
    allow_mid_plan_death: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    import numpy as np

    nrow = len(robots)
    ncol = len(pending_zone_indices)
    cost = np.full((nrow, ncol), np.inf)
    travel = np.zeros((nrow, ncol))
    for i, rob in enumerate(robots):
        for j, zi in enumerate(pending_zone_indices):
            cand = feasible_plan(
                scenario,
                rob,
                zi,
                calendar,
                now,
                allow_charge=allow_charge,
                require_witness=require_witness,
                allow_mid_plan_death=allow_mid_plan_death,
            )
            if cand is not None and math.isfinite(cand.cost):
                cost[i, j] = cand.cost
                travel[i, j] = cand.travel_m
    return cost, travel


def compare_pairs(
    cost_a: float,
    trav_a: float,
    rid_a: int,
    tid_a: int,
    cost_b: float,
    trav_b: float,
    rid_b: int,
    tid_b: int,
) -> int:
    if cost_a < cost_b - 1e-9:
        return -1
    if cost_b < cost_a - 1e-9:
        return 1
    if trav_a < trav_b - 1e-9:
        return -1
    if trav_b < trav_a - 1e-9:
        return 1
    if rid_a < rid_b:
        return -1
    if rid_b < rid_a:
        return 1
    if tid_a < tid_b:
        return -1
    if tid_b < tid_a:
        return 1
    return 0
