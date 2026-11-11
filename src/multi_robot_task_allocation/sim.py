# SPDX-License-Identifier: Apache-2.0
"""Discrete-event simulator and RunLog."""
from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from multi_robot_task_allocation.alloc.auction import auction_assign
from multi_robot_task_allocation.alloc.greedy import greedy_assign
from multi_robot_task_allocation.alloc.hungarian import hungarian_assign
from multi_robot_task_allocation.alloc.never_charge import never_charge_assign
from multi_robot_task_allocation.alloc.random_alloc import random_pick
from multi_robot_task_allocation.alloc.wrapper import (
    Candidate,
    RobotState,
    build_cost_matrix,
    feasible_plan,
)
from multi_robot_task_allocation.battery import (
    MIN_RESERVE,
    apply_charge,
    apply_drain,
    charge_duration_to_full,
)
from multi_robot_task_allocation.slots import SlotCalendar
from multi_robot_task_allocation.world import Scenario, node_for_pose, travel_distance, travel_time

EVENT_ORDER = [
    "battery_empty",
    "arrive",
    "start_work",
    "finish_work",
    "start_charge",
    "finish_charge",
    "start_dock",
    "finish_dock",
    "soc_watch",
    "horizon",
]

_INSERT_COUNTER = 0


def _next_insert_id() -> int:
    global _INSERT_COUNTER
    _INSERT_COUNTER += 1
    return _INSERT_COUNTER


@dataclass
class SimEvent:
    time: float
    kind: str
    robot_id: str = ""
    payload: dict[str, Any] = field(default_factory=dict)
    insert_id: int = 0

    def __lt__(self, other: object) -> bool:
        if not isinstance(other, SimEvent):
            return NotImplemented
        if self.time != other.time:
            return self.time < other.time
        ki = EVENT_ORDER.index(self.kind) if self.kind in EVENT_ORDER else 99
        kj = EVENT_ORDER.index(other.kind) if other.kind in EVENT_ORDER else 99
        if ki != kj:
            return ki < kj
        if self.robot_id != other.robot_id:
            return self.robot_id < other.robot_id
        return self.insert_id < other.insert_id


@dataclass
class RunLog:
    schema_version: str = "mrta-runlog-v1"
    allocator: str = ""
    scenario_family: str = ""
    seed: int = 0
    mutant_seed: int | None = None
    world_sha: str = ""
    events: list[dict[str, Any]] = field(default_factory=list)
    occupy: list[dict[str, Any]] = field(default_factory=list)
    actions: list[dict[str, Any]] = field(default_factory=list)
    robots: dict[str, dict[str, Any]] = field(default_factory=dict)
    zones_done: set[str] = field(default_factory=set)
    zones_required: list[str] = field(default_factory=list)
    complete: bool = False
    reason: str = ""
    T_horizon: float = 0.0
    makespan_s: float = 0.0


class Simulator:
    def __init__(self, scenario: Scenario, allocator: str, mutant_seed: int | None) -> None:
        self.scenario = scenario
        self.allocator = allocator
        self.mutant_seed = mutant_seed
        self.rng = (
            np.random.Generator(np.random.PCG64(mutant_seed))
            if mutant_seed is not None
            else None
        )
        self.now = 0.0
        self.calendar = SlotCalendar()
        self.queue: list[SimEvent] = []
        self.log = RunLog(
            allocator=allocator,
            scenario_family=scenario.family,
            seed=scenario.seed,
            mutant_seed=mutant_seed,
            T_horizon=scenario.T_horizon,
            zones_required=[z["id"] for z in scenario.zones],
        )
        self.robot_pose: dict[str, tuple[float, float]] = {}
        self.robot_soc: dict[str, float] = {}
        self.robot_dead: dict[str, bool] = {}
        self.robot_busy_until: dict[str, float] = {}
        self.robot_state: dict[str, str] = {}
        self.pending_zones: set[str] = set(z["id"] for z in scenario.zones)
        self.assigned_zones: set[str] = set()
        self.scheduled_zones: set[str] = set()
        self.dock_wait_s: float = 0.0
        self.charge_wait_s: float = 0.0
        self.total_travel_m: float = 0.0
        self.deaths: int = 0
        self._event_counter = 0
        self.robot_active_zone: dict[str, str] = {}

    def _coverage_incomplete(self) -> bool:
        required = set(self.log.zones_required)
        return len(self.log.zones_done) < len(required)

    def _push(self, ev: SimEvent) -> None:
        ev.insert_id = _next_insert_id()
        heapq.heappush(self.queue, ev)

    def _record(self, ev: SimEvent) -> None:
        self.log.events.append(
            {
                "t": ev.time,
                "kind": ev.kind,
                "robot_id": ev.robot_id,
                **ev.payload,
            }
        )

    def _robot_states(self) -> list[RobotState]:
        free: list[RobotState] = []
        for r in self.scenario.robots:
            rid = r["id"]
            if self.robot_dead.get(rid, False):
                continue
            if self.robot_state.get(rid, "free") != "free":
                continue
            if self.now < self.robot_busy_until.get(rid, 0.0) - 1e-12:
                continue
            free.append(
                RobotState(
                    robot_id=rid,
                    pose=self.robot_pose[rid],
                    soc=self.robot_soc[rid],
                    ready_time=max(self.now, self.robot_busy_until.get(rid, 0.0)),
                )
            )
        return free

    def _allow_charge(self) -> bool:
        return self.allocator not in ("never_charge",)

    def _run_epoch(self) -> None:
        if self.allocator == "random":
            if any(
                not self.robot_dead.get(r["id"], False)
                and self.robot_state.get(r["id"], "free") == "busy"
                for r in self.scenario.robots
            ):
                return
        free = self._robot_states()
        if not free:
            return
        pending = [i for i, z in enumerate(self.scenario.zones) if z["id"] in self.pending_zones]
        if not pending:
            if self.allocator == "never_charge":
                self._never_charge_idle_drain(free)
            elif self._allow_charge():
                self._maybe_idle_charge(free)
            return

        free_robots = free
        allow = self._allow_charge()
        mid_death = self.allocator in ("never_charge", "random")
        witness = self.allocator != "never_charge"
        matrix_allow_charge = False if self.allocator == "random" else allow
        cost, travel = build_cost_matrix(
            self.scenario,
            free_robots,
            pending,
            self.calendar,
            self.now,
            allow_charge=matrix_allow_charge,
            require_witness=witness,
            allow_mid_plan_death=mid_death,
        )

        if self.allocator == "random":
            if self.rng is None:
                return
            pick = random_pick(cost, self.rng)
            if pick is None:
                return
            if math.isfinite(float(cost[pick[0], pick[1]])):
                self._commit_pairs_sequential(
                    free_robots,
                    pending,
                    [pick],
                    cost,
                    travel,
                    allow_charge=False,
                )
                return
            self._commit_random_raw(free_robots, pending, pick)
            return

        if self.allocator == "greedy" or self.allocator == "never_charge":
            while True:
                free = self._robot_states()
                pending = [i for i, z in enumerate(self.scenario.zones) if z["id"] in self.pending_zones]
                if not free or not pending:
                    break
                cost, travel = build_cost_matrix(
                    self.scenario,
                    free,
                    pending,
                    self.calendar,
                    self.now,
                    allow_charge=allow,
                    require_witness=witness,
                    allow_mid_plan_death=mid_death,
                )
                assign_fn = never_charge_assign if self.allocator == "never_charge" else greedy_assign
                picks = assign_fn(cost, travel)
                if not picks:
                    stuck = self._robot_states()
                    if self.allocator == "never_charge" and self._coverage_incomplete():
                        for rob in stuck:
                            if not self.robot_dead.get(rob.robot_id, False):
                                self._kill_robot(
                                    rob.robot_id, self.now, 0.0, record_now=True
                                )
                    else:
                        self._never_charge_idle_drain(stuck)
                    break
                self._commit_pairs_sequential(free, pending, picks[:1], cost, travel)
            if self.allocator != "never_charge":
                self._maybe_idle_charge(self._robot_states())
            else:
                self._never_charge_idle_drain(self._robot_states())
            return

        if self.allocator in ("hungarian", "auction"):
            from multi_robot_task_allocation.alloc.auction import AuctionRoundCap

            epoch_assign = (
                hungarian_assign
                if self.allocator == "hungarian"
                else auction_assign
            )
            while True:
                free = self._robot_states()
                pending = [
                    i
                    for i, z in enumerate(self.scenario.zones)
                    if z["id"] in self.pending_zones
                ]
                if not free or not pending:
                    break
                cost, travel = build_cost_matrix(
                    self.scenario,
                    free,
                    pending,
                    self.calendar,
                    self.now,
                    allow_charge=allow,
                    require_witness=witness,
                    allow_mid_plan_death=mid_death,
                )
                try:
                    picks = epoch_assign(cost, travel)
                except AuctionRoundCap:
                    raise
                if not picks:
                    break
                ordered = sorted(
                    picks,
                    key=lambda p: (
                        float(cost[p[0], p[1]]),
                        float(travel[p[0], p[1]]),
                        p[0],
                        p[1],
                    ),
                )
                before = len(self.assigned_zones)
                for pair in ordered:
                    self._commit_pairs_sequential(
                        free, pending, [pair], cost, travel
                    )
                    if len(self.assigned_zones) > before:
                        break
                if len(self.assigned_zones) <= before:
                    break
            while True:
                free = self._robot_states()
                pending = [
                    i
                    for i, z in enumerate(self.scenario.zones)
                    if z["id"] in self.pending_zones
                ]
                if not free or not pending:
                    break
                cost, travel = build_cost_matrix(
                    self.scenario,
                    free,
                    pending,
                    self.calendar,
                    self.now,
                    allow_charge=allow,
                    require_witness=witness,
                    allow_mid_plan_death=mid_death,
                )
                picks = greedy_assign(cost, travel)
                if not picks:
                    break
                self._commit_pairs_sequential(free, pending, picks[:1], cost, travel)

        self._maybe_idle_charge(self._robot_states())

    def _commit_pairs_sequential(
        self,
        free_robots: list[RobotState],
        pending: list[int],
        pairs: list[tuple[int, int]],
        cost: np.ndarray,
        travel: np.ndarray,
        allow_charge: bool | None = None,
    ) -> None:
        if allow_charge is None:
            allow = self._allow_charge()
        else:
            allow = allow_charge
        for r_idx, t_idx in pairs:
            if r_idx >= len(free_robots) or t_idx >= len(pending):
                continue
            rob = free_robots[r_idx]
            zi = pending[t_idx]
            cand = feasible_plan(
                self.scenario,
                rob,
                zi,
                self.calendar,
                self.now,
                allow_charge=allow,
                require_witness=self.allocator != "never_charge",
                allow_mid_plan_death=self.allocator in ("never_charge", "random"),
            )
            if cand is None:
                continue
            self._execute_candidate(cand)
            free_robots = [x for x in free_robots if x.robot_id != rob.robot_id]
            pending = [i for i in pending if self.scenario.zones[i]["id"] not in self.assigned_zones]

    def _execute_candidate(self, cand: Candidate) -> None:
        rid = cand.robot_id
        zone_id = cand.zone_id
        self.pending_zones.discard(zone_id)
        self.scheduled_zones.add(zone_id)
        self.assigned_zones.add(zone_id)
        self.robot_active_zone[rid] = zone_id
        self.robot_state[rid] = "busy"
        t = max(self.now, self.robot_busy_until.get(rid, self.now))
        soc = self.robot_soc[rid]

        for step in cand.steps:
            if step.kind == "travel_charge" or step.kind == "travel_zone" or step.kind == "travel_dock":
                arr = t + step.travel_s
                self.total_travel_m += step.travel_m
                soc, death_t, dead = apply_drain(soc, self.scenario.battery["drain_move"], step.travel_s)
                if dead:
                    dt = death_t if death_t is not None else step.travel_s
                    self._kill_robot(rid, t + dt, 0.0)
                    return
                self._push(
                    SimEvent(arr, "arrive", rid, {"target": step.site_id or step.charger_id or step.dock_id})
                )
                t = arr
                if step.charger_id or (step.site_id and step.site_id.startswith("C")):
                    site_key = step.charger_id or step.site_id
                    if site_key is not None:
                        self.robot_pose[rid] = self._site_pose(site_key)
                elif step.dock_id:
                    self.robot_pose[rid] = self._site_pose(step.dock_id)
                elif step.site_id and step.site_id.startswith("Z"):
                    z = next(z for z in self.scenario.zones if z["id"] == step.site_id)
                    self.robot_pose[rid] = tuple(z["centroid"])

            elif step.kind == "charge":
                cid = step.charger_id or ""
                wait = 0.0
                arr = t
                slot = self.calendar.next_fit(cid, arr, step.service_s)
                if slot is None:
                    return
                wait = slot - arr
                self.charge_wait_s += wait
                soc = soc - self.scenario.battery["drain_idle"] * wait
                if soc <= 0.0:
                    self._kill_robot(rid, arr + wait, 0.0)
                    return
                self._book_occupy(cid, slot, slot + step.service_s, rid)
                self._push(SimEvent(slot, "start_charge", rid, {"charger_id": cid}))
                fin = slot + step.service_s
                soc = apply_charge(soc, self.scenario.battery["charge_rate"], step.service_s)
                self._push(SimEvent(fin, "finish_charge", rid, {"charger_id": cid}))
                t = fin

            elif step.kind == "work":
                self._push(SimEvent(t, "start_work", rid, {"zone_id": zone_id}))
                fin = t + step.work_s
                soc_after, death_t, dead = apply_drain(
                    soc, self.scenario.battery["drain_work"], step.work_s
                )
                soc = soc_after
                if dead:
                    self._kill_robot(
                        rid, t + (death_t if death_t is not None else step.work_s), 0.0
                    )
                    return
                self._push(SimEvent(fin, "finish_work", rid, {"zone_id": zone_id}))
                t = fin

            elif step.kind == "dock":
                did = step.dock_id or ""
                arr = t
                slot = self.calendar.next_fit(did, arr, step.service_s)
                if slot is None:
                    return
                wait = slot - arr
                self.dock_wait_s += wait
                soc = soc - self.scenario.battery["drain_idle"] * wait
                if soc <= 0.0:
                    self._kill_robot(rid, arr + wait, 0.0)
                    return
                self._book_occupy(did, slot, slot + step.service_s, rid)
                self._push(SimEvent(slot, "start_dock", rid, {"dock_id": did}))
                fin = slot + step.service_s
                soc, _, dead = apply_drain(soc, self.scenario.battery["drain_idle"], step.service_s)
                if dead:
                    self._kill_robot(rid, fin, 0.0)
                    return
                self._push(SimEvent(fin, "finish_dock", rid, {"dock_id": did}))
                t = fin

        self.robot_soc[rid] = soc
        self.robot_busy_until[rid] = t
        self._push(SimEvent(t, "arrive", rid, {"free": True}))

    def _commit_random_raw(
        self,
        free_robots: list[RobotState],
        pending: list[int],
        pick: tuple[int, int],
    ) -> None:
        r_idx, t_idx = pick
        if r_idx >= len(free_robots) or t_idx >= len(pending):
            return
        rob = free_robots[r_idx]
        zi = pending[t_idx]
        zone = self.scenario.zones[zi]
        zone_id = zone["id"]
        rn = node_for_pose(self.scenario, rob.pose)
        zn = self.scenario.site_to_node[zone_id]
        leg_s = travel_time(self.scenario, rn, zn)
        if not math.isfinite(leg_s):
            return
        self.pending_zones.discard(zone_id)
        self.assigned_zones.add(zone_id)
        self.robot_active_zone[rob.robot_id] = zone_id
        self.robot_state[rob.robot_id] = "busy"
        t = max(self.now, rob.ready_time)
        soc = rob.soc
        arr = t + leg_s
        self.total_travel_m += travel_distance(self.scenario, rn, zn)
        soc, death_t, dead = apply_drain(soc, self.scenario.battery["drain_move"], leg_s)
        if dead:
            self._kill_robot(rob.robot_id, t + (death_t or 0.0), 0.0)
            return
        self._push(SimEvent(arr, "arrive", rob.robot_id, {"zone_id": zone_id}))
        self._push(SimEvent(arr, "start_work", rob.robot_id, {"zone_id": zone_id}))
        fin = arr + float(zone["work_s"])
        soc, dt, dead = apply_drain(soc, self.scenario.battery["drain_work"], float(zone["work_s"]))
        if dead:
            self._kill_robot(rob.robot_id, arr + (dt or float(zone["work_s"])), 0.0)
            return
        self._push(SimEvent(fin, "finish_work", rob.robot_id, {"zone_id": zone_id}))
        self.robot_soc[rob.robot_id] = soc
        self.robot_busy_until[rob.robot_id] = fin
        self.robot_pose[rob.robot_id] = tuple(zone["centroid"])
        self._push(SimEvent(fin, "arrive", rob.robot_id, {"free": True}))

    def _idle_drain_free_robots(self, until_t: float) -> None:
        rate = self.scenario.battery["drain_idle"]
        for r in self.scenario.robots:
            rid = r["id"]
            if self.robot_dead.get(rid, False):
                continue
            if self.robot_state.get(rid, "free") != "free":
                continue
            start = max(self.now, self.robot_busy_until.get(rid, 0.0))
            if until_t <= start + 1e-12:
                continue
            dt = until_t - start
            soc = self.robot_soc[rid] - rate * dt
            if soc <= 0.0:
                self._kill_robot(rid, start + self.robot_soc[rid] / rate if rate > 0 else until_t, 0.0)
            else:
                self.robot_soc[rid] = soc

    def _never_charge_idle_drain(self, free: list[RobotState]) -> None:
        rate = self.scenario.battery["drain_idle"]
        for rob in free:
            if self.robot_dead.get(rob.robot_id, False):
                continue
            if self.robot_state.get(rob.robot_id) != "free":
                continue
            if rob.soc <= 0.0:
                self._kill_robot(rob.robot_id, self.now, 0.0, record_now=True)
                continue
            if not self._coverage_incomplete():
                continue
            t_death = self.now + rob.soc / rate
            cap = self.scenario.T_horizon
            if t_death > cap + 1e-6:
                self._kill_robot(rob.robot_id, cap, 0.0, record_now=True)
                continue
            self._push(SimEvent(t_death, "battery_empty", rob.robot_id, {"idle_stuck": True}))

    def _maybe_idle_charge(self, free: list[RobotState]) -> None:
        if not self._allow_charge():
            return
        for rob in free:
            if self.robot_dead.get(rob.robot_id, False):
                continue
            if not self._can_reach_charger(rob):
                self._schedule_charge(rob)
            else:
                self._schedule_soc_watch(rob)

    def _can_reach_charger(self, rob: RobotState) -> bool:
        rn = node_for_pose(self.scenario, rob.pose)
        for c in self.scenario.chargers:
            cn = self.scenario.site_to_node[c["id"]]
            ts = travel_time(self.scenario, rn, cn)
            if not math.isfinite(ts):
                continue
            soc, _, dead = apply_drain(rob.soc, self.scenario.battery["drain_move"], ts)
            if dead:
                continue
            if soc >= MIN_RESERVE:
                return True
        return False

    def _schedule_charge(self, rob: RobotState) -> None:
        rn = node_for_pose(self.scenario, rob.pose)
        best: tuple[str, float, float, Candidate | None] | None = None
        RobotState(rob.robot_id, rob.pose, rob.soc, rob.ready_time)
        for c in self.scenario.chargers:
            cid = c["id"]
            RobotState(rob.robot_id, rob.pose, rob.soc, max(self.now, rob.ready_time))
            cn = self.scenario.site_to_node[cid]
            ts = travel_time(self.scenario, rn, cn)
            if not math.isfinite(ts):
                continue
            arr = max(self.now, rob.ready_time) + ts
            soc_arr, _, dead = apply_drain(rob.soc, self.scenario.battery["drain_move"], ts)
            if dead:
                continue
            fill = charge_duration_to_full(soc_arr, self.scenario.battery["charge_rate"])
            slot = self.calendar.next_fit(cid, arr, fill)
            if slot is None:
                continue
            finish = slot + fill
            if best is None or finish < best[1]:
                best = (cid, finish, ts, None)
        if best is None:
            return
        cid, finish, ts, _ = best
        rid = rob.robot_id
        self.robot_state[rid] = "busy"
        t0 = max(self.now, rob.ready_time)
        arr = t0 + ts
        self.total_travel_m += travel_distance(
            self.scenario, rn, self.scenario.site_to_node[cid]
        )
        soc = rob.soc
        soc, dt, dead = apply_drain(soc, self.scenario.battery["drain_move"], ts)
        if dead:
            self._kill_robot(rid, t0 + (dt or ts), 0.0)
            return
        wait = 0.0
        fill = charge_duration_to_full(soc, self.scenario.battery["charge_rate"])
        slot = self.calendar.next_fit(cid, arr, fill)
        if slot is None:
            return
        wait = slot - arr
        self.charge_wait_s += wait
        soc = soc - self.scenario.battery["drain_idle"] * wait
        if soc <= 0:
            self._kill_robot(rid, arr + wait, 0.0)
            return
        self._book_occupy(cid, slot, slot + fill, rid)
        self._push(SimEvent(slot, "start_charge", rid, {"charger_id": cid}))
        fin = slot + fill
        soc = apply_charge(soc, self.scenario.battery["charge_rate"], fill)
        self._push(SimEvent(fin, "finish_charge", rid, {"charger_id": cid}))
        self.robot_soc[rid] = soc
        self.robot_busy_until[rid] = fin
        self.robot_pose[rid] = self._site_pose(cid)
        self._push(SimEvent(fin, "arrive", rid, {"free": True}))

    def _site_pose(self, site_id: str) -> tuple[float, float]:
        for coll in (self.scenario.chargers, self.scenario.docks, self.scenario.zones):
            for item in coll:
                if item.get("id") == site_id:
                    if "centroid" in item:
                        return tuple(item["centroid"])
                    return tuple(item["pose"])
        raise KeyError(site_id)

    def _book_occupy(self, site: str, start: float, end: float, rid: str) -> None:
        self.calendar.book(site, start, end, rid)
        self.log.occupy.append({"site_id": site, "start": start, "end": end, "robot_id": rid})

    def _kill_robot(self, rid: str, t: float, soc: float, *, record_now: bool = False) -> None:
        if self.robot_dead.get(rid, False):
            return
        self.robot_dead[rid] = True
        self.robot_soc[rid] = max(0.0, soc)
        self.deaths += 1
        self.robot_state[rid] = "dead"
        if rid in self.log.robots:
            self.log.robots[rid]["dead"] = True
            self.log.robots[rid]["soc_end"] = 0.0
        zid = self.robot_active_zone.pop(rid, None)
        if zid and zid not in self.log.zones_done:
            self.pending_zones.add(zid)
            self.assigned_zones.discard(zid)
            self.scheduled_zones.discard(zid)
        horizon = self.scenario.T_horizon
        for site_id in list(self.calendar.intervals.keys()):
            if self.calendar.block_from_death(site_id, rid, t, horizon):
                for rec in self.log.occupy:
                    if (
                        rec["site_id"] == site_id
                        and rec["robot_id"] == rid
                        and rec["start"] <= t + 1e-12
                        and t < rec["end"] - 1e-12
                    ):
                        rec["end"] = max(float(rec["end"]), horizon)
        ev = SimEvent(t, "battery_empty", rid, {})
        if record_now:
            self._record(ev)
        else:
            self._push(ev)

    def _schedule_soc_watch(self, rob: RobotState) -> None:
        if not self._allow_charge():
            return
        rn = node_for_pose(self.scenario, rob.pose)
        rate_idle = self.scenario.battery["drain_idle"]
        rate_move = self.scenario.battery["drain_move"]
        best_death = float("inf")
        for c in self.scenario.chargers:
            cn = self.scenario.site_to_node[c["id"]]
            ts = travel_time(self.scenario, rn, cn)
            if not math.isfinite(ts):
                continue
            soc = rob.soc
            soc, _, dead = apply_drain(soc, rate_move, ts)
            if dead:
                best_death = min(best_death, rob.ready_time + (rob.soc / rate_move if rate_move > 0 else 0))
                continue
            need = soc - MIN_RESERVE
            if need <= 0:
                return
            if rate_idle <= 0:
                return
            t_death = self.now + need / rate_idle
            best_death = min(best_death, t_death)
        if math.isfinite(best_death) and best_death <= self.scenario.T_horizon + 1e-6:
            self._push(
                SimEvent(
                    best_death,
                    "soc_watch",
                    rob.robot_id,
                    {"needs_charge": True},
                )
            )

    def run(self) -> RunLog:
        from multi_robot_task_allocation.world import world_sha256

        self.log.world_sha = world_sha256(self.scenario)
        for r in self.scenario.robots:
            rid = r["id"]
            self.robot_pose[rid] = tuple(r["start"])
            self.robot_soc[rid] = float(r["start_soc"])
            self.robot_dead[rid] = False
            self.robot_busy_until[rid] = 0.0
            self.robot_state[rid] = "free"
            self.log.robots[rid] = {
                "start": list(r["start"]),
                "start_soc": r["start_soc"],
                "dead": False,
                "soc_end": r["start_soc"],
            }

        self._push(SimEvent(self.scenario.T_horizon, "horizon", "", {}))
        self._run_epoch()

        while self.queue:
            batch_time = self.queue[0].time
            if (
                batch_time > self.now + 1e-12
                and self.allocator in ("random", "never_charge")
            ):
                self._idle_drain_free_robots(batch_time)
            batch: list[SimEvent] = []
            while self.queue and self.queue[0].time <= batch_time + 1e-12:
                batch.append(heapq.heappop(self.queue))
            batch.sort()
            for ev in batch:
                if ev.time > self.scenario.T_horizon + 1e-9 and ev.kind != "horizon":
                    continue
                self.now = ev.time
                self._handle_event(ev)
            if batch_time <= self.scenario.T_horizon + 1e-9:
                self._run_epoch()

        self._finalize()
        return self.log

    def _handle_event(self, ev: SimEvent) -> None:
        self._record(ev)
        rid = ev.robot_id
        if ev.kind == "finish_work":
            zid = ev.payload.get("zone_id")
            if zid and zid not in self.log.zones_done:
                self.log.zones_done.add(zid)
            if rid:
                self.robot_active_zone.pop(rid, None)
        elif ev.kind == "arrive" and ev.payload.get("free"):
            if rid and not self.robot_dead.get(rid, False):
                self.robot_state[rid] = "free"
                self.robot_busy_until[rid] = ev.time
                self.now = ev.time
                rob = RobotState(
                    rid,
                    self.robot_pose[rid],
                    self.robot_soc[rid],
                    ev.time,
                )
                if self.allocator in ("never_charge", "random"):
                    if self._coverage_incomplete() and not self._can_reach_charger(rob):
                        self._kill_robot(rid, ev.time, 0.0, record_now=True)
                    elif self._coverage_incomplete() and rob.soc < MIN_RESERVE:
                        self._kill_robot(rid, ev.time, 0.0, record_now=True)
                elif (
                    self._allow_charge()
                    and not self._can_reach_charger(rob)
                ):
                    self._schedule_charge(rob)
                elif ev.time <= self.scenario.T_horizon + 1e-9 and (
                    self.pending_zones or self._coverage_incomplete()
                ):
                    self._run_epoch()
        elif ev.kind == "battery_empty":
            if rid and not self.robot_dead.get(rid, False):
                self._kill_robot(rid, ev.time, 0.0, record_now=True)
        elif ev.kind == "soc_watch":
            if rid and not self.robot_dead.get(rid, False) and self._allow_charge():
                rob = RobotState(
                    rid,
                    self.robot_pose[rid],
                    self.robot_soc[rid],
                    ev.time,
                )
                if ev.payload.get("needs_charge") and not self._can_reach_charger(rob):
                    self._schedule_charge(rob)
        elif ev.kind == "horizon":
            pass

    def _finalize(self) -> None:
        required = set(self.log.zones_required)
        done = self.log.zones_done
        coverage = len(done & required) / len(required) if required else 0.0
        all_free = all(
            self.robot_state.get(r["id"], "free") == "free" or self.robot_dead.get(r["id"], False)
            for r in self.scenario.robots
        )
        self.log.complete = (
            coverage >= 1.0 - 1e-12
            and len(done) >= len(required)
            and all_free
            and self.deaths == 0
        )
        last_t = max(
            (
                e["t"]
                for e in self.log.events
                if e.get("kind") not in ("horizon", "summary")
            ),
            default=0.0,
        )
        if self.log.complete:
            self.log.makespan_s = last_t
            self.log.reason = "all_zones_complete"
        else:
            self.log.makespan_s = max(last_t, self.scenario.T_horizon)
            self.log.reason = "horizon_or_incomplete"
        for rid, meta in self.log.robots.items():
            if not meta["dead"]:
                meta["soc_end"] = self.robot_soc.get(rid, meta["soc_end"])
        meta = {
            "coverage": coverage,
            "deaths": self.deaths,
            "dock_wait_s": self.dock_wait_s,
            "charge_wait_s": self.charge_wait_s,
            "total_travel_m": self.total_travel_m,
            "slot_conflicts": 0,
        }
        self.log.events.append({"t": self.now, "kind": "summary", "meta": meta})


def simulate(scenario: Scenario, allocator: str, mutant_seed: int | None = None) -> RunLog:
    global _INSERT_COUNTER
    _INSERT_COUNTER = 0
    sim = Simulator(scenario, allocator, mutant_seed)
    return sim.run()


def runlog_to_json(log: RunLog) -> dict[str, Any]:
    return {
        "actions": log.actions,
        "allocator": log.allocator,
        "complete": log.complete,
        "events": log.events,
        "makespan_s": log.makespan_s,
        "mutant_seed": log.mutant_seed,
        "occupy": log.occupy,
        "reason": log.reason,
        "robots": log.robots,
        "schema_version": log.schema_version,
        "scenario_family": log.scenario_family,
        "seed": log.seed,
        "T_horizon": log.T_horizon,
        "world_sha": log.world_sha,
        "zones_done": sorted(log.zones_done),
        "zones_required": log.zones_required,
    }


def canonical_runlog_sha(log: RunLog) -> str:
    import hashlib

    payload = json.dumps(runlog_to_json(log), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode()).hexdigest()
