# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math

import numpy as np

from multi_robot_task_allocation.alloc.hungarian import (
    brute_force_assign,
    hungarian_assign,
    hungarian_cost_sum,
)


def kuhn_munkres_assign(cost: np.ndarray) -> list[tuple[int, int]]:
    """Independent min-cost max-cardinality assignment used as a cost-sum oracle.

    Forbidden entries stay non-finite and are never assigned. This does not apply
    the production lex tie-break; agreement is on cardinality and finite cost sum.
    """
    nr, nc = cost.shape
    if nr == 0 or nc == 0:
        return []
    edges: list[tuple[int, int, float]] = []
    for i in range(nr):
        for j in range(nc):
            c = float(cost[i, j])
            if math.isfinite(c):
                edges.append((i, j, c))
    if not edges:
        return []
    vals = [c for _, _, c in edges]
    shift = (max(abs(v) for v in vals) + 1.0) * (min(nr, nc) + 1)

    robot_task = [-1] * nr
    task_robot = [-1] * nc

    def augment() -> bool:
        n_nodes = nr + nc
        inf = float("inf")
        dist = [inf] * n_nodes
        prev: list[tuple[int, str]] = [(-1, "") for _ in range(n_nodes)]
        for i in range(nr):
            if robot_task[i] < 0:
                dist[i] = 0.0
        for _ in range(n_nodes):
            updated = False
            for i, j, c in edges:
                tc = c - shift
                if robot_task[i] != j and dist[i] + tc < dist[nr + j] - 1e-12:
                    dist[nr + j] = dist[i] + tc
                    prev[nr + j] = (i, "fwd")
                    updated = True
                if robot_task[i] == j and dist[nr + j] - tc < dist[i] - 1e-12:
                    dist[i] = dist[nr + j] - tc
                    prev[i] = (nr + j, "bwd")
                    updated = True
            if not updated:
                break
        best_j: int | None = None
        best_d = inf
        for j in range(nc):
            if task_robot[j] < 0 and dist[nr + j] < best_d - 1e-12:
                best_d = dist[nr + j]
                best_j = j
        if best_j is None:
            return False
        j = best_j
        while True:
            i, kind = prev[nr + j]
            assert kind == "fwd"
            prior = robot_task[i]
            robot_task[i] = j
            task_robot[j] = i
            if prior < 0:
                break
            j = prior
        return True

    while augment():
        pass
    return [(i, robot_task[i]) for i in range(nr) if robot_task[i] >= 0]


def _agree(cost: np.ndarray, travel: np.ndarray | None = None) -> None:
    if travel is None:
        travel = np.zeros_like(cost, dtype=float)
    hung = hungarian_assign(cost, travel)
    brute = brute_force_assign(cost, travel)
    km = kuhn_munkres_assign(cost)
    assert hung == brute
    assert len(brute) == len(km)
    sum_brute = hungarian_cost_sum(brute, cost)
    sum_km = hungarian_cost_sum(km, cost)
    if math.isinf(sum_brute):
        assert math.isinf(sum_km)
    else:
        assert abs(sum_brute - sum_km) <= 1e-9


def test_hungarian_2x3_sum6() -> None:
    cost = np.array([[5.0, 9.0, 2.0], [8.0, 4.0, 7.0]])
    travel = np.zeros_like(cost)
    pairs = hungarian_assign(cost, travel)
    assert sorted(pairs) == [(0, 2), (1, 1)]
    assert hungarian_cost_sum(pairs, cost) == 6.0
    _agree(cost, travel)


def test_equal_cost_2x2_lex() -> None:
    cost = np.array([[3.0, 3.0], [3.0, 3.0]])
    travel = np.zeros_like(cost)
    pairs = brute_force_assign(cost, travel)
    assert pairs == [(0, 0), (1, 1)]
    _agree(cost, travel)


def test_forbidden_inf() -> None:
    cost = np.array([[float("inf"), 2.0], [3.0, float("inf")]])
    travel = np.zeros_like(cost)
    pairs = brute_force_assign(cost, travel)
    assert pairs == [(0, 1), (1, 0)]
    _agree(cost, travel)


def test_empty_and_all_forbidden() -> None:
    _agree(np.zeros((0, 0)))
    _agree(np.zeros((0, 3)))
    _agree(np.zeros((3, 0)))
    _agree(np.full((2, 3), np.inf))


def test_rectangular_1x3_and_3x1() -> None:
    _agree(np.array([[4.0, 1.0, 7.0]]))
    _agree(np.array([[4.0], [1.0], [7.0]]))


def test_hall_deficient_and_mixed_inf() -> None:
    # Two robots, two tasks, but only task 0 is reachable: cardinality 1.
    cost = np.array([[2.0, np.inf], [5.0, np.inf]])
    pairs = hungarian_assign(cost, np.zeros_like(cost))
    assert len(pairs) == 1
    assert pairs[0][1] == 0
    _agree(cost)


def test_4x6_seed0_with_forbidden() -> None:
    rng = np.random.default_rng(0)
    cost = rng.random((4, 6)) * 10.0
    idx = rng.choice(24, size=6, replace=False)
    cost.ravel()[idx] = np.inf
    _agree(cost)
