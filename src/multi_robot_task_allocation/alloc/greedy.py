# SPDX-License-Identifier: Apache-2.0
"""Greedy allocator with matrix rebuild."""
from __future__ import annotations

import math

import numpy as np

from multi_robot_task_allocation.alloc.wrapper import compare_pairs


def greedy_assign(cost: np.ndarray, travel: np.ndarray) -> list[tuple[int, int]]:
    """One epoch: pick feasible pairs greedily on frozen matrix."""
    nr, nc = cost.shape
    used_r: set[int] = set()
    used_t: set[int] = set()
    pairs: list[tuple[int, int]] = []

    while True:
        best: tuple[int, int] | None = None
        best_cost = float("inf")
        best_travel = float("inf")
        for r in range(nr):
            if r in used_r:
                continue
            for t in range(nc):
                if t in used_t:
                    continue
                c = float(cost[r, t])
                if not math.isfinite(c):
                    continue
                tr = float(travel[r, t])
                if c < best_cost - 1e-9:
                    best = (r, t)
                    best_cost = c
                    best_travel = tr
                elif abs(c - best_cost) <= 1e-9 and best is not None:
                    if compare_pairs(c, tr, r, t, best_cost, best_travel, best[0], best[1]) < 0:
                        best = (r, t)
                        best_cost = c
                        best_travel = tr
        if best is None:
            break
        pairs.append(best)
        used_r.add(best[0])
        used_t.add(best[1])
    return pairs

