# SPDX-License-Identifier: Apache-2.0
"""Exact enumeration labelled Hungarian, with a cardinality-first objective.

For this 4-robot fleet, hungarian_assign enumerates legal assignments.
It is not a Kuhn-Munkres implementation. Tests compare its cardinality
and finite cost-sum against a test-only Kuhn-Munkres oracle.
"""
from __future__ import annotations

import itertools
import math
from typing import Sequence

import numpy as np


def _is_finite(x: float) -> bool:
    return math.isfinite(x)


def brute_force_assign(
    cost: np.ndarray,
    travel: np.ndarray,
) -> list[tuple[int, int]]:
    """Maximum cardinality, then min cost sum, then lex tie-break."""
    nr, nc = cost.shape
    robots = list(range(nr))
    tasks = list(range(nc))
    k = min(nr, nc)
    best_pairs: list[tuple[int, int]] = []
    best_sum = float("inf")
    best_card = -1

    for card in range(k, -1, -1):
        for rob_subset in itertools.combinations(robots, card):
            for task_subset in itertools.combinations(tasks, card):
                for perm in itertools.permutations(task_subset):
                    pairs = list(zip(rob_subset, perm, strict=True))
                    legal = True
                    csum = 0.0
                    for r, t in pairs:
                        c = float(cost[r, t])
                        if not _is_finite(c):
                            legal = False
                            break
                        csum += c
                    if not legal:
                        continue
                    if card > best_card:
                        best_card = card
                        best_sum = csum
                        best_pairs = pairs
                    elif card == best_card:
                        if csum < best_sum - 1e-9:
                            best_sum = csum
                            best_pairs = pairs
                        elif abs(csum - best_sum) <= 1e-9:
                            if _lex_better_pairs(pairs, best_pairs, cost, travel):
                                best_pairs = pairs
        if best_card == card and best_pairs:
            break
    return best_pairs


def _lex_better_pairs(
    a: list[tuple[int, int]],
    b: list[tuple[int, int]],
    cost: np.ndarray,
    travel: np.ndarray,
) -> bool:
    def key(pairs: list[tuple[int, int]]) -> list[tuple[float, float, int, int]]:
        items = []
        for r, t in pairs:
            items.append((float(cost[r, t]), float(travel[r, t]), r, t))
        items.sort(key=lambda x: (x[0], x[2], x[3]))
        return items

    return key(a) < key(b)


def hungarian_assign(
    cost: np.ndarray,
    travel: np.ndarray,
) -> list[tuple[int, int]]:
    """Exact enumeration of assignments for this 4-robot fleet."""
    return brute_force_assign(cost, travel)


def assign_epoch(
    allocator: str,
    cost: np.ndarray,
    travel: np.ndarray,
) -> list[tuple[int, int]]:
    del allocator
    return hungarian_assign(cost, travel)


def hungarian_cost_sum(pairs: Sequence[tuple[int, int]], cost: np.ndarray) -> float:
    s = 0.0
    for r, t in pairs:
        c = float(cost[r, t])
        if not _is_finite(c):
            return float("inf")
        s += c
    return s
