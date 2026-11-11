# SPDX-License-Identifier: Apache-2.0
"""Bertsekas-style forward auction on benefit minus price."""
from __future__ import annotations

import math

import numpy as np

AUCTION_EPSILON = 0.5
AUCTION_ROUND_CAP = 20000


class AuctionRoundCap(RuntimeError):
    """Auction exceeded round cap."""


def auction_assign(
    cost: np.ndarray,
    travel: np.ndarray,
    n_dummy: int = 4,
) -> list[tuple[int, int]]:
    del travel
    nr, nc = cost.shape
    finite_costs = [float(cost[r, t]) for r in range(nr) for t in range(nc) if math.isfinite(cost[r, t])]
    if not finite_costs:
        return []
    c_big = max(finite_costs) + 1.0

    n_objects = nc + n_dummy
    benefit = np.zeros((nr, n_objects))
    for r in range(nr):
        for t in range(nc):
            c = float(cost[r, t])
            if math.isfinite(c):
                benefit[r, t] = c_big - c
        for d in range(n_dummy):
            benefit[r, nc + d] = 0.0

    price = np.zeros(n_objects)
    owner = np.full(n_objects, -1, dtype=np.int32)
    assigned_robot: dict[int, int] = {}

    rounds = 0
    while len(assigned_robot) < nr and rounds < AUCTION_ROUND_CAP:
        rounds += 1
        bids: dict[int, tuple[int, float]] = {}
        for r in range(nr):
            if r in assigned_robot:
                continue
            values = benefit[r] - price
            forbidden = np.zeros(n_objects, dtype=bool)
            for t in range(nc):
                if not math.isfinite(float(cost[r, t])):
                    forbidden[t] = True
            values = np.where(forbidden, -np.inf, values)

            best_j = -1
            best_v = -np.inf
            second_v = 0.0
            for j in range(n_objects):
                v = float(values[j])
                if v > best_v + 1e-15:
                    second_v = best_v if best_j >= 0 else 0.0
                    best_v = v
                    best_j = j
                elif j < nc or j >= nc:
                    if v > second_v and abs(v - best_v) > 1e-15:
                        second_v = v
            if best_j < 0 or not math.isfinite(best_v) or best_v < -1e15:
                continue
            bid_inc = (best_v - second_v) + AUCTION_EPSILON
            bids[r] = (best_j, bid_inc)

        if not bids:
            break

        for obj, obj_bidders in _group_by_object(bids).items():
            if not obj_bidders:
                continue
            r_star, (_obj_j, inc_star) = max(obj_bidders, key=lambda x: x[1][1])
            price[obj] += float(inc_star)
            prev = int(owner[obj])
            if prev >= 0:
                del assigned_robot[prev]
            owner[obj] = r_star
            assigned_robot[r_star] = obj

    if rounds >= AUCTION_ROUND_CAP and len(assigned_robot) < nr:
        raise AuctionRoundCap("auction round cap")

    pairs: list[tuple[int, int]] = []
    for r, obj in assigned_robot.items():
        if obj < nc and math.isfinite(float(cost[r, obj])):
            pairs.append((r, obj))
    return pairs


def _group_by_object(
    bids: dict[int, tuple[int, float]],
) -> dict[int, list[tuple[int, tuple[int, float]]]]:
    out: dict[int, list[tuple[int, tuple[int, float]]]] = {}
    for r, (j, inc) in bids.items():
        out.setdefault(j, []).append((r, (j, inc)))
    return out


def auction_cost_sum(pairs: list[tuple[int, int]], cost: np.ndarray) -> float:
    s = 0.0
    for r, t in pairs:
        if t >= cost.shape[1]:
            continue
        c = float(cost[r, t])
        if not math.isfinite(c):
            return float("inf")
        s += c
    return s
