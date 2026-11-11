# SPDX-License-Identifier: Apache-2.0
import math

import numpy as np

from multi_robot_task_allocation.alloc.auction import auction_assign, auction_cost_sum
from multi_robot_task_allocation.alloc.hungarian import brute_force_assign, hungarian_cost_sum


def test_auction_bound_main_matrix() -> None:
    cost = np.array([[5.0, 9.0, 2.0], [8.0, 4.0, 7.0]])
    travel = np.zeros_like(cost)
    hung = brute_force_assign(cost, travel)
    opt = hungarian_cost_sum(hung, cost)
    auc_pairs = auction_assign(cost, travel)
    for robot_i, task_j in auc_pairs:
        assert math.isfinite(float(cost[robot_i, task_j]))
    real_sum = auction_cost_sum(auc_pairs, cost)
    assert real_sum <= opt + 2.0 + 1e-9


def test_auction_bound_alt_matrix() -> None:
    cost = np.array([[0.0, 0.2], [0.2, 0.0]])
    travel = np.zeros_like(cost)
    hung = brute_force_assign(cost, travel)
    opt = hungarian_cost_sum(hung, cost)
    auc_pairs = auction_assign(cost, travel)
    for robot_i, task_j in auc_pairs:
        assert math.isfinite(float(cost[robot_i, task_j]))
    real_sum = auction_cost_sum(auc_pairs, cost)
    assert real_sum <= opt + 2.0 + 1e-9
