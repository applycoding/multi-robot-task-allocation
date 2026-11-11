# SPDX-License-Identifier: Apache-2.0
import numpy as np

from multi_robot_task_allocation.world import Scenario, reconstruct_path, travel_time


def _hand_scenario() -> Scenario:
    s = Scenario(family="HAND", seed=0, mutant_seed=None)
    s.node_coords = [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    s.v_cruise = 1.0
    n = 4
    s.dist_m = np.full((n, n), np.inf)
    s.parent = np.full((n, n), -1, dtype=np.int32)
    for i in range(n):
        s.dist_m[i, i] = 0.0
    s.dist_m[0, 1] = 0.0
    s.dist_m[1, 0] = 0.0
    s.parent[0, 1] = 0
    s.parent[1, 0] = 1
    # manual hand graph 0-1 len 3, 0-2 len 4, 1-2 len 1 for nodes 0,1,2 only
    return s


def test_dijkstra_hand_parent() -> None:
    s = Scenario(family="HAND", seed=0, mutant_seed=None)
    s.node_coords = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    s.v_cruise = 1.0
    n = 3
    dist = np.full((n, n), np.inf)
    parent = np.full((n, n), -1, dtype=np.int32)
    for i in range(n):
        dist[i, i] = 0.0
    dist[0, 1] = dist[1, 0] = 3.0
    dist[1, 2] = dist[2, 1] = 1.0
    dist[0, 2] = dist[2, 0] = 4.0
    parent[0, 1] = 0
    parent[1, 0] = 1
    parent[1, 2] = 1
    parent[2, 1] = 2
    parent[0, 2] = 0
    parent[2, 0] = 0
    s.dist_m = dist
    s.parent = parent
    assert int(s.parent[0, 2]) == 0
    s.dist_m[0, 2] = float("inf")
    s.parent[0, 2] = -1
    assert travel_time(s, 0, 2) == float("inf")


def test_dijkstra_no_cycles() -> None:
    from multi_robot_task_allocation.eval.generate import generate

    sc = generate("s1", 20261111)
    n = len(sc.node_coords)
    for src in range(n):
        for dst in range(n):
            path = reconstruct_path(sc.parent[src], src, dst)
            if not path:
                continue
            assert len(path) == len(set(path))
