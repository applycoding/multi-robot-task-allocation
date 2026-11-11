# SPDX-License-Identifier: Apache-2.0
from multi_robot_task_allocation.eval.gates import EVAL_SEEDS, HORIZONS, metrics_from_log
from multi_robot_task_allocation.eval.generate import generate
from multi_robot_task_allocation.sim import simulate


def test_s4_hungarian_makespan_vs_greedy() -> None:
    for seed in EVAL_SEEDS["S4"]:
        sc = generate("s4", seed)
        g = metrics_from_log(simulate(sc, "greedy", None))
        h = metrics_from_log(simulate(sc, "hungarian", None))
        assert h["makespan_s"] <= g["makespan_s"] + 1e-3


def test_s3_dock_wait_positive() -> None:
    for seed in EVAL_SEEDS["S3"]:
        for alloc in ("greedy", "hungarian", "auction"):
            sc = generate("s3", seed)
            m = metrics_from_log(simulate(sc, alloc, None))
            assert m["dock_wait_s"] > 0.0


def test_s2_start_soc_frozen() -> None:
    sc = generate("s2", 20261121)
    assert all(r["start_soc"] == 0.35 for r in sc.robots)


def test_horizons_frozen() -> None:
    assert HORIZONS == {"S1": 150.0, "S2": 200.0, "S3": 300.0, "S4": 200.0}
