# SPDX-License-Identifier: Apache-2.0
import pytest

from multi_robot_task_allocation.eval.gates import EVAL_SEEDS, HORIZONS, metrics_from_log
from multi_robot_task_allocation.eval.generate import generate
from multi_robot_task_allocation.sim import simulate


@pytest.mark.parametrize("family", ["S1", "S2", "S3", "S4"])
@pytest.mark.parametrize("allocator", ["greedy", "hungarian", "auction"])
def test_oracle_complete(family: str, allocator: str) -> None:
    for seed in EVAL_SEEDS[family]:
        sc = generate(family.lower(), seed)
        log = simulate(sc, allocator, None)
        m = metrics_from_log(log)
        assert m["deaths"] == 0
        assert m["slot_conflicts"] == 0
        assert m["coverage_complete"]
        assert log.complete
        assert m["makespan_s"] <= HORIZONS[family] + 1e-3
