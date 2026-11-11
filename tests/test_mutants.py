# SPDX-License-Identifier: Apache-2.0
import math

from multi_robot_task_allocation.eval.gates import EVAL_SEEDS, metrics_from_log
from multi_robot_task_allocation.eval.generate import generate
from multi_robot_task_allocation.sim import simulate


def test_never_charge_s2() -> None:
    for seed in EVAL_SEEDS["S2"]:
        log = simulate(generate("s2", seed), "never_charge", None)
        m = metrics_from_log(log)
        assert m["deaths"] >= 1
        assert not m["coverage_complete"]
        assert not log.complete
        def has_nan(obj: object) -> bool:
            if isinstance(obj, float):
                return math.isnan(obj)
            if isinstance(obj, dict):
                return any(has_nan(v) for v in obj.values())
            if isinstance(obj, list):
                return any(has_nan(v) for v in obj)
            return False

        assert not has_nan(log.events)


def test_random_fails_all_eval() -> None:
    for fam, seeds in EVAL_SEEDS.items():
        for seed in seeds:
            log = simulate(generate(fam.lower(), seed), "random", seed)
            m = metrics_from_log(log)
            assert not m["pass"]
