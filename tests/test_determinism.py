# SPDX-License-Identifier: Apache-2.0
import pytest

from multi_robot_task_allocation.eval.generate import generate
from multi_robot_task_allocation.sim import canonical_runlog_sha, simulate


@pytest.mark.parametrize("allocator", ["greedy", "hungarian", "auction"])
def test_deterministic_runlog(allocator: str) -> None:
    sc = generate("s1", 20261111)
    a = canonical_runlog_sha(simulate(sc, allocator, None))
    b = canonical_runlog_sha(simulate(sc, allocator, None))
    assert a == b
