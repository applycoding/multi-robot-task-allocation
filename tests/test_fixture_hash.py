# SPDX-License-Identifier: Apache-2.0
import subprocess
import sys
from pathlib import Path

import pytest

from multi_robot_task_allocation.eval.gates import EVAL_SEEDS
from multi_robot_task_allocation.eval.generate import generate
from multi_robot_task_allocation.world import world_sha256

FIXTURES = Path(__file__).resolve().parent.parent / "fixtures"


@pytest.mark.parametrize(
    "family,seed",
    [(f, s) for f, seeds in EVAL_SEEDS.items() for s in seeds],
)
def test_world_sha_matches_sidecar(family: str, seed: int) -> None:
    sc = generate(family.lower(), seed)
    sha = world_sha256(sc)
    sidecar = FIXTURES / f"scenario_{family.lower()}_{seed}.json.sha256"
    if sidecar.exists():
        assert sidecar.read_text().strip() == sha


def test_world_sha_stable_two_processes() -> None:
    sc = generate("s1", 20261111)
    a = world_sha256(sc)
    proc = subprocess.run(
        [
            sys.executable,
            "-c",
            "from multi_robot_task_allocation.eval.generate import generate;"
            "from multi_robot_task_allocation.world import world_sha256;"
            "print(world_sha256(generate('s1', 20261111)))",
        ],
        env={"PYTHONPATH": "src"},
        capture_output=True,
        text=True,
        cwd=Path(__file__).resolve().parent.parent,
        check=True,
    )
    assert proc.stdout.strip() == a
