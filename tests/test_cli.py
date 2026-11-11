# SPDX-License-Identifier: Apache-2.0
import json
import tempfile
from pathlib import Path

from multi_robot_task_allocation.cli import main


def test_cli_generate_run() -> None:
    with tempfile.TemporaryDirectory() as td:
        scenario = Path(td) / "scenario.json"
        run = Path(td) / "run.json"
        assert main(["generate", "--family", "s1", "--seed", "20261111", "--out", str(scenario)]) == 0
        data = json.loads(scenario.read_text())
        assert data["family"] == "S1"
        assert main(["run", "--scenario", str(scenario), "--allocator", "greedy", "--out", str(run)]) == 0
        log = json.loads(run.read_text())
        assert log["allocator"] == "greedy"
