# SPDX-License-Identifier: Apache-2.0
import json
from pathlib import Path

from multi_robot_task_allocation.eval.report import (
    load_runlog,
    log_sha,
    validate_manifest,
)


def test_report_check_sample_file() -> None:
    sample = Path(__file__).resolve().parent.parent / "web" / "public" / "sample_report.json"
    if not sample.exists():
        return
    report = json.loads(sample.read_text())
    validate_manifest(report["rows"])
    Path(__file__).resolve().parent.parent / "artifacts" / "runs"
    assert report["sample"] is True
    assert report["git_sha"]


def test_log_sha_stable() -> None:
    from multi_robot_task_allocation.eval.generate import generate
    from multi_robot_task_allocation.sim import runlog_to_json, simulate

    log = simulate(generate("s1", 20261111), "greedy", None)
    path = Path("/tmp/mrta_run.json")
    path.write_text(json.dumps(runlog_to_json(log)))
    loaded = load_runlog(path)
    assert log_sha(loaded) == log_sha(log)
