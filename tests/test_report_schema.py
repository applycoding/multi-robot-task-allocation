# SPDX-License-Identifier: Apache-2.0
import json
import tempfile
from pathlib import Path

from multi_robot_task_allocation.eval.gates import EVAL_SEEDS, MANIFEST
from multi_robot_task_allocation.eval.generate import generate
from multi_robot_task_allocation.eval.report import build_report, log_sha, reject_non_finite
from multi_robot_task_allocation.sim import runlog_to_json, simulate


def test_manifest_row_count() -> None:
    assert len(MANIFEST) == 55


def test_report_non_finite_rejected() -> None:
    try:
        reject_non_finite({"x": float("nan")})
        raise AssertionError("expected ValueError")
    except ValueError:
        pass


def test_sample_report_builder() -> None:
    with tempfile.TemporaryDirectory() as td:
        runs = Path(td)
        rows = []
        from multi_robot_task_allocation.eval.gates import evaluate_row

        for alloc, fam, seed, expected in MANIFEST[:3]:
            log = simulate(generate(fam.lower(), seed), alloc, seed if alloc == "random" else None)
            path = runs / f"run_{alloc}_{fam}_{seed}.json"
            path.write_text(json.dumps(runlog_to_json(log)))
            row = evaluate_row(alloc, fam, seed, expected, log)
            row["log_sha"] = log_sha(log)
            rows.append(row)
        report = build_report(rows, "dev", True, "test", EVAL_SEEDS)
        assert report["sample"] is True
        assert report["git_sha"] == "dev"
