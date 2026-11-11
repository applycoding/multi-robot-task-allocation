# SPDX-License-Identifier: Apache-2.0
"""Report writer and validator."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

from multi_robot_task_allocation.eval.gates import MANIFEST, evaluate_row
from multi_robot_task_allocation.sim import RunLog, runlog_to_json

GATE_VERSION = "mrta-gates-v1"
HONESTY = (
    "Simulation only: synthetic fleet scenarios; independent travel is not multi-agent path finding."
)

REPORT_SCHEMA = 1


def log_sha(log: RunLog) -> str:
    payload = json.dumps(runlog_to_json(log), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode()).hexdigest()


def reject_non_finite(obj: Any) -> None:
    if isinstance(obj, float):
        if not math.isfinite(obj):
            raise ValueError("non-finite value in report")
    elif isinstance(obj, dict):
        for v in obj.values():
            reject_non_finite(v)
    elif isinstance(obj, list):
        for v in obj:
            reject_non_finite(v)


def build_report(
    rows: list[dict[str, Any]],
    git_sha: str,
    sample: bool,
    run_id: str,
    seeds: dict[str, list[int]],
) -> dict[str, Any]:
    reject_non_finite(rows)
    return {
        "schema_version": REPORT_SCHEMA,
        "sample": sample,
        "git_sha": git_sha,
        "run_id": run_id,
        "gate_version": GATE_VERSION,
        "honesty": HONESTY,
        "seeds": seeds,
        "rows": rows,
    }


def load_runlog(path: Path) -> RunLog:
    data = json.loads(path.read_text())
    log = RunLog()
    for k, v in data.items():
        if k == "zones_done":
            log.zones_done = set(v)
        elif hasattr(log, k):
            setattr(log, k, v)
    log.allocator = data.get("allocator", "")
    log.scenario_family = data.get("scenario_family", "")
    return log


def validate_manifest(rows: list[dict[str, Any]]) -> None:
    if len(rows) != 55:
        raise ValueError(f"expected 55 rows, got {len(rows)}")
    seen = {(r["allocator"], r["family"], r["seed"]) for r in rows}
    expected = {(a, f, s) for a, f, s, _ in MANIFEST}
    if seen != expected:
        missing = expected - seen
        extra = seen - expected
        raise ValueError(f"manifest mismatch missing={missing} extra={extra}")


def check_report(
    report: dict[str, Any],
    logs_dir: Path,
    *,
    require_ci: bool = False,
    expect_sha: str | None = None,
) -> None:
    if report.get("git_sha") in (None, "", "unknown"):
        raise ValueError("git_sha must not be unknown")
    if require_ci and report.get("git_sha") == "dev":
        raise ValueError("dev rejected in CI")
    if expect_sha and report["git_sha"] != expect_sha:
        raise ValueError(f"git_sha {report['git_sha']} != {expect_sha}")
    reject_non_finite(report)
    validate_manifest(report["rows"])
    for row in report["rows"]:
        key = (row["allocator"], row["family"], row["seed"])
        pattern = f"{row['allocator']}_{row['family']}_{row['seed']}.json"
        list(logs_dir.glob(f"*{pattern}*")) + list(logs_dir.glob(pattern))
        log_path = logs_dir / f"run_{row['allocator']}_{row['family']}_{row['seed']}.json"
        if not log_path.exists():
            log_path = logs_dir / f"{row['allocator']}_{row['family']}_{row['seed']}.json"
        if not log_path.exists():
            continue
        log = load_runlog(log_path)
        sha = log_sha(log)
        if row.get("log_sha") and row["log_sha"] != sha:
            raise ValueError("log_sha mismatch")
        expected = next(e for a, f, s, e in MANIFEST if (a, f, s) == key)
        recomputed = evaluate_row(row["allocator"], row["family"], row["seed"], expected, log)
        if row["expected"] != expected:
            raise ValueError("expected from manifest mismatch")
        if not recomputed.get("pass"):
            raise ValueError(f"row failed gate: {key}")
