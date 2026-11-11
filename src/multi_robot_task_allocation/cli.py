# SPDX-License-Identifier: Apache-2.0
"""Console entry point mrta."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from multi_robot_task_allocation.eval.gates import EVAL_SEEDS, MANIFEST, evaluate_row
from multi_robot_task_allocation.eval.generate import generate, scenario_to_json
from multi_robot_task_allocation.eval.report import build_report, check_report, log_sha
from multi_robot_task_allocation.sim import runlog_to_json, simulate
from multi_robot_task_allocation.world import world_sha256


def cmd_generate(args: argparse.Namespace) -> int:
    scenario = generate(args.family, args.seed)
    out = Path(args.out)
    payload = scenario_to_json(scenario)
    payload["world_sha256"] = world_sha256(scenario)
    out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    sidecar = out.with_suffix(out.suffix + ".sha256")
    sidecar.write_text(world_sha256(scenario) + "\n")
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    data = json.loads(Path(args.scenario).read_text())
    from multi_robot_task_allocation.eval.generate import generate

    scenario = generate(data["family"], data["seed"])
    mutant_seed = args.mutant_seed if args.allocator == "random" else None
    log = simulate(scenario, args.allocator, mutant_seed)
    Path(args.out).write_text(json.dumps(runlog_to_json(log), indent=2) + "\n")
    return 0


def cmd_report(args: argparse.Namespace) -> int:
    runs_dir = Path(args.runs)
    rows: list[dict[str, Any]] = []
    for alloc, fam, seed, expected in MANIFEST:
        log_path = runs_dir / f"run_{alloc}_{fam}_{seed}.json"
        if not log_path.exists():
            log_path = runs_dir / f"{alloc}_{fam}_{seed}.json"
        if not log_path.exists():
            print(f"missing log {log_path}", file=sys.stderr)
            return 1
        from multi_robot_task_allocation.eval.report import load_runlog

        log = load_runlog(log_path)
        row = evaluate_row(alloc, fam, seed, expected, log)
        row["log_sha"] = log_sha(log)
        row["world_sha"] = log.world_sha
        rows.append(row)

    report = build_report(
        rows,
        args.git_sha,
        args.sample == "true",
        run_id="ci",
        seeds={k: list(v) for k, v in EVAL_SEEDS.items()},
    )
    out = Path(args.out)
    out.write_text(json.dumps(report, indent=2) + "\n")

    if args.check:
        check_report(
            report,
            runs_dir,
            require_ci=args.require_ci,
            expect_sha=args.expect_sha,
        )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="mrta")
    sub = parser.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("generate")
    g.add_argument("--family", required=True)
    g.add_argument("--seed", type=int, required=True)
    g.add_argument("--out", required=True)
    g.set_defaults(func=cmd_generate)

    r = sub.add_parser("run")
    r.add_argument("--scenario", required=True)
    r.add_argument("--allocator", required=True)
    r.add_argument("--mutant-seed", type=int, default=None)
    r.add_argument("--out", required=True)
    r.set_defaults(func=cmd_run)

    rep = sub.add_parser("report")
    rep.add_argument("--runs", required=True)
    rep.add_argument("--git-sha", required=True)
    rep.add_argument("--sample", default="false")
    rep.add_argument("--out", required=True)
    rep.add_argument("--check", action="store_true")
    rep.add_argument("--require-ci", action="store_true")
    rep.add_argument("--expect-sha", default=None)
    rep.set_defaults(func=cmd_report)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
