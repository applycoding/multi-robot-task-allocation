#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
runs="${1:?runs dir}"
git_sha="${2:?git sha}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 PYTHONHASHSEED=0
export RUNS_DIR="$runs"
PYTHONPATH=src python3 - <<'PY'
import json
import os
from pathlib import Path
from multi_robot_task_allocation.eval.gates import MANIFEST, evaluate_row
from multi_robot_task_allocation.eval.generate import generate
from multi_robot_task_allocation.sim import runlog_to_json, simulate

runs = Path(os.environ["RUNS_DIR"])
for alloc, fam, seed, _expected in MANIFEST:
    mutant = seed if alloc == "random" else None
    sc = generate(fam.lower(), seed)
    log = simulate(sc, alloc, mutant)
    path = runs / f"run_{alloc}_{fam}_{seed}.json"
    path.write_text(json.dumps(runlog_to_json(log), indent=2))
print("wrote", len(list(runs.glob("*.json"))), "logs")
PY
PYTHONPATH=src python3 -m multi_robot_task_allocation.cli report \
  --runs "$runs" --git-sha "$git_sha" --sample false --out report.json \
  --check --require-ci --expect-sha "$git_sha"
