# Build tasks - multi-robot-task-allocation

Execute in order. Follow PLAN.md. Deviate only if a pin or a test proves a step broken, and record why in DEVIATIONS.md. License is **Apache-2.0**. Commit author and committer are always **Arpan Bom <applycoding@gmail.com>**. No co-author trailers. No credit to any assistant or tool in the repository.

Each task has acceptance checks. A task is done only when those checks pass. Do not loosen a gate, edit eval seeds or horizons, or drop a mutant to make CI green.

## 1. Repository skeleton and license

- Create the tree in PLAN.md (empty dirs may hold `.gitkeep`).
- Fetch `https://www.apache.org/licenses/LICENSE-2.0.txt` into `LICENSE` and confirm sha256 `cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30` (11358 bytes).
- Write `NOTICE` starting with `multi-robot-task-allocation, Copyright 2026 Arpan Bom`, then dependency notices (filled as pins land).
- README: portfolio demo, simulation only, synthetic fleet scenarios, independent travel (not MAPF; robots may occupy the same edge), centroid coverage is not a swept footprint, not a WMS, not applycoding/multi-robot-warehouse-planner, Apache-2.0, author Arpan Bom.
- `pyproject.toml` with hatchling 1.33.0, package `multi_robot_task_allocation`, script `mrta`. SPDX on authored files.
- `.gitignore`: `__pycache__/`, `.venv/`, `node_modules/`, `dist/`, `artifacts/`, `.pytest_cache/`, coverage files, local Playwright output. Fixtures are committed.

**Accept:** `sha256sum LICENSE` matches the official digest. NOTICE and README name Apache-2.0 and Arpan Bom.

## 2. Lock Python and Node pins

- Write `requirements.in` with the direct pins in PLAN.md.
- Under Python 3.12, with `pip-tools==7.6.2`, run `pip-compile --generate-hashes -o requirements.lock requirements.in`. Do not pass a `--python-version` flag.
- Record hatchling and pip-tools wheel hashes in NOTICE once installed.
- `web/package.json` and `package-lock.json` with Node 26.13.0 / npm 11.20.0 and the npm pins in PLAN.md.

**Accept:** `pip install --require-hashes -r requirements.lock` succeeds on Python 3.12. The lock was produced by the pinned pip-tools.

## 3. World, graph, and clearance

- Implement segment-AABB clearance, pose legality, coordinate-deduplicated nodes, and all-pairs Dijkstra with the locked parent rule.
- No wall clock. Time is an argument.

**Accept:** Criteria 1 and 2. Every default site and start reaches every required zone, dock, and charger.

## 4. Battery and slot calendar

- Linear SOC updates, death split at 0, charge clip at 1.0.
- `next_fit(site, t_arr, duration)` on half-open intervals. Queue wait drains at `drain_idle` before fill duration is computed.

**Accept:** Criteria 5 and 6.

## 5. Generator and fixtures

- Exact RNG order, S3 single dock, S4 work_s = 40 on three shuffled zone ids, frozen horizons and start SOC values.
- Commit eval fixtures plus sha256 sidecars. Do not retune those seeds.

**Accept:** Criterion 8. S2 start SOC is 0.35. S3 has one dock.

## 6. Feasibility wrapper

- Myopic residual, witness charger reachability (not executed), optional inserted charge, structured `Candidate` plan.
- Sequential revalidation after each commit.

**Accept:** Criterion 7, including the joint-infeasibility case.

## 7. Allocators

- Greedy with rebuild after each commit.
- Hungarian: maximum finite cardinality, then min cost, then lex tie-break; tested against the brute-force oracle; no 1e18 sentinels.
- Auction: dummy objects with benefit 0, serial bids, epsilon 0.5, cap 20000, bound `OPT + 2.0`.
- `never_charge` and `random` as in PLAN.md Decision H.

**Accept:** Criteria 3, 4, 11, 12, and 15 (gates may be checked after eval lands).

## 8. Simulator, eval, report writer

- Discrete-event engine, locked equal-timestamp order, `soc_watch`, RunLog artifacts.
- `write_report` rejects non-finite values. `mrta report --check --require-ci` recomputes every gate from the logs and checks the 55-row manifest.

**Accept:** Criteria 9-17 except the dashboard.

## 9. Pages dashboard and Playwright

- Vite app, `base: '/multi-robot-task-allocation/'`, Canvas 2D, honesty labels, fetches via `import.meta.env.BASE_URL`.
- Copy CI `report.json` into `web/public/` before the production build. No silent sample fallback.
- Playwright as locked (Chromium with deps, `webServer` on the built site, 127.0.0.1, title contains `simulation only`).
- Check in `docs/demo.png` locally once. CI uploads the screenshot and does not commit it.

**Accept:** Criterion 18.

## 10. Single workflow, honesty, authorship

- One `ci.yml` as in PLAN.md Decision K: `core`, `web` needs `core`, `pages` needs both and deploys only a main push of that SHA from the tested `web-dist`. SHA-pinned actions, Python 3.12, Node 26.13.0, `ubuntu-24.04`. Never `ubuntu-latest`.
- Thread-limit environment variables set before NumPy is imported.
- Local package installed with `--no-deps` after hashed requirements.
- `author-check` and `license-bytes`. Typographic-dash check (U+2014, U+2013, U+2212) on authored paths. README honesty test. Pass `GITHUB_SHA` into the report so `git_sha` is never `unknown`.

**Accept:** Criteria 19 and 20. CI is green and the final run on the pushed branch has **completed**.
