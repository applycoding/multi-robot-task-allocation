# multi-robot-task-allocation - v1 plan

Revised 2026-11-11 after independent review, independent version, URL, license, geometry, and math checks, and a recorded Python / NumPy prototype on the locked eval and dev seeds (see Decision I). Outbound license is **Apache-2.0**. Planning only; this document does not create a public repo or implementation.

`gh repo list applycoding --limit 200` on 2026-11-11 shows no `applycoding/multi-robot-task-allocation`.

## Goal

Public portfolio demo for Arpan Bom (idea #15): a CPU-only **simulation** lab that assigns coverage zones to a small robot fleet and inserts docking and charging as capacity-limited service chains.

Three first-principles allocators (greedy, Hungarian, auction) are compared on locked synthetic scenarios. Two mutants (random allocation and a never-charging policy) must fail their gates while still producing finite valid logs. A GitHub Pages dashboard at base `/multi-robot-task-allocation/` renders **recorded CI output**. The page does not allocate and does not simulate.

This is an original teaching implementation, inspired by published assignment, auction, and multi-robot task-allocation ideas. It is not a production warehouse manager, not a live AMR fleet, not certified multi-robot coordination, and not multi-agent path finding. Robots travel independently on a static graph and may occupy the same edge. One-shot Hungarian is optimal only for a single bipartite assignment epoch. Myopic re-solves, charge and dock insertions, and sequential slot revalidation are heuristics. Do not claim a globally optimal time-extended schedule, and do not claim "the official Hungarian / Bertsekas code."

Git author and committer: **Arpan Bom <applycoding@gmail.com>**. No co-author trailers. No credit to any assistant or tool in the README, NOTICE, or commits.

## Scope

In v1:

- One package `multi_robot_task_allocation`, console script `mrta`, Python and NumPy only in the core.
- A 20 m x 16 m synthetic bay, 4 robots, 8 coverage zones, static-graph Dijkstra, linear battery, discrete-event simulation.
- Greedy, exact enumeration labelled Hungarian, and a Bertsekas-style auction, plus `random` and `never_charge` mutants.
- Schema-v1 JSON report, Vite + Canvas dashboard, Playwright smoke, one CI workflow.

Do not depend on, submodule, vendor, or copy these.

- **applycoding/multi-robot-warehouse-planner**: CBS multi-agent path finding, prioritized and HOLD fallbacks, a live FastAPI + WebSocket canvas, and greedy FCFS plus SciPy `linear_sum_assignment` for pick-and-deliver warehouse orders, plus a battery model. This repo does not implement CBS, space-time A*, prioritized planning, or HOLD. It is not a live WebSocket UI. It does not copy that allocator, map, or battery code. The problem here is coverage-zone assignment with dock and charger slot scheduling on a static travel graph, a recorded-results dashboard, and first-principles greedy / Hungarian / auction (no SciPy).
- **applycoding/robot-fleet-mqtt-telemetry**: fleet telemetry over MQTT into TimescaleDB, alerting, and Grafana. This repo has no MQTT, TimescaleDB, Grafana, ROS, or heartbeat alerting.
- applycoding/nav2-coverage-planner-compare - single-robot geometric coverage bake-off. This repo assigns zones to a fleet and must not import `ncp`.
- applycoding/go-ride-dispatch, react-ts-fleet-dashboard - different domains or monitoring UIs.
- applycoding/nav2-behavior-trees, gazebo-robot-ci, ros2-nav2-slam-sim, ros2-voice-command-robot, teleop-console-demo, flutter-robot-operator, compose-android-robot-companion, teb-mpc-local-planner, nav-imitation-learning, ekf-gnss-localization, sim-gimbal-stabilizer, rosbag-eval-bench, field-telemetry-analyzer - other domains.

OR-Tools, SciPy, NetworkX, Nav2, and CBS libraries are citations only. Do not vendor or translate their sources.

Centroid coverage is an abstract service proxy, not a swept footprint of the zone rectangle. Charger and dock capacity is logical slot capacity, not physical collision avoidance.

## Architecture

### Decision A - NumPy only, no SciPy / OR-Tools / ROS in v1

One installable package `multi_robot_task_allocation`, console script `mrta`, build backend **hatchling 1.33.0**. Allocators, Dijkstra, the simulator, and tests use the standard library plus NumPy. Core CI installs no SciPy, OR-Tools, NetworkX, ROS, Gazebo, PyTorch, or Nav2.

Why: CI stays CPU-light and license-simple, and the demo does not retread the warehouse planner's SciPy assignment helper or its MAPF stack.

### Decision B - independent travel on a static graph, not MAPF

Frame: planar `map`, +x East, +y North (REP-103-like ENU). World is the closed rectangle `[0, 20] x [0, 16]` metres. `robot_radius_m = 0.25`. `v_cruise = 0.60` m/s, constant. No acceleration model in v1. A robot that starts moving along a path of length `L` arrives after `L / v_cruise`.

A pose outside `[0.25, 19.75] x [0.25, 15.75]`, or with distance to any obstacle rectangle strictly less than `robot_radius_m`, is illegal. Interior obstacles, metres:

- A: `[7.0, 9.0] x [0.0, 6.5]`
- B: `[7.0, 9.0] x [9.5, 16.0]`
- C: `[12.5, 14.5] x [5.0, 11.0]`

The gap `y = 6.5 .. 9.5` at `x = 7 .. 9` is an **east-west** corridor between A and B, not a north-south corridor through the wall.

Sites are zone centroids, dock poses, charger poses, the locked waypoints, and start poses. Nodes are **deduplicated by coordinates** at 1e-9 m: one geometric node may have several semantic aliases (a start may coincide with a centroid or waypoint). There is no zero-length edge.

Candidate edges are undirected and exist between every pair of distinct nodes whose segment stays in the world and whose segment-AABB clearance to every obstacle is `>= robot_radius_m`. Equality is legal (grazing). Length is Euclidean distance.

Clearance is the minimum distance from the closed segment to the closed axis-aligned rectangle: zero if they intersect, otherwise the min of endpoint-to-AABB distances and segment-to-box-edge distances.

Travel time is `path_length / v_cruise` from a **precomputed** all-pairs Dijkstra. Tie-break, locked: on a strictly shorter distance, update the parent; on equal distance (absolute difference `<= 1e-15`), keep the parent with the smaller node id; if the parent id is unchanged, keep the earlier parent. Never replace the source parent (`parent[src] = -1`). Reconstruct paths by walking parents; a unit test must reject cycles. If two sites are disconnected, travel is `+inf`.

Inter-robot collisions are ignored. Capacity conflicts exist only at docks and chargers.

The core never reads a wall clock. Time is an argument. The only RNG is `numpy.random.Generator(numpy.random.PCG64(seed))`, and only for scenario generation and the random mutant. The scenario seed and the mutant seed are distinct fields.

### Decision C - coverage, docks, chargers, battery

Default fleet: **4 robots**. Default: 8 coverage zones, 2 chargers. S1, S2, and S4 have 2 docks that are present but unused (`dock_after_zone = false`). S3 has **one** dock (D0 only).

Coverage. Each zone is an axis-aligned rectangle with a centroid work pose. `work_s = max(work_min_s, kappa * area_m2)` with `work_min_s = 10.0` s and `kappa = 2.0` s/m^2. Area is width times height, computed from the rectangle, not typed independently. A zone completes when one robot has spent `work_s` at its pose in one stay. No preemption. Default scenarios require each zone once inside `T_horizon`.

Locked default rectangles (S1, S2, S3, and the S4 geometry). **Z2 was corrected:** `[4, 6] x [6, 9]` has area `2 * 3 = 6`, so `work_s = 12`, not area 9 / 18 s.

| id | rectangle (m) | centroid | area m^2 | work_s |
| --- | --- | --- | --- | --- |
| Z0 | [1, 3] x [1, 4] | (2.0, 2.5) | 6 | 12 |
| Z1 | [1, 3] x [12, 15] | (2.0, 13.5) | 6 | 12 |
| Z2 | [4, 6] x [6, 9] | (5.0, 7.5) | 6 | 12 |
| Z3 | [10, 12] x [1, 4] | (11.0, 2.5) | 6 | 12 |
| Z4 | [10, 12] x [12, 15] | (11.0, 13.5) | 6 | 12 |
| Z5 | [15, 18] x [1, 3] | (16.5, 2.0) | 6 | 12 |
| Z6 | [15, 18] x [13, 15] | (16.5, 14.0) | 6 | 12 |
| Z7 | [16, 19] x [6, 9] | (17.5, 7.5) | 9 | 18 |

Docks. D0 `(4.0, 1.0)`, D1 `(4.0, 15.0)`, capacity 1 each when present. `dock_service_s = 8.0`. S3 uses only D0. A slot is reserved from the instant service starts until it completes, half-open `[start, end)`. Two robots never occupy the same slot (hard invariant). A robot that arrives while the slot is busy waits in a FIFO queue at the dock pose and is **not** in the slot. Queue time counts as dock wait and drains at `drain_idle`. Intervals that only touch at an endpoint do not conflict.

Chargers. C0 `(18.0, 4.0)`, C1 `(18.0, 12.0)`, capacity 1 each. Same occupy and FIFO rules. Wait counts as charge wait.

Waypoints: `(8.0, 8.0)`, `(5.0, 2.0)`, `(5.0, 14.0)`, `(11.0, 8.0)`, `(16.0, 8.0)`, `(16.0, 4.0)`, `(16.0, 12.0)`.

Battery, SOC in `[0, 1]`, linear, locked:

- `drain_move = 0.004` per second while traveling (S2 overrides to `0.008`)
- `drain_work = 0.003` per second while covering
- `drain_idle = 0.0008` per second while waiting, queued, or docking
- `charge_rate = 0.025` per second while in a charger slot, clipped at 1.0 (net charge; no extra idle drain during the occupy interval)
- `min_reserve = 0.15`
- S1, S3, S4 start SOC `1.0`. S2 start SOC **`0.35`** (frozen after the prototype; the draft's 0.45 let a never-charging policy finish the mission).

SOC updates on every event from the exact interval length. If a linear drain would reach 0, split at the hitting time, store SOC `0`, and mark the robot `dead`. SOC below 0 is never stored. SOC exactly 0 is dead, unless the robot is already in a charger slot and charging. A robot that arrives at a charger with SOC 0 may start charging if the slot is free at that instant. A dead robot stays in place; if it died inside a slot, that slot stays blocked until `T_horizon`. If it died in a queue, it is removed from the queue and does not occupy the slot. Interpolate a death pose along the polyline by the fraction of the travel interval used. Cancel stale future completion events after death.

An inserted charge restores SOC to 1.0. Duration after queue wait is `(1 - soc_at_service_start) / charge_rate`.

The **death-in-slot** unit fixture must inject a terminal failure (for example a forced drain while occupying); normal charging cannot empty the battery.

### Decision D - feasibility wrapper and slot fitting

Shared by greedy, Hungarian, and auction. Not used by `never_charge`. Used by `random` only on the legal-pair branch.

A candidate `(robot, task)` is legal only if a residual plan exists. The check is **myopic**: other robots do not move; only already committed slot intervals are occupied.

Residual for coverage: travel to the zone, work `work_s`, then if `dock_after_zone` travel to a dock and take `dock_service_s`, then **witness** that some charger is reachable with SOC `>= min_reserve` at the charger pose, and that a later full charge can fit without dying in the queue. The final charger leg is a reachability witness. It is **not** committed and not executed as part of the coverage assignment.

If the residual is infeasible at the current SOC, insert the earliest charge stop before the coverage (from the ready pose): among chargers that restore SOC to 1.0 and then admit the residual, pick the one with smallest added delay. Tie-break: lower charger id. If still infeasible, the pair is illegal (`cost = +inf`).

**Slot fitting, locked.** `next_fit(site, t_arr, duration)` is the earliest `t >= t_arr` such that half-open `[t, t+duration)` does not overlap a committed interval. It is not "the first instant the slot is idle," because a later reservation can still overlap a long service that starts in a gap. Charge duration depends on SOC after idle drain during the wait, so the service start is found by iterating: propose `t`, compute SOC after waiting `t - t_arr`, compute `duration`, and if `next_fit` is later, advance `t`. Cap 200 iterations; failure is infeasible, not a crash. Book the slot only after a gap is found. Simultaneous arrivals at equal `t` break ties by lower robot id.

Cost, locked: `finish_time_of_task` (seconds) at the end of work, or at the end of the required dock when `dock_after_zone` is true, including any inserted charge and travel. The witness trip to a charger is not in the cost. Tie-break when comparing pairs: lower travel distance of the executed residual (charge + travel + work + dock), then lower robot id, then lower task id. Do not use a floating perturbation of the cost matrix to encode this order (it cannot distinguish the two permutations of an all-equal 2x2 matrix, and it can reverse a tiny genuine gap).

The wrapper does not produce a globally optimal charge schedule.

### Decision E - allocators and epoch commit

All three are original implementations, inspired by the classical methods. They are not ports of SciPy, OR-Tools, or Bertsekas reference code.

**Greedy.** Repeatedly select the feasible pair of lowest cost (then travel, robot id, task id), commit it, **rebuild** the cost matrix against the updated calendar, and continue until no legal pair remains.

**Hungarian.** Maximum finite cardinality first, then minimum unperturbed cost, then the pair-order tie-break above. Rectangular matrices are allowed. Forbidden pairs stay forbidden; do not encode them as `1e18` (float64 spacing at 1e18 can erase subsecond costs). Implement Kuhn-Munkres with an explicit forbidden-edge rule, or, equivalently for this 4-robot fleet, an exhaustive enumeration of assignments that is tested to be identical to a correct KM on every unit matrix. A brute-force oracle on matrices up to 4x8, including empty, all-forbidden, Hall-deficient, tall, wide, and mixed-`inf` cases, is a ship test. Dummy padding, if used, must not change the cardinality-then-cost objective. Do not claim optimality of the closed-loop schedule.

**Auction.** Bertsekas forward auction on **benefit minus price**. Finite benefit `b_ij = C_big - cost_ij` with `C_big = max(finite costs) + 1`. Forbidden pairs are not biddable. Always include **4 dummy objects** with benefit **0** (idle). Dummy assignment means the robot is unassigned this epoch. There are no dummy robot bidders. Prices start at 0. Serial bidder order: increasing robot id among currently unassigned robots. Each bid raises the object's price by `(best_value - second_best_value) + epsilon`, with `epsilon = 0.5` (benefit units, which are seconds). If only one biddable object exists, `second_best_value = 0` (the dummy). The previous owner becomes unassigned. Round cap `20000`. Hitting the cap raises `AuctionRoundCap`, which is an infrastructure failure, not a successful schedule and not a mutant pass. No silent fallback to Hungarian.

With 4 bidding robots, the epsilon-optimality bound used in honesty text and tests is: the sum of **real-real** finite assignment costs is at most `OPT + 4 * 0.5 = OPT + 2.0` seconds, where `OPT` is the Hungarian unperturbed finite cost sum at the same cardinality. This is the padded-bidder bound, not a bit-identical assignment claim. Cite Bertsekas, "The Auction Algorithm: A Distributed Relaxation Method for the Assignment Problem," https://web.mit.edu/dimitrib/www/Orig_Auction.pdf (HTTP 200, application/pdf, 1443637 bytes on 2026-11-11).

**Epoch policy.** At `t = 0`, and whenever a robot becomes free or a slot frees while a robot is waiting, or a `soc_watch` event fires, process every event at that timestamp first, then run one assignment epoch.

- Pending coverage = zones not yet assigned and not yet complete.
- Available robots = robots that are free **now**. A robot already executing a committed action chain (including an inserted charge followed by its zone and optional dock) is not reassigned and does not become free in the middle of that chain. No preemption.
- Build the cost matrix for currently free robots against pending zones.

**Transactional commit, locked.** Greedy already commits one pair and rebuilds. Hungarian and auction first produce a candidate matching on the frozen matrix (epoch optimality claims apply only to that matrix). Then commit in locked order `(finish, travel, robot id, task id)`, **re-running the wrapper against the updated calendar** for each pair. If a pair is no longer legal, skip it (the robot stays free, the task stays pending). Then continue the greedy leftover pass on any still-free robots. Test two pairs that are feasible separately and jointly infeasible (same charger gap).

Idle: if no legal coverage remains for a free robot, it stays idle, unless it cannot reach any charger with SOC `>= min_reserve` on arrival (including committed occupancy and queue wait). In that case the epoch assigns a charge. Additionally, schedule a `soc_watch` at the future time when idle drain would make every charger unreachable at `min_reserve`. If that time is already now, charge immediately. The never-charge mutant skips charge requests and `soc_watch` charges.

### Decision F - simulator and metrics

Discrete-event. Event kinds: `battery_empty`, `arrive`, `start_work`, `finish_work`, `start_charge`, `finish_charge`, `start_dock`, `finish_dock`, `soc_watch`, `horizon`. Equal-timestamp order is that list, then lower robot id, then a monotonic insertion counter. Process a whole timestamp batch before an assignment epoch.

The engine does not step at 1 Hz. A 1 Hz sampled trace is derived after the run for the dashboard and is labelled as sampled. Slot overlap and service contacts are tested on the event log, using half-open occupy intervals and, for the Gantt, exact action intervals from the log (not only the 1 Hz samples).

The run ends at the earliest of: all required zones complete and no robot mid-chain, irrecoverable deadlock (pending zones, no legal pair, no scheduled `soc_watch` or slot-free event that can change that), or `T_horizon`. Report the reason. An unfinished run must not receive a successful makespan; record `makespan_s = T_horizon` only as a cap, plus `complete = false`. `finish > T_horizon` is not by itself a timeout detector; use the completion flag. Coverage is the fraction of required zones completed exactly once.

### Decision G - scenario families, frozen after the prototype

Generator `generate(family, seed)` is pure. Same seed and family produce a bit-identical world. Canonical encoding: UTF-8 JSON, sorted keys, separators comma-colon with no spaces, as in json.dumps(..., separators=(',', ':')), floats rounded to 1e-9 before hashing, including obstacles, zones, docks actually used, chargers, starts, start SOC, every drain and charge rate, `min_reserve`, `v_cruise`, `robot_radius_m`, `dock_after_zone`, `dock_service_s`, `T_horizon`, waypoints, and a schema / graph-policy version string `mrta-world-v1`. `sha256` of that encoding is the world id.

The geometry above is fixed. The seed does not move walls. `PCG64(seed)` only shuffles the start pool and, on S4 only, which three zone ids are long. Start pool, in this order before the shuffle: `(2.0, 8.0)`, `(11.0, 8.0)`, `(16.0, 8.0)`, `(2.0, 2.5)`, `(2.0, 13.5)`, `(16.5, 2.0)`. Take the first 4 distinct poses.

**S4 is a workload-imbalance family, not a resize.** Rectangles stay at the default table. After shuffling the 8 zone ids, the first 3 get `work_s = 40.0`; the others keep the table formula. This avoids the draft's area-16 resize, which cannot fit Z0 / Z1 / Z5 / Z6 inside the inset.

Families, **frozen**:

- **S1 balanced.** SOC 1.0, `drain_move = 0.004`, no required dock, 2 unused docks. `T_horizon = 150` s.
- **S2 charge-tight.** SOC 0.35, `drain_move = 0.008`, no required dock. `T_horizon = 200` s.
- **S3 dock-contention.** SOC 1.0, `drain_move = 0.004`, `dock_after_zone = true`, **1 dock** (D0). `T_horizon = 300` s.
- **S4 imbalanced.** SOC 1.0, three long zones at 40 s. `T_horizon = 200` s.

Horizons were set from the prototype's worst non-mutant makespan plus a margin, then checked so the random mutant misses the cap on every eval seed. They are not loosened to make CI green, and eval seeds are not edited to chase a metric.

Dev seeds (prototype only, not a ship gate, not shown on Pages): `7`, `11`, `42`.

Eval seeds (ship gates):

- S1: `20261111`, `20261112`, `20261113`
- S2: `20261121`, `20261122`, `20261123`
- S3: `20261131`, `20261132`, `20261133`
- S4: `20261141`, `20261142`, `20261143`, `20261144`

If a seed's world leaves a start unable to reach a required zone, dock, or charger, the generator test fails and the seed is not shipped. Do not replace an eval seed.

World sha256 recorded for S1 seed `20261111` by the prototype's canonical encoder (may differ if the checked-in encoder adds fields; CI then freezes the committed fixture hash): `d0d6e1a02f2f0538b037e66f797bc2b5a347d4ce456e4d6193c93a24dff3108e`.

### Decision H - mutants

`random`. At each epoch, if the legal-pair set is non-empty, sample one pair uniformly and commit it; else sample one uniform `(free robot, pending task)` **without** the wrapper (raw travel and work; the robot may die). Disconnected travel is a rejected attempt, not an infinite arrival. The run must be finite: no crash, no NaN. Under the frozen horizons it must fail the mission on every eval seed of S1, S2, S3, and S4 (coverage `< 1.0`, or deaths `>= 1`, or `complete = false`). That failure was measured (Decision I), not obtained by editing seeds.

`never_charge`. Greedy selection, but the wrapper never inserts a charge, the residual charger witness is ignored, `soc_watch` charges are skipped, and the allocator keeps assigning any task the robot can start (travel + work, plus a required dock on S3) until it dies or no startable task remains. On every S2 eval seed, deaths `>= 1` and coverage `< 1.0` and `complete = false`. Deaths are valid terminal states. This mutant is omitted from S1, S3, and S4 ship rows (it can finish those families on a full battery).

Timeouts, crashes, and NaNs are infrastructure failures. They are never successes and never expected mutant passes.

### Decision I - ship record, 2026-11-11

A Python / NumPy prototype of the generator, graph, battery, wrapper, `next_fit`, greedy, a brute-force cardinality-then-cost assignment (labelled Hungarian here), the auction, both mutants, and the event-like commit loop was run on every eval and dev seed. Unit checks for the 10 m SOC fraction, the 2x3 hand matrix (sum 6, pairs `(0, 2)` and `(1, 1)`), the auction bound on that matrix, and the Dijkstra parent hand case all passed. Wall time for the full suite was 0.2 s on a local CPU (Python 3.13, numpy 2.5.4).

Existence witnesses: greedy, Hungarian, and auction each produced a complete zero-death, zero-conflict schedule on every eval seed, with makespan `<=` the frozen horizon. Those measured complete runs are the constructive feasibility witnesses. A serial one-robot witness is not required to beat the horizon.

The figures in the original Decision I draft are discarded prototype output and are not the ship record: S1 auction travel of 42.5 m vs greedy 52.4 m (and 41.7 m vs 51.5 m); S1 random cap of 151 s; S2 makespans greedy 154.04, 153.40, 154.04, Hungarian 154.04, 153.40, 154.51, auction 152.53, 151.94, 156.15; S2 charge wait "about 77-108 s"; S2 `never_charge` coverage fractions 0.875, 0.75, 0.875; S2 random makespans 272.20, 231.07, and a 201 s cap; S3 makespans 181.95, 183.78, 175.78 s; S4 seed `20261142` listed as 92.8; S4 seed `20261143` listed as greedy 102.1, Hungarian 92.0, auction 102.1. The ship numbers below are taken from the 55-row `report.json` (`sample: false`) at the shipped revision.

Eval results used as ship numbers:

**S1**, horizon 150 s. All three non-mutants: coverage 1.0, deaths 0, conflicts 0, makespan 60.90, 60.90, 60.94 s (greedy, Hungarian, and auction tied on each seed). Travel is also tied: 52.39 m on seed `20261111`, 51.52 m on `20261112`, and 73.41 m on `20261113`. Auction does not travel less than greedy. Random failed all three seeds (coverage incomplete, makespan capped at the frozen horizon of 150.0 s).

**S2**, horizon 200 s, start SOC 0.35. Non-mutant makespans: 133.86, 167.48, 139.99 s, with greedy, Hungarian, and auction tied on each seed. Deaths 0, coverage 1.0, conflicts 0. Charge wait was real (fleet totals 96.62, 92.43, and 87.86 s). `never_charge` failed all three seeds with deaths 4, 4, 4 and coverage incomplete. Random failed all three (makespan capped at the frozen horizon of 200.0 s, deaths 4, coverage incomplete).

**S3**, horizon 300 s, one dock. All three non-mutants: coverage 1.0, deaths 0, conflicts 0, makespan 269.54, 267.71, 250.39 s (allocators tied on each seed). Fleet dock wait was strictly positive on every eval seed and every non-mutant (minimum 1.84 s, 7.90 s on six of the nine rows). Random failed all three (coverage incomplete, makespan capped at the frozen horizon of 300.0 s).

**S4**, horizon 200 s. Non-mutant coverage 1.0, deaths 0, conflicts 0. Makespans:

| seed | greedy | Hungarian | auction | Hungarian `<=` greedy | strict gap |
| --- | --- | --- | --- | --- | --- |
| 20261141 | 80.37 | 80.37 | 80.37 | yes | no |
| 20261142 | 85.47 | 85.47 | 85.47 | yes | no |
| 20261143 | 103.59 | 92.00 | 92.00 | yes | yes |
| 20261144 | 113.57 | 113.57 | 113.57 | yes | no |

**S4 quality gate, frozen honestly:** all three non-mutants complete with zero deaths and zero conflicts and makespan `<= 200`. Hungarian makespan is `<=` greedy on all four eval seeds. Do **not** require a strict gap on at least half the seeds (only one of four was strict, seed `20261143`). On that seed, auction ties Hungarian, not greedy. Report all twelve makespans.

Draft values that were rejected: S2 start SOC 0.45 (never-charge could finish); S4 area-16 resize (rectangles would leave the inset); horizons 360 / 540 / 720 / 480 (loose enough that random finished); S3 with two docks (dock wait was often 0).

### Decision J - reports and Pages

Schema version 1. The validator recomputes every gate from the **RunLog** event records, not from pre-cooked row fields, with non-finite values rejected (`allow_nan=False`, and also reject Infinity). Artifacts: one `RunLog` JSON per (allocator, family, seed) with the world sha, event list, occupy intervals, and per-robot summaries; one `report.json` that binds each row to that log's sha256 and the world sha.

Report fields: `schema_version`, `sample` bool, `git_sha`, `run_id`, `gate_version` (`mrta-gates-v1`), `honesty` (one sentence: simulation only, synthetic, not MAPF), `seeds`, `rows[]` with `allocator`, `family`, `seed`, `world_sha`, `log_sha`, `coverage_complete`, `deaths`, `makespan_s`, `complete`, `total_travel_m`, `dock_wait_s`, `charge_wait_s`, `slot_conflicts`, `pass` bool, `expected` enum (`pass` or `fail`), `observed` enum (`pass` or `fail`). Manifest, exact eval rows: 3 allocators x (3+3+3+4) = 39 non-mutant rows, 13 random rows (all eval seeds), 3 never-charge rows (S2 only). Total 55 rows. Missing or extra rows fail the report check. `expected` comes from this manifest, not from caller-controlled JSON. On CI, `git_sha` must equal `GITHUB_SHA`; `dev` is rejected there.

Traces: every non-mutant allocator on the first eval seed of each family, plus `random` and `never_charge` on S2 seed `20261121`. Draw the Gantt from exact action intervals; the 1 Hz pose sample is for the map only and is labelled sampled.

Vite + TypeScript + Canvas 2D. Base path `/multi-robot-task-allocation/`. Playwright, Chromium only. `docs/demo.png` is checked in once from a local Playwright capture at 1280x720. CI uploads the Playwright screenshot at `web/artifacts/demo.png` as the `playwright-shot` artifact and does not commit it. A sample report is committed with `sample: true`. The deployed CI report has `sample: false` and must not silently fall back to the sample on `main`. Missing or malformed report: visible error state, no sample banner.

HTML document title contains `simulation only`. Visible text, exact:

- `Simulation only`
- `Synthetic fleet scenarios; not a live allocator`
- `Recorded CI output; this page does not run the simulator`
- A sentence that independent travel is not multi-agent path finding and robots may occupy the same edge.

Do not claim a production WMS, real robots, or CBS/MAPF. Do not claim that a zone rectangle was swept.

### Decision K - CI

`runs-on: ubuntu-24.04`. Never `ubuntu-latest`. Jobs `core`, `web` (`needs: [core]`), and `pages` (`needs: [core, web]`). `timeout-minutes: 20` on `core`; `15` on `web`; `15` on `pages`. Coverage **80%** line floor, aggregate, on authored modules `world`, `battery`, `sim`, `alloc`, `eval` (omit `cli.py` and `web/`). `--cov-fail-under=80` must be able to fail the job. This is an aggregate floor, not a per-file floor. Include `battery.py`.

`core`: checkout SHA pin; setup-python 3.12 SHA pin; thread env vars set before NumPy import; `git config --global --add safe.directory "$GITHUB_WORKSPACE"` if `git rev-parse` is used; `pip install --require-hashes -r requirements.lock`; install the local package `--no-deps`; `ruff`; `mypy --strict`; `mrta generate` for every eval seed into a temp dir and diff hashes to committed fixtures; run all three allocators and both required mutants; write RunLogs and `report.json` with `git_sha` from `GITHUB_SHA` (never `unknown`, never `dev`); `mrta report --check --require-ci --expect-sha ${{ github.sha }}`; `pytest` including oracles, gates, and coverage fail-under 80; `author-check`; `license-bytes`; typographic-dash check on authored paths only (exclude lockfiles, `node_modules`, LICENSE official text, and generated coverage); upload artifacts `report` and `runlogs`. A missing upload fails the job.

`web`: checkout; setup-node 26.13.0 with npm cache; `npm ci`; `npx tsc --noEmit`; `npx vitest run`; download `report`; copy to `web/public/report.json` (fail if missing); `npx vite build --base /multi-robot-task-allocation/`; `npx playwright install chromium` from the pinned package (deviation: `--with-deps` is omitted because apt on ubuntu-24.04 failed with temporary DNS errors resolving azure.archive.ubuntu.com; Playwright still passes on the runner image); Playwright against the built site at that base path; upload screenshot artifact and `web-dist` (the tested `web/dist` that already contains `report.json`). Do not `git add` the screenshot.

`pages`: main push only, after core and web of the same SHA. Download the tested `web-dist`, publish it with `actions/upload-pages-artifact` and `actions/deploy-pages`. Do not rebuild. `github-pages` environment, permissions `pages: write` and `id-token: write`, and concurrency so a stale run cannot replace a newer one.

### Decision L - honesty

No real robot logs, no customer warehouses, no live fleet, nothing tied to patent-pending work, no private or unreleased product algorithms. No claim of ISO or safety certification. No claim that this is OR-Tools, Nav2, CBS, MAPF, or a warehouse management system.

## Pinned stack

Verified 2026-11-11 against PyPI, npm, nodejs.org, and the GitHub API (Actions tag objects). Python runtime on the runner: **3.12** (latest 3.12.x that `setup-python` resolves; do not float to 3.13). Python 3.12.16 exists as a source-only security release dated 2026-10-25 (https://www.python.org/downloads/release/python-31216/); v1 does not pin that patch in `python-version`.

Direct Python packages, equality pins, **filenames and sha256** of the linux x86_64 (or py3-none-any) wheels, from `https://pypi.org/pypi/<name>/<ver>/json` on 2026-11-11:

| Package | Version | Wheel | sha256 |
| --- | --- | --- | --- |
| numpy | 2.5.4 | `numpy-2.5.4-cp312-cp312-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl` | `fbde6962867ee75b48b0ee29b2b9372ec5d617799dbaf38e82dc0596f2f7738a` |
| pytest | 9.2.0 | `pytest-9.2.0-py3-none-any.whl` | `07acdcf88ec174f907dc01af70bf52d49b014bbab03faeeade31c96b18c2e588` |
| pytest-cov | 7.1.0 | `pytest_cov-7.1.0-py3-none-any.whl` | `a0461110b7865f9a271aa1b51e516c9a95de9d696734a2f71e3e78f46e1d4678` |
| coverage | 7.17.2 | `coverage-7.17.2-cp312-cp312-manylinux1_x86_64.manylinux_2_28_x86_64.manylinux_2_5_x86_64.whl` | `36314997533266bde3d07765289674304d61bfe49165bb1b4fa9be12cb4ba0d1` |
| ruff | 0.17.4 | `ruff-0.17.4-py3-none-manylinux_2_17_x86_64.manylinux2014_x86_64.whl` | `2cd025509b2e15721b426fc70fd946d71f43b3db3b08e946dfec2d696d911690` |
| mypy | 2.4.1 | `mypy-2.4.1-cp312-cp312-manylinux2014_x86_64.manylinux_2_17_x86_64.manylinux_2_28_x86_64.whl` | `a20ff6cd2af7a670c64f089bfa5fbcebe8ff3bf896c5e6ac7c9862c7960e4494` |
| hatchling | 1.33.0 | `hatchling-1.33.0-py3-none-any.whl` | `415290b84f464d2884303fca8974d38e7b85579069fab02a15db0828fbb1446d` |
| pip-tools | 7.6.2 | `pip_tools-7.6.2-py3-none-any.whl` | `f37eb4b736859c2c30230e50977312c61918b7ef931071ff7811fd067715c25f` |

Do **not** pass `pip-compile --python-version`. Run pinned `pip-tools==7.6.2` **under Python 3.12**: `pip-compile --generate-hashes -o requirements.lock requirements.in`. CI installs with `pip install --require-hashes -r requirements.lock`, then the local package with `--no-deps`. No scipy, networkx, or ortools.

Node Active LTS: **26.13.0** Lithium, 2026-10-28, bundled npm **11.20.0** (https://nodejs.org/dist/index.json and https://nodejs.org/en/blog/release/v26.13.0). npm packages, current latest on 2026-11-11, licenses checked on the registry:

| Package | Version | License |
| --- | --- | --- |
| vite | 8.3.8 | MIT |
| typescript | 7.0.2 | Apache-2.0 |
| vitest | 5.1.0 | MIT |
| @playwright/test | 1.65.0 | Apache-2.0 |
| @types/node | 26.6.6 | MIT |

Commit `package-lock.json` generated with npm 11.20.0. Playwright browsers: Chromium only, the build bundled with `@playwright/test@1.65.0`.

GitHub Actions, full commit SHAs (latest releases on 2026-11-11), never floating tags, verified via `gh api repos/actions/<name>/git/refs/tags/<tag>` (annotated tags peeled):

| Action | Tag | SHA |
| --- | --- | --- |
| actions/checkout | v7.0.1 | `3d3c42e5aac5ba805825da76410c181273ba90b1` |
| actions/setup-python | v7.1.0 | `6d4fd22b20465ccc0f7354476588e8df7c9ca0e0` |
| actions/setup-node | v7.1.0 | `949feb2413d6458794dcd2491c4babbbce0c15c1` |
| actions/upload-artifact | v7.0.2 | `cf430e030ddbb5b0abf93d22962f4752f3646cd9` |
| actions/download-artifact | v8.0.2 | `9000827ccba6bdab643e8b6fd33ac0654aef8333` |
| actions/configure-pages | v6.0.0 | `45bfe0192ca1faeb007ade9deae92b16b8254a0d` |
| actions/deploy-pages | v5.0.1 | `368f82528645a54fb793d4d04e342629a3f51346` |
| actions/upload-pages-artifact | v5.0.0 | `fc324d3547104276b827a68afc52ff2a11cc49c9` |

The official Apache-2.0 text was fetched on 2026-11-11 from https://www.apache.org/licenses/LICENSE-2.0.txt: 11358 bytes, sha256 `cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30`.

Thread env, set before NumPy import: `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`, `MKL_NUM_THREADS=1`, `NUMEXPR_NUM_THREADS=1`, `PYTHONHASHSEED=0`.

The runner's exact 3.12 patch and the Ubuntu image float; that boundary is accepted. Fixture hashes must still match.

## File layout

```text
pyproject.toml
requirements.in
requirements.lock
NOTICE
LICENSE
README.md
REFERENCES.md
.github/workflows/ci.yml
src/multi_robot_task_allocation/
  __init__.py
  cli.py
  world.py
  battery.py
  sim.py
  alloc/{greedy,hungarian,auction,wrapper,random_alloc,never_charge}.py
  eval/{generate,gates,report}.py
tests/
fixtures/                  # committed eval worlds and sha256 sidecars
web/                       # Vite + TS + Canvas; base /multi-robot-task-allocation/
docs/demo.png
web/public/sample_report.json
```

`SPDX-License-Identifier: Apache-2.0` on every authored Python and TypeScript file.

## Interfaces

- `generate(family: str, seed: int) -> Scenario`
- `travel_time(scenario, a: int, b: int) -> float` (`inf` if disconnected)
- `assign_epoch(allocator: str, cost, travel) -> list[tuple[int, int]]`
- `feasible_plan(robot, task, committed) -> Candidate | None` (structured plan plus cost; execution consumes this plan)
- `simulate(scenario, allocator: str, mutant_seed: int | None) -> RunLog`
- `evaluate(run: RunLog) -> Row` recomputes gates from the log

Allocator names: `greedy`, `hungarian`, `auction`, `random`, `never_charge`.

CLI:

- `mrta generate --family s1|s2|s3|s4 --seed INT --out scenario.json`
- `mrta run --scenario scenario.json --allocator NAME [--mutant-seed INT] --out run.json`
- `mrta report --runs DIR --git-sha SHA --sample true|false --out report.json`

`mrta run --mutant-seed` is only for `random`. Deterministic allocators ignore it except as a recorded field. Missing `--git-sha` is an error in CI. Local use may pass `dev`; CI rejects it. The report must not write `unknown`.

## Edge cases

- Disconnected sites: Dijkstra returns `inf`. The pair is illegal. A world where a start cannot reach a zone or a charger fails the generator test.
- All chargers occupied: `next_fit` may delay a charge; if idle drain would kill the robot before a gap, the pair is illegal. The robot is not teleported.
- Start SOC below `min_reserve`: the idle / `soc_watch` rule assigns a charge if the charge itself is feasible. The generator must not emit a start that cannot reach any charger before SOC 0 on eval seeds.
- Start pose on an obstacle or outside the inset: `generate` raises `InvalidScenario`.
- Zero-duration zone: prevented by `work_min_s = 10`. A unit test may build a hand scenario with `work_s = 0` and expect completion at arrival.
- Equal costs: tie-break is travel, then robot id, then task id, by lexicographic comparison, not a float perturbation.
- Robot dies inside a charger slot: the fixture injects a terminal failure; the slot stays occupied until `T_horizon`.
- Horizon cap: `complete` is false if any required zone is unfinished at `T_horizon`, even if the last event was at exactly `T_horizon`.
- More tasks than robots: one epoch assigns at most 4 zones; the rest stay pending.
- More robots than tasks: extra robots idle or charge under the idle rule.
- Two candidates legal alone, illegal together: sequential revalidation drops the later pair.
- NaN or Inf cost: rejected. A NaN fails the process. It is not a mutant pass.
- Two occupy intervals that only touch at an endpoint: not a conflict (half-open).
- Auction round cap: `AuctionRoundCap`, non-zero exit, not a row with `observed: fail`.
- Co-located start and centroid: one graph node, two aliases, no zero edge.

## Test plan

- `test_clearance`: segment far from A is legal; a segment that clips A is blocked; clearance exactly `0.25` is legal.
- `test_dijkstra`: the hand graph in Verification hints; parent of node 2 is 0; disconnected pair is `inf`; reconstructed paths have no cycles; alias / co-located nodes do not create a zero-edge loop.
- `test_hungarian_hand`: 2x3 matrix sum 6, pairs `(0, 2)` and `(1, 1)`; equal-cost 2x2 assigns `(0, 0)` and `(1, 1)` by the lex tie-break; mixed-`inf`, empty, all-forbidden, 1x3, 3x1, and 4x6 brute-force agreement (seed 0, including forbidden entries).
- `test_auction_bound`: on those matrices, real-real cost sum `<= hungarian_sum + 4 * 0.5`, feasible matching, dummy benefit 0 is not preferred over a finite real task when a real task remains and cardinality allows it. Do not require identical pairs. Include a second matrix with a distinct feasible sum inside the bound, for example `[[0, 0.2], [0.2, 0]]`.
- `test_battery_interval`: 10 m at 0.60 m/s consumes exactly `1/15` SOC at `drain_move = 0.004`, and `2/15` at `0.008`. A step that would go negative splits at SOC 0 and sets `dead`. Charge clips at 1.0. Queue wait is included before fill duration.
- `test_slots`: `next_fit` rejects a gap that is shorter than the service; two robots, one charger, overlapping requests, FIFO wait, zero overlap of occupy intervals. Injected death inside a slot blocks it.
- `test_wrapper`: a known feasible residual without a charge stays charge-free; a known tight residual (the 15 m / 12 s / 15 m S2 style budget in Verification hints, using actual graph distances) gets one inserted charge; a proved impossible residual is `inf`. Two jointly infeasible pairs: the second is rejected after the first commit.
- `test_fixture_hash`: family+seed world sha256 stable across two processes, and equal to the committed sidecar for every eval seed.
- `test_oracles`: every eval seed has a recorded complete zero-death witness with makespan `<=` the frozen horizon (the Decision I non-mutant runs, independently recomputed from the log).
- `test_gates`: Decision I integers and inequalities, including S3 dock wait `> 0` on every eval seed for all three non-mutants, and the frozen S4 Hungarian `<=` greedy rule.
- `test_mutants`: `random` fails every eval seed; `never_charge` on each S2 eval seed has deaths `>= 1` and coverage `< 1.0`, exit 0, no NaN. A forced exception in the simulator is a test failure, not a mutant pass.
- `test_report_schema`: 55-row manifest, log sha binding, `git_sha` equals the CLI value, sample report has `sample: true`, non-finite values rejected.
- `test_determinism`: two `simulate` calls, greedy / Hungarian / auction, identical canonical event-log sha256 on S1 seed `20261111`.
- Playwright: title contains `simulation only`; the three exact visible strings; the not-MAPF sentence; missing-report error state; no sample banner when `sample: false`.

## Numbered success criteria

1. `SC-01` `test_clearance`: blocked iff segment-AABB distance `< 0.25`.
2. `SC-02` `test_dijkstra`: hand equal-length case keeps parent 0 for node 2; a disconnected pair is `inf`; no parent cycles.
3. `SC-03` `test_hungarian_hand`: 2x3 sum is 6 with pairs `(R0, T2)` and `(R1, T1)`; brute-force agrees on the locked matrix suite.
4. `SC-04` `test_auction_bound`: real-real epoch sum `<= OPT + 2.0` on each Hungarian hand matrix; no forbidden pair selected.
5. `SC-05` `test_battery_interval`: 10 m at 0.60 m/s consumes exactly `1/15` SOC at `drain_move = 0.004`.
6. `SC-06` `test_slots`: `next_fit` honors duration; zero positive-length overlap on the event log, including the injected death-in-slot case.
7. `SC-07` `test_wrapper`: one known charge-insert case, one known charge-free case, one known `inf` case, and one joint-infeasibility case. An always-`inf` wrapper must fail this test.
8. `SC-08` `test_fixture_hash`: two processes, same family and seed, same world sha256, matching the committed sidecar.
9. `SC-09` `test_oracles`: every eval seed in S1-S4 has one complete zero-death schedule with makespan `<=` the frozen `T_horizon`.
10. `SC-10` `test_gates` S1: greedy, Hungarian, and auction on `20261111` through `20261113` have coverage 1.0, deaths 0, conflicts 0, complete, makespan `<= 150`.
11. `SC-11` `test_gates` S2: the same three allocators on `20261121` through `20261123` complete with deaths 0, conflicts 0, and makespan `<= 200`.
12. `SC-12` `test_mutants` never_charge: on each of those three S2 seeds, deaths `>= 1`, coverage `< 1.0`, exit code 0, no NaN.
13. `SC-13` `test_gates` S3: the three allocators on `20261131` through `20261133` complete, conflicts 0, makespan `<= 300`, and `dock_wait_s > 0`.
14. `SC-14` `test_gates` S4: the three allocators on `20261141` through `20261144` complete with makespan `<= 200`, and Hungarian makespan `<=` greedy on each seed.
15. `SC-15` `test_mutants` random: exit 0 and mission failure on every S1, S2, S3, and S4 eval seed.
16. `SC-16` `test_report_schema`: 55-row manifest; validator recomputes `pass` from the log; `git_sha` equals the CLI value.
17. `SC-17` `test_determinism`: canonical event-log sha256 matches across two runs for greedy, Hungarian, and auction on S1 seed `20261111`.
18. `SC-18` Playwright: the document title contains `simulation only`, and the three exact strings plus the not-MAPF sentence are visible. The page does not execute the Python package.
19. `SC-19` CI dash check: authored source, plan-like docs in the repo, README, NOTICE, and tests contain no U+2014, U+2013, or U+2212 (LICENSE official text and lockfiles excluded).
20. `SC-20` Coverage job: `--cov-fail-under=80` aggregate on `world`, `battery`, `sim`, `alloc`, `eval`. A documented smoke that drops the aggregate below 80 must fail this check (not left in the tree).

## Out of scope

MAPF, CBS, space-time A*, prioritized planning, HOLD, corridor reservations, acceleration limits, preemption, partial zone credit, multi-visit coverage, ROS, Gazebo, Nav2, MQTT, TimescaleDB, Grafana, BehaviorTree.CPP, a live WebSocket UI, SciPy, OR-Tools, NetworkX, swept-footprint coverage claims, and any claim of a global time-extended optimum.

## Documentation and honesty

README states: simulation only; synthetic scenarios; robots may occupy the same edge; not MAPF; not a WMS; not a live fleet; centroid coverage is not a swept zone; Hungarian optimality is per frozen epoch matrix only; the charge wrapper is myopic; sequential slot revalidation can drop a pair; auction is epsilon-optimal within `OPT + 2.0` seconds on the epoch's real-real cost sum.

References belong in REFERENCES.md (idea-only, licences confirmed). Confirm third-party notices for installed dependencies in NOTICE when pins land. Required third-party notices are not "credit to an assistant."

## License

Apache-2.0. `LICENSE` is the verbatim official text, 11358 bytes, sha256 `cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30`. `NOTICE` starts with `multi-robot-task-allocation, Copyright 2026 Arpan Bom`. `SPDX-License-Identifier: Apache-2.0` on every authored source file.

## Build and CI

One workflow, as in Decision K. SHA-pinned actions. `runs-on: ubuntu-24.04`. Do not raise the 20-minute core timeout without a plan amendment.

## Risks

- A later correct implementation may differ in subsecond makespans from the prototype; the ship gates use inequalities against the frozen horizons and the qualitative mutant / S3 / S4 rules, not a byte-identical copy of every prototype makespan except where a table is an explicit bound (`<=`).
- Dev seed 42 under some earlier S2 tunings let never-charge finish; the ship gate is the three eval seeds only, where deaths were 4/4/4.
- The myopic wrapper can still reject a pair that a different charge order would allow; `soc_watch` and sequential revalidation are the specified remedies. Do not edit eval seeds.
- Auction cap: hitting 20000 rounds is a red build.
- CI pin drift: lockfiles hold the resolution. Action SHAs were verified once.
- Dash characters in the README will fail `SC-19`. Write ASCII hyphen only.

## Verification hints

Travel time for 10 m: `t = 10 / 0.60 = 50/3` s.

SOC consumed at `drain_move = 0.004`: `0.004 * 10 / 0.60 = 1/15`. At S2 `0.008`, the same 10 m consumes `2/15`.

Illustrative residual, not a measured path: 15 m + 12 s work + 15 m at `drain_move = 0.008` consumes `0.20 + 0.036 + 0.20 = 0.436`. From start SOC 0.35 and `min_reserve` 0.15 the reserve budget is 0.20, so that residual is illegal without a charge. Actual assignments must use graph distances (Decision I already showed that start SOC 0.45 was not tight enough for never-charge to fail, which is why S2 is frozen at 0.35).

Charge: 20 s at `charge_rate = 0.025` adds 0.50 SOC, clipped at 1.0. Time to fill from SOC `s` after queue wait is `(1 - s) / 0.025` seconds.

Hungarian hand matrix (rows R0, R1; columns T0, T1, T2):

```text
5  9  2
8  4  7
```

Optimal finite sum is `2 + 4 = 6`, pairs `(R0, T2)` and `(R1, T1)`. Auction must land at a real-real sum `<= 6 + 2.0 = 8`. On this matrix the only feasible sums are 6 and 9, so a legal auction that assigns both robots must score 6. A second matrix such as `[[0, 0.2], [0.2, 0]]` has sums 0 and 0.4, both inside a `2.0` bound, and may differ from Hungarian.

Equal-cost 2x2, all entries 3: the lex tie-break assigns R0-T0 and R1-T1.

Dijkstra hand: nodes 0, 1, 2. Edges 0-1 length 3, 0-2 length 4, 1-2 length 1. Distance 0 to 2 is 4. Direct parent 0 versus parent 1; lower parent id wins, so the recorded parent of 2 is 0.

The frozen horizons (150, 200, 300, 200) and the Decision I integers are the ship numbers. Do not loosen them to make CI green.
