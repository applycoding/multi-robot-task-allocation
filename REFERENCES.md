# References

Permissively licensed GitHub projects used as **references only**. Do not copy sources, tests, worlds, allocators, or documentation prose. Credit any conceptually derived idea in the README. Licenses were checked on 2026-11-11 with `gh api repos/OWNER/REPO --jq .license.spdx_id` and with the raw LICENSE file when the SPDX API was not enough.

**Outbound license for this demo repo is Apache-2.0**, not MIT. A LICENSE file with the official Apache-2.0 text, a NOTICE file crediting Arpan Bom, and SPDX-License-Identifier: Apache-2.0 headers are required at build time.

This demo must not duplicate existing applycoding repos. The closest are [applycoding/multi-robot-warehouse-planner](https://github.com/applycoding/multi-robot-warehouse-planner) (CBS MAPF plus a live WebSocket UI and SciPy assignment for pick-and-deliver orders) and [applycoding/robot-fleet-mqtt-telemetry](https://github.com/applycoding/robot-fleet-mqtt-telemetry) (MQTT / TimescaleDB telemetry). This repo is coverage-zone assignment with dock and charger slot scheduling, independent travel, first-principles greedy / Hungarian / auction, and a recorded-results dashboard. Do not copy those trees.

## Selected references (3)

### 1. google/or-tools

- URL: https://github.com/google/or-tools
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0; LICENSE: https://github.com/google/or-tools/blob/stable/LICENSE, HTTP 200 on 2026-11-11, text begins "Apache License" version 2.0)
- Borrow: the *idea* of assignment and routing as a taught fleet problem. v1 does **not** depend on or vendor OR-Tools.

### 2. scipy/scipy

- URL: https://github.com/scipy/scipy
- License: **BSD-3-Clause** (`gh api` SPDX: BSD-3-Clause; LICENSE: https://github.com/scipy/scipy/blob/main/LICENSE.txt, HTTP 200 on 2026-11-11)
- Borrow: the *idea* of linear sum assignment. This demo does not depend on SciPy. The allocator labelled Hungarian is exact enumeration of assignments, which is sufficient for this 4-robot fleet. Tests check its cardinality and finite cost-sum against a test-only Kuhn-Munkres oracle. The shipped allocator is not a Kuhn-Munkres implementation.

### 3. networkx/networkx

- URL: https://github.com/networkx/networkx
- License: **BSD-3-Clause**. The repository LICENSE.txt is the 3-clause BSD license (https://github.com/networkx/networkx/blob/main/LICENSE.txt, HTTP 200 on 2026-11-11, text begins "NetworkX is distributed with the 3-clause BSD license"). GitHub's license API reports `NOASSERTION` / `Other`; that API value is incomplete, not a different license. Treat the project as BSD-3-Clause and do not copy sources.
- Borrow: graph shortest-path *ideas* only. The Dijkstra in this demo is a new implementation with the parent rule locked in PLAN.md.

## Academic citations (not code)

- D. P. Bertsekas, "The Auction Algorithm: A Distributed Relaxation Method for the Assignment Problem," and the later survey. https://web.mit.edu/dimitrib/www/Orig_Auction.pdf (HTTP 200, application/pdf, 1443637 bytes on 2026-11-11) and https://web.mit.edu/dimitrib/www/Auction_Survey.pdf (HTTP 200 on 2026-11-11). Algorithm citation only. Do not port reference code.
- H. W. Kuhn, "The Hungarian Method for the Assignment Problem," Naval Research Logistics Quarterly, 1955, and J. Munkres, "Algorithms for the Assignment and Transportation Problems," Journal of the Society for Industrial and Applied Mathematics, 1957. Academic citations only.
- B. P. Gerkey and M. J. Mataric, "A Formal Analysis and Taxonomy of Task Allocation in Multi-Robot Systems," International Journal of Robotics Research, 2004. DOI 10.1177/0278364904045564. A commonly linked author PDF host (`cres.usc.edu`) returned HTTP 500 on 2026-11-11 and is not used as a pin. Citation only; do not copy prose or figures.

## Idea-only or exclude

- applycoding/multi-robot-warehouse-planner - differentiation target, not a source.
- applycoding/robot-fleet-mqtt-telemetry - differentiation target, not a source.
- applycoding/nav2-coverage-planner-compare - do not import or copy the `ncp` planner library.
- ros-navigation/navigation2 and CBS / MAPF libraries - mixed or incompatible licenses; not dependencies. Do not vendor or copy them.
- GPL or LGPL planning or allocation libraries - not dependencies.

Direct runtime and dev dependencies (numpy, pytest, ruff, mypy, Vite, TypeScript, Playwright, and the rest of the lockfiles) belong in NOTICE with version and license once installed. They are dependencies, not idea references.
