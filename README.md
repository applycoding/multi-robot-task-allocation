# multi-robot-task-allocation

[![CI](https://github.com/applycoding/multi-robot-task-allocation/actions/workflows/ci.yml/badge.svg)](https://github.com/applycoding/multi-robot-task-allocation/actions/workflows/ci.yml)

Simulation-only fleet lab: assign coverage zones, and insert docking and charging, for a small robot fleet on a synthetic map. Greedy, Hungarian, and auction allocators are compared on locked scenarios. A GitHub Pages dashboard shows recorded CI output.

**This is a simulation demo. It is not connected to real robots, a warehouse manager, or any production fleet. Scenarios are synthetic. Independent travel is not multi-agent path finding; robots may occupy the same edge. Centroid coverage is not a swept footprint of a zone.**

The Pages site, once enabled after a green CI run, loads recorded results. It does not run the allocator or the simulator.

The allocator labelled Hungarian is exact enumeration for this 4-robot fleet, not a shipped Kuhn-Munkres implementation; tests check its cost-sum against a test-only Kuhn-Munkres oracle. Hungarian optimality applies only to one frozen epoch cost matrix, not to the closed-loop schedule. The charge wrapper is myopic. Sequential slot revalidation can drop a pair that was feasible when the epoch matrix was built. Auction is epsilon-optimal within OPT + 2.0 seconds on the epoch's real-real finite cost sum.

See `PLAN.md` for the locked design, `TASKS.md` for the build sequence, and `REFERENCES.md` for idea-only citations. Outbound license is Apache-2.0. Author: Arpan Bom.
CI installs Playwright Chromium with `npx playwright install chromium` and omits `--with-deps`. Installing OS packages through apt failed on ubuntu-24.04 with temporary DNS errors resolving azure.archive.ubuntu.com. Tests still run on the libraries already present in the runner image.
