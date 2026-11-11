# SPDX-License-Identifier: Apache-2.0
"""Random mutant allocator."""
from __future__ import annotations

import math

import numpy as np


def random_pick(
    cost: np.ndarray,
    rng: np.random.Generator,
) -> tuple[int, int] | None:
    """Sample one legal pair if any, else raw pair ignoring wrapper."""
    nr, nc = cost.shape
    legal: list[tuple[int, int]] = []
    for r in range(nr):
        for t in range(nc):
            if math.isfinite(float(cost[r, t])):
                legal.append((r, t))
    if legal:
        idx = int(rng.integers(0, len(legal)))
        return legal[idx]
    if nr == 0 or nc == 0:
        return None
    return (int(rng.integers(0, nr)), int(rng.integers(0, nc)))
