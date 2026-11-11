# SPDX-License-Identifier: Apache-2.0
"""Never-charge mutant: greedy without charge insertion."""
from __future__ import annotations

import numpy as np

from multi_robot_task_allocation.alloc.greedy import greedy_assign


def never_charge_assign(cost: np.ndarray, travel: np.ndarray) -> list[tuple[int, int]]:
    return greedy_assign(cost, travel)
