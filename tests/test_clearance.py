# SPDX-License-Identifier: Apache-2.0
from multi_robot_task_allocation.world import OBSTACLES, segment_aabb_clearance, segment_legal


def test_clearance_far_legal() -> None:
    ax, ay, bx, by = 2.0, 2.0, 6.0, 2.0
    ox0, ox1, oy0, oy1 = OBSTACLES[0]
    d = segment_aabb_clearance(ax, ay, bx, by, ox0, ox1, oy0, oy1)
    assert d >= 0.25
    assert segment_legal(ax, ay, bx, by)


def test_clearance_clips_blocked() -> None:
    ax, ay, bx, by = 8.0, 3.0, 8.0, 13.0
    assert not segment_legal(ax, ay, bx, by)


def test_clearance_graze_legal() -> None:
    ax, ay, bx, by = 2.0, 2.5, 6.75, 2.5
    ox0, ox1, oy0, oy1 = OBSTACLES[0]
    d = segment_aabb_clearance(ax, ay, bx, by, ox0, ox1, oy0, oy1)
    assert d + 1e-9 >= 0.25
    assert segment_legal(ax, ay, bx, by, 0.25)
