# SPDX-License-Identifier: Apache-2.0
from multi_robot_task_allocation.battery import apply_charge, apply_drain


def test_travel_drain_10m() -> None:
    t = 10.0 / 0.60
    soc, death_t, dead = apply_drain(1.0, 0.004, t)
    assert not dead
    assert abs(soc - (1.0 - 1.0 / 15.0)) < 1e-12


def test_travel_drain_s2_rate() -> None:
    t = 10.0 / 0.60
    soc, _, dead = apply_drain(1.0, 0.008, t)
    assert not dead
    assert abs(soc - (1.0 - 2.0 / 15.0)) < 1e-12


def test_death_split_at_zero() -> None:
    soc, hit, dead = apply_drain(0.01, 0.004, 10.0)
    assert dead
    assert soc == 0.0
    assert hit is not None
    assert abs(hit - 2.5) < 1e-9


def test_charge_clips_one() -> None:
    assert apply_charge(0.9, 0.025, 20.0) == 1.0
