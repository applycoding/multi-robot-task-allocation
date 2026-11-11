# SPDX-License-Identifier: Apache-2.0
"""Linear battery SOC model."""
from __future__ import annotations

MIN_RESERVE = 0.15


def apply_drain(
    soc: float,
    drain_rate: float,
    duration_s: float,
    in_charge_slot: bool = False,
) -> tuple[float, float | None, bool]:
    """Return new_soc, death_time_or_none, is_dead after draining."""
    if duration_s <= 0.0:
        return soc, None, False
    if in_charge_slot:
        return soc, None, False
    end_soc = soc - drain_rate * duration_s
    if end_soc > 0.0:
        return end_soc, None, False
    if drain_rate <= 0.0:
        return max(soc, 0.0), None, soc <= 0.0
    hit = soc / drain_rate
    return 0.0, hit, True


def apply_charge(soc: float, charge_rate: float, duration_s: float) -> float:
    return min(1.0, soc + charge_rate * duration_s)


def charge_duration_to_full(soc_at_start: float, charge_rate: float) -> float:
    if soc_at_start >= 1.0:
        return 0.0
    return (1.0 - soc_at_start) / charge_rate


def soc_after_idle(soc: float, drain_idle: float, wait_s: float) -> float:
    new_soc, _, dead = apply_drain(soc, drain_idle, wait_s)
    return new_soc


def travel_drain(soc: float, drain_move: float, travel_s: float) -> tuple[float, float | None, bool]:
    return apply_drain(soc, drain_move, travel_s)


def work_drain(soc: float, drain_work: float, work_s: float) -> tuple[float, float | None, bool]:
    return apply_drain(soc, drain_work, work_s)
