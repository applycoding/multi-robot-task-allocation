# SPDX-License-Identifier: Apache-2.0
"""Charger and dock slot calendar."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Interval:
    start: float
    end: float
    robot_id: str


@dataclass
class SlotCalendar:
    """Committed half-open occupy intervals per site."""

    intervals: dict[str, list[Interval]] = field(default_factory=dict)

    def committed(self, site_id: str) -> list[Interval]:
        return self.intervals.get(site_id, [])

    def overlaps(self, site_id: str, start: float, end: float) -> bool:
        for iv in self.committed(site_id):
            if start < iv.end and end > iv.start:
                return True
        return False

    def next_fit(self, site_id: str, t_arr: float, duration: float) -> float | None:
        """Earliest t >= t_arr with half-open [t, t+duration) free."""
        if duration < 0.0:
            return None
        t = t_arr
        committed = sorted(self.committed(site_id), key=lambda x: x.start)
        for _ in range(len(committed) + 10):
            end = t + duration
            conflict = False
            for iv in committed:
                if t < iv.end and end > iv.start:
                    t = iv.end
                    conflict = True
                    break
            if not conflict:
                return t
        return None

    def book(self, site_id: str, start: float, end: float, robot_id: str) -> None:
        if site_id not in self.intervals:
            self.intervals[site_id] = []
        self.intervals[site_id].append(Interval(start, end, robot_id))
        self.intervals[site_id].sort(key=lambda x: x.start)

    def remove_robot_future(self, site_id: str, robot_id: str, from_t: float) -> None:
        if site_id not in self.intervals:
            return
        self.intervals[site_id] = [
            iv for iv in self.intervals[site_id]
            if not (iv.robot_id == robot_id and iv.start >= from_t - 1e-12)
        ]

    def block_from_death(
        self, site_id: str, robot_id: str, death_t: float, horizon: float
    ) -> bool:
        """If the robot occupies site_id at death_t, extend that interval to horizon.

        Normal charging cannot empty the battery, so callers inject this
        terminal failure. Returns True if a current occupy interval was extended.
        """
        found = False
        for iv in self.committed(site_id):
            if iv.robot_id != robot_id:
                continue
            if iv.start <= death_t + 1e-12 and death_t < iv.end - 1e-12:
                if horizon > iv.end:
                    iv.end = horizon
                found = True
        return found
