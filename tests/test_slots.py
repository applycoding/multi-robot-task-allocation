# SPDX-License-Identifier: Apache-2.0
from multi_robot_task_allocation.slots import SlotCalendar


def test_next_fit_duration_gap() -> None:
    cal = SlotCalendar()
    cal.book("C0", 0.0, 10.0, "R0")
    t = cal.next_fit("C0", 2.0, 5.0)
    assert t is not None
    assert t == 10.0


def test_no_overlap() -> None:
    cal = SlotCalendar()
    cal.book("C0", 0.0, 10.0, "R0")
    cal.book("C0", 10.0, 20.0, "R1")
    assert cal.overlaps("C0", 9.5, 10.5)
    assert cal.next_fit("C0", 0.0, 5.0) == 20.0
