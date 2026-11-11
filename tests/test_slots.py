# SPDX-License-Identifier: Apache-2.0
from multi_robot_task_allocation.eval.gates import count_slot_conflicts
from multi_robot_task_allocation.sim import RunLog
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


def test_death_in_slot_blocks_until_horizon() -> None:
    """Injected death inside a charge slot keeps the slot blocked until horizon."""
    horizon = 100.0
    death_t = 5.0
    cal = SlotCalendar()
    cal.book("C0", 0.0, 10.0, "R0")
    occupy = [{"site_id": "C0", "start": 0.0, "end": 10.0, "robot_id": "R0"}]
    events = [
        {"t": 0.0, "kind": "start_charge", "robot_id": "R0", "charger_id": "C0"},
        {"t": death_t, "kind": "battery_empty", "robot_id": "R0", "injected": True},
    ]

    assert cal.block_from_death("C0", "R0", death_t, horizon) is True
    for rec in occupy:
        if (
            rec["site_id"] == "C0"
            and rec["robot_id"] == "R0"
            and rec["start"] <= death_t + 1e-12
            and death_t < rec["end"] - 1e-12
        ):
            rec["end"] = max(float(rec["end"]), horizon)

    committed = cal.committed("C0")
    assert len(committed) == 1
    assert committed[0].start == 0.0
    assert committed[0].end == horizon
    assert committed[0].robot_id == "R0"
    assert cal.next_fit("C0", 0.0, 5.0) == horizon
    assert cal.overlaps("C0", 10.0, 20.0)
    assert not cal.overlaps("C0", horizon, horizon + 5.0)

    cal.book("C0", horizon, horizon + 5.0, "R1")
    occupy.append({"site_id": "C0", "start": horizon, "end": horizon + 5.0, "robot_id": "R1"})
    events.append({"t": horizon, "kind": "start_charge", "robot_id": "R1", "charger_id": "C0"})

    log = RunLog(T_horizon=horizon, events=events, occupy=occupy)
    assert count_slot_conflicts(log) == 0
    # Half-open intervals may touch at the horizon but must not overlap in length.
    assert occupy[0]["end"] == occupy[1]["start"]
    assert occupy[0]["end"] - occupy[0]["start"] > 0.0


def test_kill_robot_extends_occupy_on_event_log() -> None:
    from multi_robot_task_allocation.eval.generate import generate
    from multi_robot_task_allocation.sim import Simulator

    scenario = generate("s1", 20261111)
    sim = Simulator(scenario, "greedy", None)
    sim._book_occupy("C0", 0.0, 10.0, "R0")
    sim._kill_robot("R0", 5.0, 0.0)
    recs = [r for r in sim.log.occupy if r["robot_id"] == "R0" and r["site_id"] == "C0"]
    assert recs
    assert recs[0]["end"] == scenario.T_horizon
    committed = sim.calendar.committed("C0")
    assert len(committed) == 1
    assert committed[0].end == scenario.T_horizon
    assert count_slot_conflicts(sim.log) == 0
    assert sim.calendar.next_fit("C0", 0.0, 5.0) == scenario.T_horizon
    assert sim.robot_dead["R0"] is True
