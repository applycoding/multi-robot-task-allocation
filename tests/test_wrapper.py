# SPDX-License-Identifier: Apache-2.0
from multi_robot_task_allocation.alloc.wrapper import RobotState, build_cost_matrix, feasible_plan
from multi_robot_task_allocation.eval.generate import generate
from multi_robot_task_allocation.slots import SlotCalendar


def test_wrapper_charge_free_s1() -> None:
    sc = generate("s1", 20261111)
    cal = SlotCalendar()
    rob = RobotState("R0", tuple(sc.robots[0]["start"]), 1.0, 0.0)
    cand = feasible_plan(sc, rob, 0, cal, 0.0)
    assert cand is not None
    assert not cand.charge_inserted


def test_wrapper_charge_insert_s2() -> None:
    sc = generate("s2", 20261121)
    rob = RobotState("R0", tuple(sc.robots[0]["start"]), 0.35, 0.0)
    direct = feasible_plan(sc, rob, 0, SlotCalendar(), 0.0, allow_charge=False)
    assert direct is None
    charged = feasible_plan(sc, rob, 0, SlotCalendar(), 0.0, allow_charge=True)
    assert charged is not None
    assert charged.charge_inserted


def test_wrapper_known_inf_low_soc() -> None:
    sc = generate("s2", 20261121)
    rob = RobotState("R0", tuple(sc.robots[0]["start"]), 0.0, 0.0)
    cand = feasible_plan(sc, rob, 0, SlotCalendar(), 0.0, allow_charge=True)
    assert cand is None


def test_wrapper_joint_infeasible_same_charger_gap() -> None:
    sc = generate("s2", 20261121)
    pose = tuple(sc.robots[1]["start"])
    rob_a = RobotState("RA", pose, 0.35, 0.0)
    rob_b = RobotState("RB", pose, 0.35, 0.0)
    empty = SlotCalendar()
    first = feasible_plan(sc, rob_a, 3, empty, 0.0)
    second = feasible_plan(sc, rob_b, 3, empty, 0.0)
    assert first is not None and first.charge_inserted
    assert second is not None and second.charge_inserted
    travel = next(s for s in first.steps if s.kind == "travel_charge")
    charge = next(s for s in first.steps if s.kind == "charge")
    arr = travel.travel_s
    dur = charge.service_s
    tight = SlotCalendar()
    tight.book("C0", 0.0, arr, "BLOCK")
    tight.book("C0", arr + dur + 0.05, 10000.0, "BLOCK")
    tight.book("C1", 0.0, 10000.0, "BLOCK")
    first_tight = feasible_plan(sc, rob_a, 3, tight, 0.0)
    second_tight = feasible_plan(sc, rob_b, 3, tight, 0.0)
    assert first_tight is not None
    assert second_tight is not None
    slot_travel = next(s for s in first_tight.steps if s.kind == "travel_charge")
    slot_charge = next(s for s in first_tight.steps if s.kind == "charge")
    slot = tight.next_fit(slot_charge.charger_id or "C0", slot_travel.travel_s, slot_charge.service_s)
    assert slot is not None
    tight.book(slot_charge.charger_id or "C0", slot, slot + slot_charge.service_s, "RA")
    after = feasible_plan(sc, rob_b, 3, tight, 0.0)
    assert after is None


def test_cost_matrix_shape() -> None:
    sc = generate("s1", 20261111)
    robots = [
        RobotState(r["id"], tuple(r["start"]), float(r["start_soc"]), 0.0) for r in sc.robots
    ]
    pending = list(range(len(sc.zones)))
    c, t = build_cost_matrix(sc, robots, pending, SlotCalendar(), 0.0)
    assert c.shape == (4, 8)
