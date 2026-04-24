"""Unit tests for :mod:`hamilton_star_ros.machine_fsm`."""

from __future__ import annotations

from hamilton_star_ros.machine_fsm import (
    ActionKind, ChannelState, FSMEvent, MachineFSM,
)


def _collect_events() -> tuple[MachineFSM, list[FSMEvent]]:
    events: list[FSMEvent] = []
    fsm = MachineFSM(num_channels=8, event_sink=events.append)
    return fsm, events


def _to_idle(fsm: MachineFSM) -> None:
    fsm.connect()
    fsm.finish_init(ok=True)
    assert fsm.state == "Idle"


def test_initial_state_is_disconnected() -> None:
    fsm, _ = _collect_events()
    assert fsm.state == "Disconnected"
    assert not fsm.snapshot()["connected"]
    assert not fsm.snapshot()["initialized"]


def test_happy_path_to_idle() -> None:
    fsm, events = _collect_events()
    fsm.connect()
    assert fsm.state == "Initializing"
    fsm.finish_init(ok=True)
    assert fsm.state == "Idle"
    # every real transition emits an event
    kinds = [e.kind for e in events]
    assert kinds.count("fsm_transition") == 2


def test_initialization_failure_goes_to_error() -> None:
    fsm, _ = _collect_events()
    fsm.connect()
    fsm.finish_init(ok=False, reason="usb timeout")
    assert fsm.state == "Error"
    assert fsm.error_reason == "usb timeout"


def test_shared_acquire_and_release() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.on_shared_accepted(ActionKind.ASPIRATE)
    assert fsm.state == "BusyShared"
    fsm.on_shared_accepted(ActionKind.DISPENSE)
    assert fsm.state == "BusyShared"  # stays; counter is 2
    fsm.on_shared_finished(ActionKind.ASPIRATE)
    assert fsm.state == "BusyShared"  # counter is 1
    fsm.on_shared_finished(ActionKind.DISPENSE)
    assert fsm.state == "Idle"


def test_exclusive_blocks_shared() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.on_exclusive_accepted(ActionKind.MOVE_RESOURCE)
    assert fsm.state == "BusyExclusive"
    gate = fsm.can_accept(
        ActionKind.ASPIRATE,
        {"use_channels": [0]},
    )
    assert not gate.ok
    assert "exclusive op in progress" in gate.reason
    fsm.on_exclusive_finished(ActionKind.MOVE_RESOURCE)
    assert fsm.state == "Idle"


def test_shared_blocks_exclusive() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.channels[0] = ChannelState.HAS_TIP  # so aspirate doesn't get rejected
    fsm.on_shared_accepted(ActionKind.ASPIRATE)
    gate = fsm.can_accept(ActionKind.MOVE_RESOURCE, {"transport": "auto"})
    assert not gate.ok
    assert "exclusive op requires Idle" in gate.reason


def test_error_rejects_all_goals() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_error("firmware hang")
    assert fsm.state == "Error"
    gate = fsm.can_accept(ActionKind.ASPIRATE, {"use_channels": [0]})
    assert not gate.ok
    assert "machine in error state" in gate.reason


def test_reset_error_requires_nonempty_ack() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_error("test")
    assert not fsm.reset_error("").ok
    assert not fsm.reset_error("   ").ok
    assert fsm.reset_error("operator: checked").ok
    assert fsm.state == "Idle"
    assert fsm.error_reason == ""


def test_reset_error_only_from_error() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    res = fsm.reset_error("nope")
    assert not res.ok


def test_tip_state_gates_aspirate() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    # channel 0 has no tip -> aspirate rejected
    gate = fsm.can_accept(
        ActionKind.ASPIRATE, {"use_channels": [0]},
    )
    assert not gate.ok
    assert "no tip" in gate.reason
    fsm.mark_tips_picked_up([0])
    gate = fsm.can_accept(
        ActionKind.ASPIRATE, {"use_channels": [0]},
    )
    assert gate.ok


def test_pick_up_tips_requires_free_channel() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_tips_picked_up([0])
    gate = fsm.can_accept(
        ActionKind.PICK_UP_TIPS, {"use_channels": [0]},
    )
    assert not gate.ok
    assert "not free" in gate.reason


def test_core_gripper_blocks_aspirate_on_held_channels() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_core_gripper_picked_up(front_channel=7)  # -> channels (6,7)
    gate = fsm.can_accept(ActionKind.ASPIRATE, {"use_channels": [6]})
    assert not gate.ok
    assert "holds CoRe gripper" in gate.reason
    # channel 3 is still free but has no tip, so aspirate still rejected:
    gate = fsm.can_accept(ActionKind.ASPIRATE, {"use_channels": [3]})
    assert not gate.ok  # because no tip


def test_core_gripper_double_pickup_rejected() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_core_gripper_picked_up(front_channel=7)
    gate = fsm.can_accept(
        ActionKind.PICK_UP_CORE_GRIPPER,
        {"front_channel": 5},
    )
    assert not gate.ok
    assert "already held" in gate.reason


def test_core_gripper_return_requires_held() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    gate = fsm.can_accept(ActionKind.RETURN_CORE_GRIPPER, {})
    assert not gate.ok
    assert "already parked" in gate.reason


def test_core_gripper_pickup_validates_front_channel_range() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    # front_channel=0 invalid (back would be -1)
    gate = fsm.can_accept(
        ActionKind.PICK_UP_CORE_GRIPPER, {"front_channel": 0},
    )
    assert not gate.ok and "out of range" in gate.reason or "must be in" in gate.reason
    # front_channel == num_channels invalid
    gate = fsm.can_accept(
        ActionKind.PICK_UP_CORE_GRIPPER, {"front_channel": 8},
    )
    assert not gate.ok


def test_core_gripper_pickup_requires_free_pair() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_tips_picked_up([6])
    gate = fsm.can_accept(
        ActionKind.PICK_UP_CORE_GRIPPER, {"front_channel": 7},
    )
    assert not gate.ok
    assert "not free" in gate.reason


def test_move_resource_transport_core_requires_held_gripper() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    gate = fsm.can_accept(ActionKind.MOVE_RESOURCE, {"transport": "core"})
    assert not gate.ok
    assert "requires a held CoRe gripper" in gate.reason
    fsm.mark_core_gripper_picked_up(front_channel=7)
    gate = fsm.can_accept(ActionKind.MOVE_RESOURCE, {"transport": "core"})
    assert gate.ok


def test_channel_out_of_range_rejected() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    gate = fsm.can_accept(ActionKind.PICK_UP_TIPS, {"use_channels": [9]})
    assert not gate.ok
    assert "out of range" in gate.reason


def test_snapshot_reflects_substate_changes() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_tips_picked_up([0, 1])
    fsm.mark_core_gripper_picked_up(front_channel=7)  # -> channels (6,7)
    snap = fsm.snapshot()
    assert snap["core_parked"] is False
    assert snap["core_channels"] == [6, 7]
    assert set(snap["busy_channels"]) == {0, 1, 6, 7}
    assert snap["op_state"] == "Idle"


def test_core96_gates() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    assert not fsm.can_accept(ActionKind.DROP_TIPS_96, {}).ok
    assert not fsm.can_accept(ActionKind.ASPIRATE_96, {}).ok
    assert fsm.can_accept(ActionKind.PICK_UP_TIPS_96, {}).ok
    fsm.mark_core96_loaded(True)
    assert not fsm.can_accept(ActionKind.PICK_UP_TIPS_96, {}).ok
    assert fsm.can_accept(ActionKind.ASPIRATE_96, {}).ok


def test_abort_transitions() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.channels[0] = ChannelState.HAS_TIP
    fsm.on_shared_accepted(ActionKind.ASPIRATE)
    assert fsm.begin_abort().ok
    assert fsm.state == "Aborting"
    fsm.finish_abort(ok=True)
    assert fsm.state == "Idle"
    assert fsm.snapshot()["shared_goal_count"] == 0


def test_abort_failure_goes_to_error() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.on_exclusive_accepted(ActionKind.MOVE_RESOURCE)
    fsm.begin_abort()
    fsm.finish_abort(ok=False, reason="firmware stuck")
    assert fsm.state == "Error"
    assert fsm.error_reason == "firmware stuck"


def test_abort_rejected_from_error() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_error("x")
    res = fsm.begin_abort()
    assert not res.ok


# ---------------------------------------------------------------------------
# Self-healing substates: iswap_drive_faulted, iswap_at_safe_pose,
# last_handoff_stage_reached, and mark_error_and_invalidate.
# ---------------------------------------------------------------------------


def test_new_substates_default_values() -> None:
    fsm, _ = _collect_events()
    snap = fsm.snapshot()
    assert snap["iswap_drive_faulted"] is False
    assert snap["iswap_at_safe_pose"] is False
    assert snap["last_handoff_stage_reached"] == 0


def test_mark_iswap_drive_faulted_emits_event() -> None:
    fsm, events = _collect_events()
    _to_idle(fsm)
    fsm.mark_iswap_drive_faulted(True)
    fault_events = [
        e for e in events
        if e.kind == "iswap_state" and e.details.get("iswap_drive_faulted") == "True"
    ]
    assert len(fault_events) == 1
    assert fault_events[0].severity == 2  # ERROR severity for drive fault


def test_mark_iswap_at_safe_pose_round_trip() -> None:
    fsm, events = _collect_events()
    _to_idle(fsm)
    fsm.mark_iswap_at_safe_pose(True)
    assert fsm.snapshot()["iswap_at_safe_pose"] is True
    fsm.mark_iswap_at_safe_pose(False)
    assert fsm.snapshot()["iswap_at_safe_pose"] is False
    safe_events = [
        e for e in events
        if e.kind == "iswap_state" and "iswap_at_safe_pose" in e.details
    ]
    assert len(safe_events) == 2


def test_mark_handoff_stage_tracks_and_resets() -> None:
    fsm, events = _collect_events()
    _to_idle(fsm)
    for s in (1, 2, 3):
        fsm.mark_handoff_stage(s)
    assert fsm.snapshot()["last_handoff_stage_reached"] == 3
    fsm.mark_handoff_stage(0)  # reset on success
    assert fsm.snapshot()["last_handoff_stage_reached"] == 0
    stage_events = [e for e in events if e.kind == "handoff_stage"]
    assert [e.details["last_handoff_stage_reached"] for e in stage_events] == [
        "1", "2", "3", "0",
    ]


def test_handoff_gate_rejected_when_drive_faulted() -> None:
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_iswap_initialized(True)  # would otherwise pass
    fsm.mark_iswap_drive_faulted(True)
    gate = fsm.can_accept(
        ActionKind.HANDOFF_TRANSFER,
        {
            "direction": "to_handoff",
            "calibration_name": "incubator_handoff",
            "on_deck_resource": "plate_01",
        },
    )
    assert not gate.ok
    assert "faulted" in gate.reason
    # The faulted check should fire BEFORE the initialized check.
    fsm.mark_iswap_initialized(False)
    gate2 = fsm.can_accept(
        ActionKind.HANDOFF_TRANSFER,
        {
            "direction": "to_handoff",
            "calibration_name": "incubator_handoff",
            "on_deck_resource": "plate_01",
        },
    )
    assert not gate2.ok
    assert "faulted" in gate2.reason  # faulted takes precedence


def test_mark_error_and_invalidate_clears_iswap_flags() -> None:
    fsm, events = _collect_events()
    _to_idle(fsm)
    fsm.mark_iswap_initialized(True)
    fsm.mark_iswap_y_path_clear(True)
    fsm.mark_iswap_at_safe_pose(True)
    fsm.mark_iswap_holding_plate(True)
    fsm.mark_handoff_stage(3)

    fsm.mark_error_and_invalidate(
        "handoff crashed at stage 3", mark_iswap_faulted=True,
    )

    snap = fsm.snapshot()
    assert snap["op_state"] == "Error"
    # Pessimistically invalidated.
    assert snap["iswap_initialized"] is False
    assert snap["iswap_y_path_clear"] is False
    assert snap["iswap_at_safe_pose"] is False
    assert snap["iswap_drive_faulted"] is True
    # Preserved so operator can act on them.
    assert snap["iswap_holding_plate"] is True
    assert snap["last_handoff_stage_reached"] == 3


def test_mark_error_and_invalidate_can_skip_faulted_flag() -> None:
    """Non-iSWAP exceptions (e.g. tip pickup) shouldn't flag the drive."""
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_iswap_initialized(True)

    fsm.mark_error_and_invalidate(
        "pipetting failed", mark_iswap_faulted=False,
    )

    snap = fsm.snapshot()
    assert snap["op_state"] == "Error"
    assert snap["iswap_drive_faulted"] is False
    # iSWAP init is still pessimistically cleared because a fault might
    # have moved the arm (pipetting share the Y axis).
    assert snap["iswap_initialized"] is False


def test_mark_error_legacy_shim_still_works() -> None:
    """``mark_error`` is a legacy wrapper; existing callers keep working
    (no drive fault marked, default pessimistic invalidations)."""
    fsm, _ = _collect_events()
    _to_idle(fsm)
    fsm.mark_iswap_initialized(True)
    fsm.mark_error("something failed")
    snap = fsm.snapshot()
    assert snap["op_state"] == "Error"
    assert snap["iswap_initialized"] is False
    assert snap["iswap_drive_faulted"] is False  # legacy shim default


def test_mark_error_no_op_from_error() -> None:
    fsm, events = _collect_events()
    _to_idle(fsm)
    fsm.mark_error_and_invalidate("first fault")
    fsm.mark_error_and_invalidate("second fault")  # no-op transition
    # Only one actual transition to Error.
    transitions = [e for e in events if e.kind == "fsm_transition"]
    assert transitions[-1].message == "state -> Error"
    # Second call still emits an error event (for visibility) but does
    # not transition.
    err_events = [e for e in events if e.kind == "error"]
    assert len(err_events) >= 1
