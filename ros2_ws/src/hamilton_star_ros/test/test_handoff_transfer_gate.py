"""FSM gate tests for ``ActionKind.HANDOFF_TRANSFER``.

Covers the three substate flags added for iSWAP handoff transfers:
  - ``iswap_initialized`` must be True
  - ``iswap_holding_plate`` must be False
  - Any pipetting action invalidates ``iswap_y_path_clear`` (not a gate,
    but observable in the snapshot so the action server knows to re-run
    C0 FY)
plus the per-goal param validation (direction, calibration_name,
on_deck_resource).
"""

from __future__ import annotations

from hamilton_star_ros.machine_fsm import ActionKind, MachineFSM


def _fsm_at_idle() -> MachineFSM:
    fsm = MachineFSM(num_channels=8)
    fsm.connect()
    fsm.finish_init(ok=True)
    assert fsm.state == "Idle"
    return fsm


def _good_params() -> dict:
    return {
        "direction": "to_handoff",
        "calibration_name": "incubator_handoff",
        "on_deck_resource": "plate_carrier_01/site_0",
    }


def test_rejects_when_iswap_not_initialized() -> None:
    fsm = _fsm_at_idle()
    # default: iswap_initialized=False
    gate = fsm.can_accept(ActionKind.HANDOFF_TRANSFER, _good_params())
    assert not gate.ok
    assert "iSWAP not initialized" in gate.reason


def test_accepts_when_initialized_and_params_valid() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    gate = fsm.can_accept(ActionKind.HANDOFF_TRANSFER, _good_params())
    assert gate.ok, gate.reason


def test_rejects_when_holding_plate() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    fsm.mark_iswap_holding_plate(True)
    gate = fsm.can_accept(ActionKind.HANDOFF_TRANSFER, _good_params())
    assert not gate.ok
    assert "holding a plate" in gate.reason


def test_rejects_bad_direction() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    params = _good_params() | {"direction": "sideways"}
    gate = fsm.can_accept(ActionKind.HANDOFF_TRANSFER, params)
    assert not gate.ok
    assert "direction" in gate.reason


def test_rejects_missing_calibration_name() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    params = _good_params() | {"calibration_name": ""}
    gate = fsm.can_accept(ActionKind.HANDOFF_TRANSFER, params)
    assert not gate.ok
    assert "calibration_name" in gate.reason


def test_rejects_missing_on_deck_resource() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    params = _good_params() | {"on_deck_resource": ""}
    gate = fsm.can_accept(ActionKind.HANDOFF_TRANSFER, params)
    assert not gate.ok
    assert "on_deck_resource" in gate.reason


def test_y_path_clear_invalidated_by_pipetting() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    fsm.mark_iswap_y_path_clear(True)
    assert fsm.iswap_y_path_clear

    # Any pipetting action invalidates the Y-path-clear promise.
    fsm.on_shared_accepted(ActionKind.PICK_UP_TIPS)
    assert not fsm.iswap_y_path_clear, (
        "pipetting action should clear iswap_y_path_clear"
    )

    # FSM snapshot reflects it too.
    snap = fsm.snapshot()
    assert snap["iswap_initialized"] is True
    assert snap["iswap_y_path_clear"] is False
    assert snap["iswap_holding_plate"] is False


def test_y_path_clear_invalidated_by_exclusive_channel_move() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    fsm.mark_iswap_y_path_clear(True)

    fsm.on_exclusive_accepted(ActionKind.JOG_CHANNEL)
    assert not fsm.iswap_y_path_clear


def test_y_path_clear_not_invalidated_by_another_handoff() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    fsm.mark_iswap_y_path_clear(True)

    # A second HandoffTransfer shouldn't invalidate the Y-path clear
    # promise — nothing else moved the channels.
    fsm.on_exclusive_accepted(ActionKind.HANDOFF_TRANSFER)
    assert fsm.iswap_y_path_clear
    fsm.on_exclusive_finished(ActionKind.HANDOFF_TRANSFER)


def test_reinitializing_iswap_clears_y_path() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    fsm.mark_iswap_y_path_clear(True)

    # Re-running initialize_iswap flips ``iswap_initialized`` True again;
    # the mutator also clears the Y-path promise because FI re-homes.
    fsm.mark_iswap_initialized(True)
    # Actually, True → True is a no-op for the Y-path side-effect; the
    # side-effect is specifically "if not initialized -> also clear".
    # Simulate a full re-init cycle:
    fsm.mark_iswap_initialized(False)
    assert not fsm.iswap_y_path_clear
    fsm.mark_iswap_initialized(True)
    assert fsm.iswap_initialized
    assert not fsm.iswap_y_path_clear  # still cleared from the False step


def test_handoff_gate_fields_in_snapshot() -> None:
    fsm = _fsm_at_idle()
    snap = fsm.snapshot()
    assert snap["iswap_initialized"] is False
    assert snap["iswap_y_path_clear"] is False
    assert snap["iswap_holding_plate"] is False


# ---------------------------------------------------------------------------
# MOVE_RESOURCE gate when the destination is a registered handoff: the
# server auto-detects this at the action layer and threads the same iSWAP
# health checks as HandoffTransfer through ``can_accept`` via
# ``params["use_unsafe_hotel"]``.
# ---------------------------------------------------------------------------


def test_move_resource_iswap_hotel_rejects_when_drive_faulted() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    fsm.mark_iswap_drive_faulted(True)
    gate = fsm.can_accept(
        ActionKind.MOVE_RESOURCE,
        {"transport": "iswap", "use_unsafe_hotel": True},
    )
    assert not gate.ok
    assert "faulted" in gate.reason


def test_move_resource_iswap_hotel_rejects_when_holding_plate() -> None:
    fsm = _fsm_at_idle()
    fsm.mark_iswap_initialized(True)
    fsm.mark_iswap_holding_plate(True)
    gate = fsm.can_accept(
        ActionKind.MOVE_RESOURCE,
        {"transport": "iswap", "use_unsafe_hotel": True},
    )
    assert not gate.ok
    assert "holding a plate" in gate.reason


def test_move_resource_plain_bypasses_iswap_health() -> None:
    """A normal (non-handoff) MoveResource doesn't gate on iSWAP health
    — the existing transport=core / transport=auto paths still work
    when the iSWAP is faulted."""
    fsm = _fsm_at_idle()
    fsm.mark_iswap_drive_faulted(True)
    gate = fsm.can_accept(
        ActionKind.MOVE_RESOURCE, {"transport": "auto"},
    )
    assert gate.ok, gate.reason
