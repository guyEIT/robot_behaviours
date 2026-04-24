"""Tests for the ``~/acknowledge_plate_released`` Trigger service and the
new iSWAP substates surfaced on Status."""

from __future__ import annotations

import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from std_srvs.srv import Trigger

from hamilton_star_msgs.msg import Event
from hamilton_star_msgs.srv import GetStatus


def _status(helper, name, ros_helpers):
    return ros_helpers.call_service(
        helper,
        helper.create_client(GetStatus, f"/{name}/get_status"),
        GetStatus.Request(),
    ).status


def test_ack_rejected_when_no_plate_held(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    client = helper.create_client(Trigger, f"/{name}/acknowledge_plate_released")
    resp = ros_helpers.call_service(helper, client, Trigger.Request())
    assert resp.success is False
    assert "no plate-held state" in resp.message


def test_ack_clears_holding_plate_flag(sim_node, ros_helpers):
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    # Simulate the state that a mid-handoff failure would leave behind.
    node._fsm.mark_iswap_holding_plate(True)
    assert _status(helper, name, ros_helpers).iswap_holding_plate is True

    client = helper.create_client(Trigger, f"/{name}/acknowledge_plate_released")
    resp = ros_helpers.call_service(helper, client, Trigger.Request())
    assert resp.success is True
    assert "cleared" in resp.message

    assert _status(helper, name, ros_helpers).iswap_holding_plate is False


def test_ack_emits_event_with_last_stage(sim_node, ros_helpers):
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    received: list[Event] = []
    helper.create_subscription(
        Event, f"/{name}/events", lambda m: received.append(m), 50,
    )
    # Let the subscription come up.
    time.sleep(0.2)

    node._fsm.mark_handoff_stage(3)
    node._fsm.mark_iswap_holding_plate(True)

    client = helper.create_client(Trigger, f"/{name}/acknowledge_plate_released")
    resp = ros_helpers.call_service(helper, client, Trigger.Request())
    assert resp.success is True

    got = ros_helpers.wait_for(
        lambda: any(e.kind == "plate_released_ack" for e in received),
        timeout=3.0,
    )
    assert got, f"no plate_released_ack event received; kinds={[e.kind for e in received]}"
    ack = next(e for e in received if e.kind == "plate_released_ack")
    details = dict(zip(ack.keys, ack.values))
    assert details.get("last_handoff_stage_reached") == "3"


def test_status_exposes_new_iswap_substates(sim_node, ros_helpers):
    """The six newly-added Status fields are populated from the FSM snapshot."""
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    s = _status(helper, name, ros_helpers)
    # After a clean sim startup, the action server marks the iSWAP
    # initialized but NOT at safe pose — post-init Y≈626 isn't safe
    # for rotations on this STAR; the first iSWAP-using action parks
    # at SAFE_Y via the pre-flight (see _exec_move_resource /
    # _execute_handoff).
    assert s.iswap_initialized is True
    assert s.iswap_at_safe_pose is False
    assert s.iswap_drive_faulted is False
    assert s.iswap_holding_plate is False
    assert s.iswap_y_path_clear is False  # first handoff will run C0 FY
    assert s.last_handoff_stage_reached == 0


def test_reset_error_clears_iswap_drive_faulted(sim_node, ros_helpers):
    """After a fault + reset_error, iswap_drive_faulted flips back. The
    chatterbox sim returns ``None`` for the two firmware queries the
    soft-recovery path issues, so we stub them with plausible
    responses — this test exercises the reset_error service plumbing,
    not pylabrobot's firmware serialization."""
    from types import SimpleNamespace

    from hamilton_star_msgs.srv import ResetError
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    backend = node._machine.backend

    async def _ok_parking():
        return {"rg": 0}  # 0 => not parked, same semantics as real hw

    async def _ok_position():
        return SimpleNamespace(x=0.0, y=450.0, z=280.0)

    backend.request_iswap_in_parking_position = _ok_parking
    backend.request_iswap_position = _ok_position

    # Simulate a handoff fault: FSM in Error, drive flagged.
    node._fsm.mark_error_and_invalidate(
        "simulated X001/32", mark_iswap_faulted=True,
    )
    assert _status(helper, name, ros_helpers).iswap_drive_faulted is True
    assert _status(helper, name, ros_helpers).op_state == "Error"

    client = helper.create_client(ResetError, f"/{name}/reset_error")
    resp = ros_helpers.call_service(
        helper, client, ResetError.Request(acknowledgment="operator test"),
    )
    assert resp.success is True, resp.message

    s = _status(helper, name, ros_helpers)
    assert s.op_state == "Idle"
    assert s.iswap_drive_faulted is False
    assert s.iswap_initialized is True
    # FI leaves the arm at post-init Y (not SAFE_Y); the next iSWAP
    # op re-parks via the pre-flight.
    assert s.iswap_at_safe_pose is False


def test_reset_error_surfaces_firmware_failure(sim_node, ros_helpers):
    """If the firmware position query raises, reset_error returns
    success=False with a restart-required message and leaves the FSM
    in Error / drive_faulted=True. We swap the backend to a non-
    chatterbox fake so the server takes the real-hardware code path
    (reset_error skips firmware recovery when backend is chatterbox)."""
    from hamilton_star_msgs.srv import ResetError
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    class _FakeBackend:
        """Not-a-chatterbox so the server runs its real-hardware path."""
        async def request_iswap_initialization_status(self):
            return True

        async def request_iswap_in_parking_position(self):
            return {"rg": 0}

        async def initialize_iswap(self):
            return None

        async def request_iswap_position(self):
            raise RuntimeError("iSWAP X-drive still faulted")

    node._machine.backend = _FakeBackend()

    node._fsm.mark_error_and_invalidate(
        "simulated X001/32", mark_iswap_faulted=True,
    )

    client = helper.create_client(ResetError, f"/{name}/reset_error")
    resp = ros_helpers.call_service(
        helper, client, ResetError.Request(acknowledgment="operator test"),
    )
    assert resp.success is False
    assert "restart" in resp.message.lower() or "power-cycle" in resp.message.lower()

    s = _status(helper, name, ros_helpers)
    assert s.op_state == "Error"
    assert s.iswap_drive_faulted is True
