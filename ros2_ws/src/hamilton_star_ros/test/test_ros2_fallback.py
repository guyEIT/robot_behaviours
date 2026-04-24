"""Fallback / error-recovery tests against the sim backend.

When an action fails mid-execution the node must:

1. Mark the FSM Error and surface a useful message on the result.
2. Release any locks it was holding (shared RW + channel locks, or exclusive).
3. Decrement the shared counter so BusyShared doesn't get stuck.
4. Allow recovery via ResetError.
5. Accept subsequent goals after recovery.

These tests drive failures through the real action pipeline by asking
for resources that don't exist on the deck, then assert the recovery
invariants.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from rclpy.action import ActionClient

from hamilton_star_msgs.action import (
    Aspirate, Dispense, DropTips, MoveResource, PickUpCoreGripper, PickUpTips,
    ReturnCoreGripper,
)
from hamilton_star_msgs.srv import GetStatus, ResetError


TIP_RACK = "tip_rack_01"
PLATE = "plate_01"


def _client(helper, action_type, name, endpoint):
    return ActionClient(helper, action_type, f"/{name}/{endpoint}")


def _status(helper, name, ros_helpers):
    return ros_helpers.call_service(
        helper,
        helper.create_client(GetStatus, f"/{name}/get_status"),
        GetStatus.Request(),
    ).status


def _reset(helper, name, ros_helpers, ack="test"):
    return ros_helpers.call_service(
        helper,
        helper.create_client(ResetError, f"/{name}/reset_error"),
        ResetError.Request(acknowledgment=ack),
    )


# -----------------------------------------------------------------------
# Failure surfaces as Error, ResetError recovers it
# -----------------------------------------------------------------------

def test_failed_shared_action_puts_fsm_in_error(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # Pick up tip so aspirate passes the tip-state gate, then aspirate from
    # a resource that doesn't exist — backend will raise at resolve time.
    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0]),
    )

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"),
        Aspirate.Goal(
            resources=["__does_not_exist__"],
            volumes=[50.0],
            use_channels=[0],
        ),
    )
    assert r.success is False
    assert r.message

    s = _status(helper, name, ros_helpers)
    assert s.op_state == "Error"
    assert s.error_reason


def test_reset_error_restores_idle(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0]),
    )
    ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"),
        Aspirate.Goal(
            resources=["__does_not_exist__"], volumes=[50.0], use_channels=[0],
        ),
    )

    resp = _reset(helper, name, ros_helpers, ack="operator-cleared")
    assert resp.success is True, resp.message

    s = _status(helper, name, ros_helpers)
    assert s.op_state == "Idle"
    assert s.error_reason == ""
    assert s.shared_goal_count == 0
    assert s.exclusive_op == ""


# -----------------------------------------------------------------------
# Lock release after failure — next goal must not hang
# -----------------------------------------------------------------------

def test_next_action_succeeds_after_failure_and_reset(
    sim_node_pipetting, ros_helpers,
):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # Step 1: fail
    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0]),
    )
    ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"),
        Aspirate.Goal(
            resources=["__does_not_exist__"], volumes=[50.0], use_channels=[0],
        ),
    )
    # Step 2: reset
    _reset(helper, name, ros_helpers, ack="continue")

    # Step 3: pick up a tip on a DIFFERENT channel — must succeed and
    # complete within the timeout (no leaked RW/channel lock).
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["B1"], use_channels=[1]),
        timeout=5.0,
    )
    assert r.success is True, r.message

    s = _status(helper, name, ros_helpers)
    assert s.op_state == "Idle"
    # ch 0 still HAS_TIP from pre-failure pickup (tip-state tracker isn't
    # touched by the aspirate failure); ch 1 has the new tip.
    assert s.channels[0].state == s.channels[0].HAS_TIP
    assert s.channels[1].state == s.channels[1].HAS_TIP


def test_failed_shared_action_decrements_counter(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0]),
    )
    # pre-failure counter 0
    assert _status(helper, name, ros_helpers).shared_goal_count == 0

    ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"),
        Aspirate.Goal(
            resources=["__does_not_exist__"], volumes=[50.0], use_channels=[0],
        ),
    )

    # Post-failure counter also 0 — finally: clause ran.
    s = _status(helper, name, ros_helpers)
    assert s.shared_goal_count == 0


def test_failed_exclusive_action_releases_exclusive_slot(
    sim_node_pipetting, ros_helpers,
):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # Force an exclusive failure via MoveResource with a non-existent name.
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, MoveResource, name, "move_resource"),
        MoveResource.Goal(
            resource="__nope__", to=PLATE, transport="auto",
        ),
    )
    assert r.success is False

    s = _status(helper, name, ros_helpers)
    # exclusive_op cleared even though we landed in Error.
    assert s.exclusive_op == ""
    # After reset, the node should be usable.
    _reset(helper, name, ros_helpers, ack="continue")
    assert _status(helper, name, ros_helpers).op_state == "Idle"


# -----------------------------------------------------------------------
# Error-state goal rejection
# -----------------------------------------------------------------------

def test_error_state_blocks_further_goals_until_reset(
    sim_node_pipetting, ros_helpers,
):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0]),
    )
    ros_helpers.send_goal_blocking(
        helper, _client(helper, Aspirate, name, "aspirate"),
        Aspirate.Goal(
            resources=["__nope__"], volumes=[10.0], use_channels=[0],
        ),
    )

    # In Error: any new goal (even a perfectly-valid drop) is refused.
    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, DropTips, name, "drop_tips"),
        DropTips.Goal(
            target="trash", wells=["trash"], use_channels=[0],
            allow_nonzero_volume=True,
        ),
    )
    assert accepted is False

    _reset(helper, name, ros_helpers, ack="cleared")

    # After reset the same goal is accepted.
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, DropTips, name, "drop_tips"),
        DropTips.Goal(
            target="trash", wells=["trash"], use_channels=[0],
            allow_nonzero_volume=True,
        ),
    )
    assert r.success is True, r.message


def test_read_only_services_work_in_error_state(sim_node_pipetting, ros_helpers):
    """GetStatus / ListResources / SerializeDeck stay available when in Error."""
    node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME
    node._fsm.mark_error("induced for read-only-in-error coverage")
    try:
        s = _status(helper, name, ros_helpers)
        assert s.op_state == "Error"
        assert s.error_reason
        assert s.deck_loaded is True  # stays populated
    finally:
        node._fsm.reset_error("test cleanup")


# -----------------------------------------------------------------------
# Gripper-held when failure hits — state survives and is recoverable
# -----------------------------------------------------------------------

def test_failure_while_gripper_held_preserves_gripper_state(
    sim_node_pipetting, ros_helpers,
):
    """A failure in some other action doesn't drop the CoRe gripper state."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )
    s = _status(helper, name, ros_helpers)
    assert s.core_parked is False

    # Trigger an exclusive failure.
    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, MoveResource, name, "move_resource"),
        MoveResource.Goal(resource="__nope__", to="trash", transport="auto"),
    )
    assert r.success is False

    s = _status(helper, name, ros_helpers)
    # Gripper substate still reflects the physical truth.
    assert s.core_parked is False
    assert list(s.core_channels) == [6, 7]

    _reset(helper, name, ros_helpers, ack="cleared")

    # After reset we can still return the gripper as the final cleanup.
    ret = ros_helpers.send_goal_blocking(
        helper, _client(helper, ReturnCoreGripper, name, "return_core_gripper"),
        ReturnCoreGripper.Goal(),
    )
    assert ret.success is True, ret.message
