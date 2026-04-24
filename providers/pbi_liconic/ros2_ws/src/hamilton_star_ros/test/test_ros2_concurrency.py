"""Concurrency behaviour of the ROS 2 action server against the sim backend.

These tests drive the FSM directly from the test (via ``node._fsm``) to
simulate ongoing shared / exclusive work, then assert the ROS-level
goal_callback gates the right way. That lets us verify the writer-preference
and reject-on-conflict behaviour without needing a real tip rack / plate on
the deck.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from rclpy.action import ActionClient

from hamilton_star_msgs.action import (
    Aspirate, JogChannel, MoveResource, PickUpCoreGripper, ReturnCoreGripper,
)
from hamilton_star_msgs.srv import GetStatus

from hamilton_star_ros.machine_fsm import ActionKind, ChannelState


def _client(helper, action_type, name, endpoint):
    return ActionClient(helper, action_type, f"/{name}/{endpoint}")


def _status(helper, name, ros_helpers):
    return ros_helpers.call_service(
        helper,
        helper.create_client(GetStatus, f"/{name}/get_status"),
        GetStatus.Request(),
    ).status


def test_exclusive_rejected_during_busy_shared(sim_node, ros_helpers):
    """While any shared goal is notionally in flight, exclusive ops reject."""
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    # Pretend one channel has a tip and an aspirate is in flight.
    node._fsm.channels[0] = ChannelState.HAS_TIP
    node._fsm.on_shared_accepted(ActionKind.ASPIRATE)
    assert node._fsm.state == "BusyShared"
    try:
        move_goal = MoveResource.Goal(resource="trash", to="trash", transport="auto")
        accepted = ros_helpers.try_send_goal(
            helper, _client(helper, MoveResource, name, "move_resource"), move_goal,
        )
        assert accepted is False

        jog_goal = JogChannel.Goal(channel=0, axis="z", target=200.0)
        accepted = ros_helpers.try_send_goal(
            helper, _client(helper, JogChannel, name, "jog_channel"), jog_goal,
        )
        assert accepted is False

        gripper_goal = PickUpCoreGripper.Goal(front_channel=7)
        accepted = ros_helpers.try_send_goal(
            helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
            gripper_goal,
        )
        assert accepted is False
    finally:
        node._fsm.on_shared_finished(ActionKind.ASPIRATE)
        node._fsm.channels[0] = ChannelState.FREE


def test_shared_rejected_during_busy_exclusive(sim_node, ros_helpers):
    """While an exclusive op is notionally in flight, shared ops reject."""
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    node._fsm.channels[0] = ChannelState.HAS_TIP
    node._fsm.on_exclusive_accepted(ActionKind.MOVE_RESOURCE)
    assert node._fsm.state == "BusyExclusive"
    try:
        aspirate_goal = Aspirate.Goal(
            resources=["x"], volumes=[10.0], use_channels=[0],
        )
        accepted = ros_helpers.try_send_goal(
            helper, _client(helper, Aspirate, name, "aspirate"), aspirate_goal,
        )
        assert accepted is False
    finally:
        node._fsm.on_exclusive_finished(ActionKind.MOVE_RESOURCE)
        node._fsm.channels[0] = ChannelState.FREE


def test_parallel_shared_goals_both_accepted(sim_node, ros_helpers):
    """Two gripper pickups on different fronts — second one must fail only
    because core_parked flips, not because the first held a lock forever."""
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    first = PickUpCoreGripper.Goal(front_channel=7)
    result1 = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        first,
    )
    assert result1.success is True

    # Second identical pickup rejected (gripper held), not stuck on lock.
    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=5),
    )
    assert accepted is False

    # Return completes — if the first pickup leaked its lock, return would hang.
    ret = ros_helpers.send_goal_blocking(
        helper, _client(helper, ReturnCoreGripper, name, "return_core_gripper"),
        ReturnCoreGripper.Goal(),
    )
    assert ret.success is True


def test_fsm_state_recovers_after_exclusive_op(sim_node, ros_helpers):
    """Full exclusive cycle leaves FSM back at Idle with clean counters."""
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    pre = _status(helper, name, ros_helpers)
    assert pre.op_state == "Idle"
    assert pre.shared_goal_count == 0
    assert pre.exclusive_op == ""

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )
    ros_helpers.send_goal_blocking(
        helper, _client(helper, ReturnCoreGripper, name, "return_core_gripper"),
        ReturnCoreGripper.Goal(),
    )

    post = _status(helper, name, ros_helpers)
    assert post.op_state == "Idle"
    assert post.shared_goal_count == 0
    assert post.exclusive_op == ""
    assert post.core_parked is True


def test_shared_counter_increments_and_decrements(sim_node, ros_helpers):
    """Drive the FSM counter directly and observe it through GetStatus."""
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    node._fsm.channels[0] = ChannelState.HAS_TIP
    node._fsm.channels[1] = ChannelState.HAS_TIP
    try:
        node._fsm.on_shared_accepted(ActionKind.ASPIRATE)
        node._fsm.on_shared_accepted(ActionKind.DISPENSE)
        s = _status(helper, name, ros_helpers)
        assert s.op_state == "BusyShared"
        assert s.shared_goal_count == 2

        node._fsm.on_shared_finished(ActionKind.ASPIRATE)
        s = _status(helper, name, ros_helpers)
        assert s.shared_goal_count == 1

        node._fsm.on_shared_finished(ActionKind.DISPENSE)
        s = _status(helper, name, ros_helpers)
        assert s.op_state == "Idle"
        assert s.shared_goal_count == 0
    finally:
        node._fsm.channels[0] = ChannelState.FREE
        node._fsm.channels[1] = ChannelState.FREE
