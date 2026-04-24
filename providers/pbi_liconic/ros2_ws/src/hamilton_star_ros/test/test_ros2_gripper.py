"""CoRe II gripper pickup / return / interaction tests against the sim backend."""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from rclpy.action import ActionClient

from hamilton_star_msgs.action import (
    MoveResource, PickUpCoreGripper, ReturnCoreGripper,
)
from hamilton_star_msgs.srv import GetStatus


def _pickup_client(helper, name):
    return ActionClient(helper, PickUpCoreGripper, f"/{name}/pick_up_core_gripper")


def _return_client(helper, name):
    return ActionClient(helper, ReturnCoreGripper, f"/{name}/return_core_gripper")


def _move_client(helper, name):
    return ActionClient(helper, MoveResource, f"/{name}/move_resource")


def _status(helper, name, ros_helpers):
    return ros_helpers.call_service(
        helper,
        helper.create_client(GetStatus, f"/{name}/get_status"),
        GetStatus.Request(),
    ).status


def test_pickup_and_return_round_trip(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    goal = PickUpCoreGripper.Goal(gripper_resource="core_grippers", front_channel=7)
    result = ros_helpers.send_goal_blocking(helper, _pickup_client(helper, name), goal)
    assert result.success is True, result.message
    assert list(result.channels_used) == [6, 7]

    s = _status(helper, name, ros_helpers)
    assert s.core_parked is False
    assert list(s.core_channels) == [6, 7]
    assert s.channels[6].state == s.channels[6].HOLDS_GRIPPER
    assert s.channels[7].state == s.channels[7].HOLDS_GRIPPER
    assert set(s.busy_channels) >= {6, 7}

    ret = ros_helpers.send_goal_blocking(helper, _return_client(helper, name), ReturnCoreGripper.Goal())
    assert ret.success is True, ret.message

    s = _status(helper, name, ros_helpers)
    assert s.core_parked is True
    assert list(s.core_channels) == []
    assert s.channels[6].state == s.channels[6].FREE
    assert s.channels[7].state == s.channels[7].FREE


def test_pickup_with_valid_middle_front_channel(sim_node, ros_helpers):
    """Arbitrary valid front_channel (not just the typical 7) works too."""
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    goal = PickUpCoreGripper.Goal(front_channel=4)
    result = ros_helpers.send_goal_blocking(helper, _pickup_client(helper, name), goal)
    assert result.success is True, result.message
    assert list(result.channels_used) == [3, 4]

    # Return so the fixture teardown finds a clean state — function-scope
    # fixture recreates the node anyway, but this also exercises return on
    # a non-default channel pair.
    ros_helpers.send_goal_blocking(helper, _return_client(helper, name), ReturnCoreGripper.Goal())


@pytest.mark.parametrize("bad_channel", [-1, 0, 8, 9, 99])
def test_pickup_invalid_front_channel_rejected(sim_node, ros_helpers, bad_channel):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = PickUpCoreGripper.Goal(front_channel=bad_channel)
    accepted = ros_helpers.try_send_goal(helper, _pickup_client(helper, name), goal)
    assert accepted is False, f"front_channel={bad_channel} was accepted but shouldn't be"


def test_pickup_rejected_when_already_held(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    first = PickUpCoreGripper.Goal(front_channel=7)
    ros_helpers.send_goal_blocking(helper, _pickup_client(helper, name), first)

    second = PickUpCoreGripper.Goal(front_channel=5)
    accepted = ros_helpers.try_send_goal(helper, _pickup_client(helper, name), second)
    assert accepted is False

    ros_helpers.send_goal_blocking(helper, _return_client(helper, name), ReturnCoreGripper.Goal())


def test_return_rejected_when_parked(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    accepted = ros_helpers.try_send_goal(
        helper, _return_client(helper, name), ReturnCoreGripper.Goal(),
    )
    assert accepted is False


def test_move_resource_core_transport_rejected_without_gripper(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = MoveResource.Goal(resource="trash", to="trash", transport="core")
    accepted = ros_helpers.try_send_goal(helper, _move_client(helper, name), goal)
    assert accepted is False


def test_move_resource_core_transport_accepted_with_gripper(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _pickup_client(helper, name),
        PickUpCoreGripper.Goal(front_channel=7),
    )

    # Gate-level acceptance is the assertion. Trash-to-trash via core will
    # fail at backend motion and mark FSM Error; function-scope fixture
    # teardown cleans up.
    goal = MoveResource.Goal(resource="trash", to="trash", transport="core")
    accepted = ros_helpers.try_send_goal(helper, _move_client(helper, name), goal)
    assert accepted is True


def test_status_topic_reflects_gripper_state_change(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    s0 = _status(helper, name, ros_helpers)
    assert s0.core_parked is True

    ros_helpers.send_goal_blocking(
        helper, _pickup_client(helper, name), PickUpCoreGripper.Goal(front_channel=7),
    )
    s1 = _status(helper, name, ros_helpers)
    assert s1.core_parked is False

    ros_helpers.send_goal_blocking(
        helper, _return_client(helper, name), ReturnCoreGripper.Goal(),
    )
    s2 = _status(helper, name, ros_helpers)
    assert s2.core_parked is True
