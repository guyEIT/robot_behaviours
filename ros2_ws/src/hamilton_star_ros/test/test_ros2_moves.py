"""MoveResource and JogChannel tests against the sim backend."""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from rclpy.action import ActionClient

from hamilton_star_msgs.action import (
    JogChannel, MoveResource, PickUpCoreGripper, ReturnCoreGripper,
)
from hamilton_star_msgs.srv import GetStatus


def _client(helper, action_type, name, endpoint):
    return ActionClient(helper, action_type, f"/{name}/{endpoint}")


def _status(helper, name, ros_helpers):
    return ros_helpers.call_service(
        helper,
        helper.create_client(GetStatus, f"/{name}/get_status"),
        GetStatus.Request(),
    ).status


# -----------------------------------------------------------------------
# JogChannel
# -----------------------------------------------------------------------

@pytest.mark.parametrize("axis", ["x", "y", "z"])
def test_jog_channel_each_axis_accepted(sim_node, ros_helpers, axis):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = JogChannel.Goal(channel=0, axis=axis, target=150.0)
    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, JogChannel, name, "jog_channel"), goal,
    )
    assert accepted is True, f"jog on axis={axis} rejected"


@pytest.mark.parametrize("bad_axis", ["w", "", "zz", "1", "xy"])
def test_jog_channel_bad_axis_surfaces_as_error(sim_node, ros_helpers, bad_axis):
    """Bad axis is validated by the backend wrapper; goal accepted, executes and fails."""
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = JogChannel.Goal(channel=0, axis=bad_axis, target=100.0)
    client = _client(helper, JogChannel, name, "jog_channel")
    # Goal is accepted (the FSM doesn't reject the axis string), but the result
    # must report failure — the backend raises ValueError.
    result = ros_helpers.send_goal_blocking(helper, client, goal, timeout=5.0)
    assert result.success is False
    assert "axis" in result.message.lower() or "unknown" in result.message.lower()


def test_jog_channel_holds_exclusive_briefly(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME

    # run jog, then immediately query status — should be Idle again.
    goal = JogChannel.Goal(channel=0, axis="z", target=200.0)
    result = ros_helpers.send_goal_blocking(
        helper, _client(helper, JogChannel, name, "jog_channel"), goal,
    )
    # chatterbox returns success for move_channel_*
    assert result.success is True

    status = _status(helper, name, ros_helpers)
    assert status.op_state == "Idle"
    assert status.exclusive_op == ""


def test_jog_channel_rejected_in_error_state(sim_node, ros_helpers):
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    node._fsm.mark_error("induced for jog error-reject")
    try:
        goal = JogChannel.Goal(channel=0, axis="z", target=100.0)
        accepted = ros_helpers.try_send_goal(
            helper, _client(helper, JogChannel, name, "jog_channel"), goal,
        )
        assert accepted is False
    finally:
        node._fsm.reset_error("test cleanup")


# -----------------------------------------------------------------------
# MoveResource
# -----------------------------------------------------------------------

def test_move_resource_iswap_transport_accepted_from_idle(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    # iSwap transport doesn't require a held CoRe gripper.
    goal = MoveResource.Goal(
        resource="trash", to="trash", transport="iswap",
    )
    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, MoveResource, name, "move_resource"), goal,
    )
    assert accepted is True


def test_move_resource_auto_transport_accepted_from_idle(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = MoveResource.Goal(
        resource="trash", to="trash", transport="auto",
    )
    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, MoveResource, name, "move_resource"), goal,
    )
    assert accepted is True


def test_move_resource_unknown_resource_surfaces_as_error(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = MoveResource.Goal(
        resource="__does_not_exist__",
        to="trash",
        transport="auto",
    )
    client = _client(helper, MoveResource, name, "move_resource")
    result = ros_helpers.send_goal_blocking(helper, client, goal, timeout=5.0)
    assert result.success is False
    # The node marks Error when the action executor raises. That's intentional:
    # an unresolvable resource is a config fault, not a transient failure.
    # Clean the state for teardown-neighbours.


def test_move_resource_core_transport_requires_gripper(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = MoveResource.Goal(resource="trash", to="trash", transport="core")
    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, MoveResource, name, "move_resource"), goal,
    )
    assert accepted is False


def test_move_resource_core_transport_accepted_with_gripper(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    ros_helpers.send_goal_blocking(
        helper,
        _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )

    # Gate-level acceptance is the assertion; actual motion with transport=core
    # against a trash-to-trash move would fail on the backend and mark FSM
    # Error, which is fine — function-scope fixture teardown cleans up.
    goal = MoveResource.Goal(resource="trash", to="trash", transport="core")
    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, MoveResource, name, "move_resource"), goal,
    )
    assert accepted is True
