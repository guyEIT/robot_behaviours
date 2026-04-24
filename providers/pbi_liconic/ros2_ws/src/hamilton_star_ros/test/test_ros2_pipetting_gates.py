"""FSM-gate tests for pipetting actions.

These tests exercise the goal_callback rejection path for each pipetting
action without running real pipetting motion. Real pipetting requires a
fully-populated deck (tip rack + plate) and is covered in
``test_ros2_pipetting_happy_path`` when that fixture is added.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from rclpy.action import ActionClient

from hamilton_star_msgs.action import (
    Aspirate, Aspirate96, Dispense, Dispense96,
    DropTips, DropTips96, PickUpCoreGripper, PickUpTips, PickUpTips96,
    ReturnCoreGripper, Transfer,
)


def _client(helper, action_type, name, endpoint):
    return ActionClient(helper, action_type, f"/{name}/{endpoint}")


# -----------------------------------------------------------------------
# 1-8 channel: all these start from Idle with Free channels (no tips).
# -----------------------------------------------------------------------

@pytest.mark.parametrize("channels", [[0], [0, 1], [0, 7]])
def test_aspirate_rejected_without_tip(sim_node, ros_helpers, channels):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = Aspirate.Goal(
        resources=[f"r_{i}" for i in channels],
        volumes=[50.0] * len(channels),
        use_channels=channels,
    )
    assert ros_helpers.try_send_goal(
        helper, _client(helper, Aspirate, name, "aspirate"), goal,
    ) is False


@pytest.mark.parametrize("channels", [[0], [0, 1], [3, 4]])
def test_dispense_rejected_without_tip(sim_node, ros_helpers, channels):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = Dispense.Goal(
        resources=[f"r_{i}" for i in channels],
        volumes=[10.0] * len(channels),
        use_channels=channels,
    )
    assert ros_helpers.try_send_goal(
        helper, _client(helper, Dispense, name, "dispense"), goal,
    ) is False


def test_drop_tips_rejected_when_no_tip(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = DropTips.Goal(target="trash", wells=["trash"], use_channels=[0])
    assert ros_helpers.try_send_goal(
        helper, _client(helper, DropTips, name, "drop_tips"), goal,
    ) is False


def test_transfer_rejected_without_tip(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = Transfer.Goal(
        source="src", targets=["dst"], source_volume=10.0,
    )
    # transfer uses channel 0 implicitly
    assert ros_helpers.try_send_goal(
        helper, _client(helper, Transfer, name, "transfer"), goal,
    ) is False


@pytest.mark.parametrize("bad_channel", [-1, 8, 9, 100])
def test_pick_up_tips_with_out_of_range_channel_rejected(sim_node, ros_helpers, bad_channel):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = PickUpTips.Goal(
        tip_rack="teaching_tip_rack", wells=["A1"], use_channels=[bad_channel],
    )
    assert ros_helpers.try_send_goal(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"), goal,
    ) is False


def test_aspirate_on_gripper_channel_rejected(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    # Occupy channels 6,7 with CoRe gripper
    ros_helpers.send_goal_blocking(
        helper,
        ActionClient(helper, PickUpCoreGripper, f"/{name}/pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )
    try:
        goal = Aspirate.Goal(
            resources=["x"], volumes=[10.0], use_channels=[6],
        )
        assert ros_helpers.try_send_goal(
            helper, _client(helper, Aspirate, name, "aspirate"), goal,
        ) is False

        goal = Aspirate.Goal(
            resources=["x"], volumes=[10.0], use_channels=[7],
        )
        assert ros_helpers.try_send_goal(
            helper, _client(helper, Aspirate, name, "aspirate"), goal,
        ) is False
    finally:
        ros_helpers.send_goal_blocking(
            helper,
            ActionClient(helper, ReturnCoreGripper, f"/{name}/return_core_gripper"),
            ReturnCoreGripper.Goal(),
        )


# -----------------------------------------------------------------------
# 96-head gates
# -----------------------------------------------------------------------

def test_aspirate96_rejected_when_empty(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = Aspirate96.Goal(resource="plate", volume=20.0)
    assert ros_helpers.try_send_goal(
        helper, _client(helper, Aspirate96, name, "aspirate96"), goal,
    ) is False


def test_dispense96_rejected_when_empty(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = Dispense96.Goal(resource="plate", volume=20.0)
    assert ros_helpers.try_send_goal(
        helper, _client(helper, Dispense96, name, "dispense96"), goal,
    ) is False


def test_drop_tips96_rejected_when_empty(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    goal = DropTips96.Goal(target="trash")
    assert ros_helpers.try_send_goal(
        helper, _client(helper, DropTips96, name, "drop_tips96"), goal,
    ) is False


def test_pick_up_tips96_after_load_rejected(sim_node, ros_helpers):
    """Force the FSM into core96_loaded=True and assert pickup96 is rejected."""
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    node._fsm.mark_core96_loaded(True)
    try:
        goal = PickUpTips96.Goal(tip_rack="teaching_tip_rack")
        assert ros_helpers.try_send_goal(
            helper, _client(helper, PickUpTips96, name, "pick_up_tips96"), goal,
        ) is False
    finally:
        node._fsm.mark_core96_loaded(False)


# -----------------------------------------------------------------------
# Error-state rejection: all actions refused.
# -----------------------------------------------------------------------

@pytest.mark.parametrize(("action_type", "endpoint", "goal_factory"), [
    (Aspirate, "aspirate", lambda: Aspirate.Goal(resources=["x"], volumes=[10.0], use_channels=[0])),
    (Dispense, "dispense", lambda: Dispense.Goal(resources=["x"], volumes=[10.0], use_channels=[0])),
    (PickUpTips, "pick_up_tips", lambda: PickUpTips.Goal(tip_rack="teaching_tip_rack", wells=["A1"], use_channels=[0])),
    (DropTips, "drop_tips", lambda: DropTips.Goal(target="trash", wells=["trash"], use_channels=[0])),
    (PickUpCoreGripper, "pick_up_core_gripper", lambda: PickUpCoreGripper.Goal(front_channel=7)),
    (ReturnCoreGripper, "return_core_gripper", lambda: ReturnCoreGripper.Goal()),
])
def test_error_state_rejects_all_actions(sim_node, ros_helpers, action_type, endpoint, goal_factory):
    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    node._fsm.mark_error("induced for error-reject coverage")
    try:
        accepted = ros_helpers.try_send_goal(
            helper, _client(helper, action_type, name, endpoint), goal_factory(),
        )
        assert accepted is False, f"{endpoint} accepted a goal in Error state"
    finally:
        node._fsm.reset_error("test cleanup")
