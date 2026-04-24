"""End-to-end CoRe II gripper workflows and state-machine cascades.

Where ``test_ros2_gripper.py`` covers each gripper action in isolation,
this file chains them together and verifies the substate cascades that
follow (channels flip to HOLDS_GRIPPER, LoadDeck rejected while held,
post-return pickups work on the former gripper channels, etc.).
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from rclpy.action import ActionClient

from hamilton_star_msgs.action import (
    Aspirate, Dispense, DropTips, MoveResource,
    PickUpCoreGripper, PickUpTips, ReturnCoreGripper,
)
from hamilton_star_msgs.srv import GetStatus, LoadDeck, SerializeDeck


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


# -----------------------------------------------------------------------
# Channel-state cascades after pickup / return
# -----------------------------------------------------------------------

def test_pickup_cascades_channels_to_holds_gripper(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )

    s = _status(helper, name, ros_helpers)
    assert s.core_parked is False
    assert list(s.core_channels) == [6, 7]
    for ch in (6, 7):
        assert s.channels[ch].state == s.channels[ch].HOLDS_GRIPPER
    for ch in range(6):
        assert s.channels[ch].state == s.channels[ch].FREE


def test_pickup_blocks_tip_pickup_on_gripper_channels(
    sim_node_pipetting, ros_helpers,
):
    """After gripper pickup, ch 6/7 are HOLDS_GRIPPER — PickUpTips on them is rejected."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )

    for bad_ch in (6, 7):
        accepted = ros_helpers.try_send_goal(
            helper, _client(helper, PickUpTips, name, "pick_up_tips"),
            PickUpTips.Goal(
                tip_rack=TIP_RACK, wells=[f"A{bad_ch + 1}"], use_channels=[bad_ch],
            ),
        )
        assert accepted is False, f"PickUpTips accepted on gripper ch {bad_ch}"


def test_pickup_still_allows_other_channels_for_pipetting(
    sim_node_pipetting, ros_helpers,
):
    """After gripper pickup on 6/7, channels 0–5 can still pick up tips."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(
            tip_rack=TIP_RACK, wells=["A1", "B1", "C1"], use_channels=[0, 1, 2],
        ),
    )
    assert r.success is True, r.message

    s = _status(helper, name, ros_helpers)
    for ch in (0, 1, 2):
        assert s.channels[ch].state == s.channels[ch].HAS_TIP
    for ch in (6, 7):
        assert s.channels[ch].state == s.channels[ch].HOLDS_GRIPPER


def test_return_restores_channels_to_free(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )
    ros_helpers.send_goal_blocking(
        helper, _client(helper, ReturnCoreGripper, name, "return_core_gripper"),
        ReturnCoreGripper.Goal(),
    )

    s = _status(helper, name, ros_helpers)
    assert s.core_parked is True
    assert list(s.core_channels) == []
    for ch in (6, 7):
        assert s.channels[ch].state == s.channels[ch].FREE


def test_after_return_pickup_tips_on_former_gripper_channel(
    sim_node_pipetting, ros_helpers,
):
    """Full cycle: gripper up → gripper down → pickup tips on ch 7."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )
    ros_helpers.send_goal_blocking(
        helper, _client(helper, ReturnCoreGripper, name, "return_core_gripper"),
        ReturnCoreGripper.Goal(),
    )

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(
            tip_rack=TIP_RACK, wells=["H1"], use_channels=[7],
        ),
    )
    assert r.success is True, r.message

    s = _status(helper, name, ros_helpers)
    assert s.channels[7].state == s.channels[7].HAS_TIP


# -----------------------------------------------------------------------
# Service-level restrictions while gripper is held
# -----------------------------------------------------------------------

def test_load_deck_rejected_while_gripper_held(
    sim_node_pipetting, ros_helpers, tmp_path,
):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # Capture current deck to disk first (we'll try to load it back later).
    ser = ros_helpers.call_service(
        helper,
        helper.create_client(SerializeDeck, f"/{name}/serialize_deck"),
        SerializeDeck.Request(),
    )
    deck_file = tmp_path / "deck.json"
    deck_file.write_text(ser.deck_json)

    # Hold the gripper.
    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )

    # LoadDeck must refuse.
    resp = ros_helpers.call_service(
        helper,
        helper.create_client(LoadDeck, f"/{name}/load_deck"),
        LoadDeck.Request(deck_file=str(deck_file)),
    )
    assert resp.success is False
    assert "parked" in resp.message.lower() or "gripper" in resp.message.lower()


def test_load_deck_accepted_after_return(
    sim_node_pipetting, ros_helpers, tmp_path,
):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ser = ros_helpers.call_service(
        helper,
        helper.create_client(SerializeDeck, f"/{name}/serialize_deck"),
        SerializeDeck.Request(),
    )
    deck_file = tmp_path / "deck.json"
    deck_file.write_text(ser.deck_json)

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )
    ros_helpers.send_goal_blocking(
        helper, _client(helper, ReturnCoreGripper, name, "return_core_gripper"),
        ReturnCoreGripper.Goal(),
    )

    resp = ros_helpers.call_service(
        helper,
        helper.create_client(LoadDeck, f"/{name}/load_deck"),
        LoadDeck.Request(deck_file=str(deck_file)),
    )
    assert resp.success is True, resp.message


# -----------------------------------------------------------------------
# Full workflow: pickup → move plate via core → return
# -----------------------------------------------------------------------

def test_gripper_move_workflow_transport_core(sim_node_pipetting, ros_helpers):
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )

    # Move plate to the deck trash using the CoRe gripper. Chatterbox sim
    # may or may not produce a clean motion command for this — we only
    # assert gate-level acceptance + FSM state flow.
    accepted = ros_helpers.try_send_goal(
        helper, _client(helper, MoveResource, name, "move_resource"),
        MoveResource.Goal(
            resource=PLATE, to="trash", transport="core",
        ),
    )
    assert accepted is True

    # Even if the motion failed, the FSM's counter + exclusive-slot
    # accounting must unwind. If the op succeeded we stay Idle; if it
    # failed we'd be in Error (fixture teardown cleans up either way).


def test_gripper_pickup_rejected_during_busy_exclusive(
    sim_node_pipetting, ros_helpers,
):
    """While one exclusive op is nominally running, a second gripper pickup is refused."""
    node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    from hamilton_star_ros.machine_fsm import ActionKind
    node._fsm.on_exclusive_accepted(ActionKind.MOVE_RESOURCE)
    try:
        accepted = ros_helpers.try_send_goal(
            helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
            PickUpCoreGripper.Goal(front_channel=7),
        )
        assert accepted is False
    finally:
        node._fsm.on_exclusive_finished(ActionKind.MOVE_RESOURCE)


def test_gripper_workflow_emits_events(sim_node_pipetting, ros_helpers):
    """Pickup + return publish identifiable events on ~/events."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    from hamilton_star_msgs.msg import Event as EventMsg
    received: list[EventMsg] = []
    helper.create_subscription(
        EventMsg, f"/{name}/events",
        lambda m: received.append(m), 50,
    )

    ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper"),
        PickUpCoreGripper.Goal(front_channel=7),
    )
    ros_helpers.send_goal_blocking(
        helper, _client(helper, ReturnCoreGripper, name, "return_core_gripper"),
        ReturnCoreGripper.Goal(),
    )

    got = ros_helpers.wait_for(
        lambda: (
            any(e.kind == "core_gripper_picked_up" for e in received)
            and any(e.kind == "core_gripper_returned" for e in received)
        ),
        timeout=3.0,
    )
    assert got, f"missing gripper events; kinds={[e.kind for e in received]}"
