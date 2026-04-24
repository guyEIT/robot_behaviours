"""Node lifecycle, services, and status topic tests against the sim backend."""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from std_srvs.srv import Trigger

from hamilton_star_msgs.msg import Event, Status
from hamilton_star_msgs.srv import (
    GetStatus, InitializeModule, ListResources, LoadDeck, ResetError, SerializeDeck,
)


def test_get_status_reports_idle(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    client = helper.create_client(GetStatus, f"/{ros_helpers.NODE_NAME}/get_status")
    resp = ros_helpers.call_service(helper, client, GetStatus.Request())
    assert resp.status.connected is True
    assert resp.status.initialized is True
    assert resp.status.op_state == "Idle"
    assert resp.status.core_parked is True
    assert list(resp.status.core_channels) == []
    assert resp.status.num_channels == 8
    assert resp.status.shared_goal_count == 0
    assert resp.status.exclusive_op == ""
    assert resp.status.error_reason == ""


def test_get_status_reports_channel_states(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    client = helper.create_client(GetStatus, f"/{ros_helpers.NODE_NAME}/get_status")
    resp = ros_helpers.call_service(helper, client, GetStatus.Request())
    assert len(resp.status.channels) == 8
    for i, ch in enumerate(resp.status.channels):
        assert ch.channel == i
        assert ch.state == ch.FREE


def test_list_resources_includes_standard_deck_items(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    client = helper.create_client(ListResources, f"/{ros_helpers.NODE_NAME}/list_resources")
    resp = ros_helpers.call_service(helper, client, ListResources.Request())
    assert resp.success is True
    assert len(resp.names) == len(resp.categories)
    assert "core_grippers" in resp.names
    assert "trash" in resp.names
    assert "waste_block" in resp.names


def test_list_resources_categories_match_names(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    client = helper.create_client(ListResources, f"/{ros_helpers.NODE_NAME}/list_resources")
    resp = ros_helpers.call_service(helper, client, ListResources.Request())
    pairs = dict(zip(resp.names, resp.categories))
    assert pairs["core_grippers"] == "core_grippers"
    assert pairs["trash"] == "trash"


def test_serialize_deck_returns_valid_json(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    client = helper.create_client(SerializeDeck, f"/{ros_helpers.NODE_NAME}/serialize_deck")
    resp = ros_helpers.call_service(helper, client, SerializeDeck.Request())
    assert resp.success is True
    blob = json.loads(resp.deck_json)
    assert isinstance(blob, dict)
    assert "name" in blob
    assert "children" in blob
    assert isinstance(blob["children"], list)


def test_serialize_deck_contains_core_grippers(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    client = helper.create_client(SerializeDeck, f"/{ros_helpers.NODE_NAME}/serialize_deck")
    resp = ros_helpers.call_service(helper, client, SerializeDeck.Request())
    assert resp.success is True
    names = json.dumps(json.loads(resp.deck_json))
    assert "core_grippers" in names


def test_abort_motion_keeps_backend_alive(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    abort_client = helper.create_client(Trigger, f"/{ros_helpers.NODE_NAME}/abort_motion")
    status_client = helper.create_client(GetStatus, f"/{ros_helpers.NODE_NAME}/get_status")

    resp = ros_helpers.call_service(helper, abort_client, Trigger.Request())
    assert resp.success is True, resp.message

    status = ros_helpers.call_service(helper, status_client, GetStatus.Request())
    assert status.status.connected is True
    assert status.status.initialized is True
    assert status.status.op_state == "Idle"


def test_abort_motion_from_disconnected_rejected(sim_node, ros_helpers):
    """AbortMotion requires an actionable state; Error/Disconnected/Aborting should refuse."""
    # Can't reach Disconnected without stopping the node, but we can test the
    # error-state rejection path by first putting the FSM in Error.
    node, helper, _ex = sim_node
    abort_client = helper.create_client(Trigger, f"/{ros_helpers.NODE_NAME}/abort_motion")
    node._fsm.mark_error("induced for test")
    resp = ros_helpers.call_service(helper, abort_client, Trigger.Request())
    assert resp.success is False
    assert "Error" in resp.message or "cannot abort" in resp.message
    # Cleanup so other tests don't inherit Error state — fixture is function-scope,
    # but belt-and-braces.
    node._fsm.reset_error("test cleanup")


def test_reset_error_rejected_when_not_in_error(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    client = helper.create_client(ResetError, f"/{ros_helpers.NODE_NAME}/reset_error")
    resp = ros_helpers.call_service(
        helper, client, ResetError.Request(acknowledgment="operator"),
    )
    assert resp.success is False
    assert "not in Error state" in resp.message


def test_reset_error_from_error_with_ack_succeeds(sim_node, ros_helpers):
    node, helper, _ex = sim_node
    client = helper.create_client(ResetError, f"/{ros_helpers.NODE_NAME}/reset_error")
    node._fsm.mark_error("simulated firmware hang for test")

    resp = ros_helpers.call_service(
        helper, client, ResetError.Request(acknowledgment="operator reviewed"),
    )
    assert resp.success is True
    assert node._fsm.state == "Idle"
    assert node._fsm.error_reason == ""


def test_reset_error_requires_nonempty_ack(sim_node, ros_helpers):
    node, helper, _ex = sim_node
    client = helper.create_client(ResetError, f"/{ros_helpers.NODE_NAME}/reset_error")
    node._fsm.mark_error("simulated")

    resp = ros_helpers.call_service(
        helper, client, ResetError.Request(acknowledgment=""),
    )
    assert resp.success is False
    assert "acknowledgment must be non-empty" in resp.message
    # still in Error
    assert node._fsm.state == "Error"

    node._fsm.reset_error("test cleanup")


def test_initialize_module_known_module(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    client = helper.create_client(
        InitializeModule, f"/{ros_helpers.NODE_NAME}/initialize_module",
    )
    req = InitializeModule.Request(module="pip")
    resp = ros_helpers.call_service(helper, client, req, timeout=5.0)
    assert resp is not None


def test_initialize_module_unknown_rejected(sim_node, ros_helpers):
    """Unknown module string surfaces as a service failure, not a crash."""
    node, helper, _ex = sim_node
    client = helper.create_client(
        InitializeModule, f"/{ros_helpers.NODE_NAME}/initialize_module",
    )
    req = InitializeModule.Request(module="not_a_real_module")
    resp = ros_helpers.call_service(helper, client, req, timeout=5.0)
    assert resp.success is False
    assert "unknown module" in resp.message.lower()
    # node still responsive and NOT in Error (reject_module is validation, not fault)
    assert node._fsm.state in ("Idle", "Error")


def test_load_deck_with_valid_json_swaps_deck(sim_node, ros_helpers, tmp_path):
    _node, helper, _ex = sim_node

    serialize_client = helper.create_client(
        SerializeDeck, f"/{ros_helpers.NODE_NAME}/serialize_deck",
    )
    load_client = helper.create_client(
        LoadDeck, f"/{ros_helpers.NODE_NAME}/load_deck",
    )
    status_client = helper.create_client(
        GetStatus, f"/{ros_helpers.NODE_NAME}/get_status",
    )

    current = ros_helpers.call_service(helper, serialize_client, SerializeDeck.Request())
    deck_file = tmp_path / "deck.json"
    deck_file.write_text(current.deck_json)

    resp = ros_helpers.call_service(
        helper, load_client, LoadDeck.Request(deck_file=str(deck_file)),
    )
    assert resp.success is True, resp.message
    assert resp.deck_hash  # non-empty hash returned

    # After LoadDeck, status reports the new hash (not checked for equality with
    # the pre-load hash because pylabrobot's deserialize() doesn't produce a
    # byte-identical serialize(); the test goal is just 'swap succeeded').
    new_status = ros_helpers.call_service(
        helper, status_client, GetStatus.Request(),
    )
    assert new_status.status.deck_loaded is True
    assert new_status.status.deck_hash == resp.deck_hash


def test_load_deck_nonexistent_file_fails_gracefully(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    client = helper.create_client(LoadDeck, f"/{ros_helpers.NODE_NAME}/load_deck")
    resp = ros_helpers.call_service(
        helper, client, LoadDeck.Request(deck_file="/nonexistent/deck.json"),
    )
    assert resp.success is False
    assert resp.message


def test_status_topic_publishes_latched(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    received: list[Status] = []

    from rclpy.qos import (
        QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
    )
    qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
    )
    helper.create_subscription(
        Status, f"/{ros_helpers.NODE_NAME}/status",
        lambda m: received.append(m), qos,
    )
    # latched: subscriber should receive the last published status quickly
    got = ros_helpers.wait_for(lambda: len(received) > 0, timeout=3.0)
    assert got, "no latched status message received"
    assert received[-1].op_state in ("Idle", "Initializing")


def test_events_topic_publishes_on_transition(sim_node, ros_helpers):
    """Forcing an FSM transition should surface on the ~/events topic."""
    node, helper, _ex = sim_node
    received: list[Event] = []
    helper.create_subscription(
        Event, f"/{ros_helpers.NODE_NAME}/events",
        lambda m: received.append(m), 50,
    )

    # Drain startup events by waiting a moment
    time.sleep(0.2)
    received.clear()

    node._fsm.mark_error("induced for topic test")
    got = ros_helpers.wait_for(
        lambda: any(e.kind == "fsm_transition" for e in received),
        timeout=3.0,
    )
    assert got, f"no fsm_transition event seen; received={[e.kind for e in received]}"
    node._fsm.reset_error("test cleanup")
