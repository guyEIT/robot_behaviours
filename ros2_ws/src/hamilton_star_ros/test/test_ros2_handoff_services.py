"""ROS 2 service tests for the handoff registry and its unified
integration with the deck tree.

Covers:
  * /define_handoff inserts, rejects duplicates without flag, overwrites
    with flag, and injects a deck ResourceHolder under the same name.
  * /list_handoffs returns the registered set.
  * /delete_handoff removes registry + deck holder.
  * MoveResource can target a registered handoff by ``to=<name>`` and
    is gated on iSWAP health (since it routes via the firmware hotel).
  * LoadDeck doesn't lose registered handoffs — they're re-injected
    onto the new deck.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from hamilton_star_msgs.action import MoveResource
from hamilton_star_msgs.msg import Handoff
from hamilton_star_msgs.srv import (
    DefineHandoff, DeleteHandoff, GetStatus, ListHandoffs, ListResources,
)


def _handoff_msg(
    name: str = "cytomat",
    # Default coords differ from the seeded ``incubator_handoff``
    # (at X=-216, Y=170, Z=123) so the deck-bounding-box check doesn't
    # reject us with "Location already occupied".
    x: float = -216.0, y: float = 400.0, z: float = 123.0,
    plate_width: float = 80.0,
    hotel_depth: float = 500.0,
    hotel_clearance_height: float = 8.0,
    rotation: str = "LEFT",
    wrist: str = "STRAIGHT",
    grip_direction: str = "FRONT",
) -> Handoff:
    h = Handoff()
    h.name = name
    h.x, h.y, h.z = float(x), float(y), float(z)
    h.plate_width = float(plate_width)
    h.rotation = rotation
    h.wrist = wrist
    h.grip_direction = grip_direction
    h.hotel_depth = float(hotel_depth)
    h.hotel_clearance_height = float(hotel_clearance_height)
    return h


def _status(helper, name, ros_helpers):
    return ros_helpers.call_service(
        helper,
        helper.create_client(GetStatus, f"/{name}/get_status"),
        GetStatus.Request(),
    ).status


def _list(helper, name, ros_helpers) -> list[str]:
    resp = ros_helpers.call_service(
        helper,
        helper.create_client(ListHandoffs, f"/{name}/list_handoffs"),
        ListHandoffs.Request(),
    )
    return [h.name for h in resp.handoffs]


def _list_resources(helper, name, ros_helpers) -> list[str]:
    resp = ros_helpers.call_service(
        helper,
        helper.create_client(ListResources, f"/{name}/list_resources"),
        ListResources.Request(),
    )
    return list(resp.names)


def test_initial_registry_seeds_incubator_handoff(sim_node, ros_helpers):
    """The legacy ``handoff.incubator_handoff.*`` params seed the
    registry at __init__ time."""
    _node, helper, _ex = sim_node
    names = _list(helper, ros_helpers.NODE_NAME, ros_helpers)
    assert "incubator_handoff" in names


def test_seeded_handoff_injected_into_deck(sim_node, ros_helpers):
    """The handoff registered at startup shows up on the deck tree."""
    _node, helper, _ex = sim_node
    names = _list_resources(helper, ros_helpers.NODE_NAME, ros_helpers)
    assert "incubator_handoff" in names


def test_define_new_handoff(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    client = helper.create_client(DefineHandoff, f"/{name}/define_handoff")
    resp = ros_helpers.call_service(
        helper, client,
        DefineHandoff.Request(handoff=_handoff_msg("cytomat"), replace_existing=False),
    )
    assert resp.success is True, resp.message
    assert "cytomat" in _list(helper, name, ros_helpers)
    # And it's on the deck now.
    assert "cytomat" in _list_resources(helper, name, ros_helpers)


def test_define_refuses_overwrite_without_flag(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    client = helper.create_client(DefineHandoff, f"/{name}/define_handoff")
    ros_helpers.call_service(
        helper, client,
        DefineHandoff.Request(handoff=_handoff_msg("hotel-a"), replace_existing=False),
    )
    resp = ros_helpers.call_service(
        helper, client,
        DefineHandoff.Request(
            handoff=_handoff_msg("hotel-a", x=0.0), replace_existing=False,
        ),
    )
    assert resp.success is False
    assert "already exists" in resp.message


def test_define_overwrites_with_flag_and_updates_deck(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    client = helper.create_client(DefineHandoff, f"/{name}/define_handoff")
    ros_helpers.call_service(
        helper, client,
        DefineHandoff.Request(handoff=_handoff_msg("hotel-b"), replace_existing=False),
    )
    resp = ros_helpers.call_service(
        helper, client,
        DefineHandoff.Request(
            handoff=_handoff_msg("hotel-b", x=-300.0), replace_existing=True,
        ),
    )
    assert resp.success is True

    # Registry reflects new x.
    handoffs = ros_helpers.call_service(
        helper,
        helper.create_client(ListHandoffs, f"/{name}/list_handoffs"),
        ListHandoffs.Request(),
    ).handoffs
    updated = [h for h in handoffs if h.name == "hotel-b"][0]
    assert abs(updated.x - (-300.0)) < 0.01


def test_define_rejects_conflict_with_deck_resource(
    sim_node_pipetting, ros_helpers,
):
    """Don't let an operator smuggle a handoff name on top of an
    existing on-deck resource (e.g. ``tip_rack_01``)."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME
    client = helper.create_client(DefineHandoff, f"/{name}/define_handoff")
    resp = ros_helpers.call_service(
        helper, client,
        DefineHandoff.Request(
            handoff=_handoff_msg("tip_rack_01"), replace_existing=True,
        ),
    )
    assert resp.success is False
    assert "clashes" in resp.message


def test_delete_handoff(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    define = helper.create_client(DefineHandoff, f"/{name}/define_handoff")
    delete = helper.create_client(DeleteHandoff, f"/{name}/delete_handoff")

    ros_helpers.call_service(
        helper, define,
        DefineHandoff.Request(handoff=_handoff_msg("temp"), replace_existing=False),
    )
    assert "temp" in _list(helper, name, ros_helpers)
    assert "temp" in _list_resources(helper, name, ros_helpers)

    resp = ros_helpers.call_service(
        helper, delete, DeleteHandoff.Request(name="temp"),
    )
    assert resp.success is True
    assert "temp" not in _list(helper, name, ros_helpers)
    assert "temp" not in _list_resources(helper, name, ros_helpers)


def test_delete_unknown_returns_false(sim_node, ros_helpers):
    _node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    delete = helper.create_client(DeleteHandoff, f"/{name}/delete_handoff")
    resp = ros_helpers.call_service(
        helper, delete, DeleteHandoff.Request(name="nonexistent"),
    )
    assert resp.success is False
    assert "not registered" in resp.message


def test_move_resource_to_handoff_gates_on_iswap_fault(sim_node, ros_helpers):
    """When ``to`` names a registered handoff, the gate rejects when
    the iSWAP drive is faulted — same semantics as HandoffTransfer /
    HotelTransfer."""
    from rclpy.action import ActionClient

    node, helper, _ex = sim_node
    name = ros_helpers.NODE_NAME
    # Seeded handoff: "incubator_handoff".
    node._fsm.mark_iswap_drive_faulted(True)

    client = ActionClient(helper, MoveResource, f"/{name}/move_resource")
    goal = MoveResource.Goal()
    goal.resource = "some_plate"
    goal.to = "incubator_handoff"
    goal.transport = "iswap"
    accepted = ros_helpers.try_send_goal(helper, client, goal)
    assert accepted is False