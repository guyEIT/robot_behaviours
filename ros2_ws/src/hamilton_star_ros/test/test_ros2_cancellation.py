"""Cancellation tests — request cancel on an in-flight goal, verify the
goal ends CANCELED, locks/counters unwind, and the node stays usable.

Real pipetting against the chatterbox sim completes in milliseconds, so
we monkey-patch the relevant :class:`Machine` method to await a long
``asyncio.sleep`` — that gives us a reliable window during which the
goal is actually executing and can be cancelled.
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Any

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("hamilton_star_msgs")

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from hamilton_star_msgs.action import (
    Aspirate, JogChannel, MoveResource, PickUpCoreGripper,
    PickUpTips, ReturnCoreGripper,
)
from hamilton_star_msgs.srv import GetStatus


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


def _send_and_wait_until_executing(
    helper, action_client, goal, node, expected_state: str,
    timeout: float = 3.0,
):
    """Send a goal, wait for accept, then wait for FSM to transition into
    ``expected_state`` (so we know the execute callback is running)."""
    assert action_client.wait_for_server(timeout_sec=timeout)
    send_fut = action_client.send_goal_async(goal)
    deadline = time.monotonic() + timeout
    while not send_fut.done():
        if time.monotonic() > deadline:
            raise TimeoutError("send_goal_async did not return")
        time.sleep(0.01)
    gh = send_fut.result()
    assert gh.accepted, "goal unexpectedly rejected"

    # Wait for the FSM to actually enter the expected state.
    deadline = time.monotonic() + timeout
    while node._fsm.state != expected_state:
        if time.monotonic() > deadline:
            raise TimeoutError(
                f"FSM never reached {expected_state}; still in {node._fsm.state}"
            )
        time.sleep(0.01)
    return gh


def _cancel_and_wait(gh, timeout: float = 5.0):
    """Request cancel and wait for the final result."""
    cancel_fut = gh.cancel_goal_async()
    deadline = time.monotonic() + timeout
    while not cancel_fut.done():
        if time.monotonic() > deadline:
            raise TimeoutError("cancel_goal_async did not return")
        time.sleep(0.01)

    result_fut = gh.get_result_async()
    while not result_fut.done():
        if time.monotonic() > deadline:
            raise TimeoutError("get_result_async did not return after cancel")
        time.sleep(0.02)
    return result_fut.result()


def _install_slow_method(node, method_name: str, delay: float = 2.0):
    """Replace ``node._machine.<method_name>`` with a coroutine that sleeps."""
    async def _slow(*args: Any, **kwargs: Any) -> None:
        await asyncio.sleep(delay)
    setattr(node._machine, method_name, _slow)


# -----------------------------------------------------------------------
# Cancellation — shared goal (per-channel slot)
# -----------------------------------------------------------------------

def test_cancel_inflight_shared_goal_ends_canceled(sim_node_pipetting, ros_helpers):
    node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    _install_slow_method(node, "pick_up_tips", delay=3.0)
    client = _client(helper, PickUpTips, name, "pick_up_tips")
    goal = PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0])

    gh = _send_and_wait_until_executing(
        helper, client, goal, node, expected_state="BusyShared",
    )
    result_wrapper = _cancel_and_wait(gh)
    assert result_wrapper.status == GoalStatus.STATUS_CANCELED

    # After cancel: FSM unwinds to Idle, counter 0.
    deadline = time.monotonic() + 2.0
    while node._fsm.state != "Idle" and time.monotonic() < deadline:
        time.sleep(0.02)
    assert node._fsm.state == "Idle"
    assert node._fsm.snapshot()["shared_goal_count"] == 0


def test_cancel_releases_channel_lock(sim_node_pipetting, ros_helpers):
    """A cancelled pick_up_tips on channel 0 must free channel 0 so the
    next goal on ch 0 isn't blocked waiting for the lock."""
    node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    _install_slow_method(node, "pick_up_tips", delay=3.0)
    client = _client(helper, PickUpTips, name, "pick_up_tips")

    gh = _send_and_wait_until_executing(
        helper, client, PickUpTips.Goal(
            tip_rack=TIP_RACK, wells=["A1"], use_channels=[0],
        ),
        node, expected_state="BusyShared",
    )
    _cancel_and_wait(gh)

    # Wait for unwind
    time.sleep(0.3)

    # Restore the real method and send a fresh pickup — must complete quickly.
    from hamilton_star_ros.machine import Machine
    node._machine.pick_up_tips = Machine.pick_up_tips.__get__(node._machine)

    r = ros_helpers.send_goal_blocking(
        helper, client,
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0]),
        timeout=3.0,
    )
    assert r.success is True, r.message


# -----------------------------------------------------------------------
# Cancellation — exclusive goal
# -----------------------------------------------------------------------

def test_cancel_inflight_exclusive_goal_ends_canceled(
    sim_node_pipetting, ros_helpers,
):
    node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    _install_slow_method(node, "jog_channel", delay=3.0)
    client = _client(helper, JogChannel, name, "jog_channel")
    goal = JogChannel.Goal(channel=0, axis="z", target=100.0)

    gh = _send_and_wait_until_executing(
        helper, client, goal, node, expected_state="BusyExclusive",
    )
    result_wrapper = _cancel_and_wait(gh)
    assert result_wrapper.status == GoalStatus.STATUS_CANCELED

    deadline = time.monotonic() + 2.0
    while node._fsm.state != "Idle" and time.monotonic() < deadline:
        time.sleep(0.02)
    assert node._fsm.state == "Idle"
    assert node._fsm.snapshot()["exclusive_op"] == ""


def test_cancel_exclusive_releases_rw_lock(sim_node_pipetting, ros_helpers):
    """After cancelling an exclusive op, a shared op on the machine works."""
    node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    _install_slow_method(node, "jog_channel", delay=3.0)
    client = _client(helper, JogChannel, name, "jog_channel")
    gh = _send_and_wait_until_executing(
        helper, client,
        JogChannel.Goal(channel=0, axis="z", target=100.0),
        node, expected_state="BusyExclusive",
    )
    _cancel_and_wait(gh)

    # Wait for unwind
    time.sleep(0.3)

    # Subsequent shared op (pick up tips) must run — any leaked writer lock
    # would prevent the shared RWLock.shared() from being acquired.
    from hamilton_star_ros.machine import Machine
    node._machine.jog_channel = Machine.jog_channel.__get__(node._machine)

    r = ros_helpers.send_goal_blocking(
        helper, _client(helper, PickUpTips, name, "pick_up_tips"),
        PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0]),
        timeout=3.0,
    )
    assert r.success is True, r.message


# -----------------------------------------------------------------------
# Cancellation — gripper workflow
# -----------------------------------------------------------------------

def test_cancel_inflight_core_gripper_pickup(sim_node_pipetting, ros_helpers):
    """Cancelling pickup leaves the FSM at Idle and core_parked=True (because
    we never finished pickup — substate isn't updated until success)."""
    node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    _install_slow_method(node, "pick_up_core_gripper", delay=3.0)
    client = _client(helper, PickUpCoreGripper, name, "pick_up_core_gripper")

    gh = _send_and_wait_until_executing(
        helper, client,
        PickUpCoreGripper.Goal(front_channel=7),
        node, expected_state="BusyExclusive",
    )
    result_wrapper = _cancel_and_wait(gh)
    assert result_wrapper.status == GoalStatus.STATUS_CANCELED

    deadline = time.monotonic() + 2.0
    while node._fsm.state != "Idle" and time.monotonic() < deadline:
        time.sleep(0.02)
    assert node._fsm.state == "Idle"
    # Substate unchanged — gripper still parked because the success-hook
    # (mark_core_gripper_picked_up) never ran.
    assert node._fsm.core_parked is True
    assert node._fsm.core_channels is None


# -----------------------------------------------------------------------
# Cancelling multiple parallel shared goals independently
# -----------------------------------------------------------------------

def test_cancel_one_of_two_parallel_shared_goals(sim_node_pipetting, ros_helpers):
    """Two pick_up_tips on disjoint channels running in parallel; cancel one,
    the other completes normally."""
    node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    # Make pick_up_tips slow so we can cancel it.
    _install_slow_method(node, "pick_up_tips", delay=3.0)
    client = _client(helper, PickUpTips, name, "pick_up_tips")
    assert client.wait_for_server(timeout_sec=3.0)

    # Fire both goals roughly concurrently.
    g0 = PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0])
    g1 = PickUpTips.Goal(tip_rack=TIP_RACK, wells=["B1"], use_channels=[1])
    send_fut0 = client.send_goal_async(g0)
    send_fut1 = client.send_goal_async(g1)

    deadline = time.monotonic() + 3.0
    while not (send_fut0.done() and send_fut1.done()):
        if time.monotonic() > deadline:
            raise TimeoutError("send_goal_async timeouts")
        time.sleep(0.01)
    gh0 = send_fut0.result()
    gh1 = send_fut1.result()
    assert gh0.accepted and gh1.accepted

    # Wait for both to be actually executing (shared counter >= 2).
    deadline = time.monotonic() + 3.0
    while node._fsm.snapshot()["shared_goal_count"] < 2:
        if time.monotonic() > deadline:
            raise TimeoutError(
                f"shared_goal_count never reached 2; now "
                f"{node._fsm.snapshot()['shared_goal_count']}"
            )
        time.sleep(0.01)

    # Cancel gh0, leave gh1 running.
    cancel_fut = gh0.cancel_goal_async()
    deadline = time.monotonic() + 3.0
    while not cancel_fut.done():
        if time.monotonic() > deadline:
            raise TimeoutError("cancel hang")
        time.sleep(0.01)

    res0_fut = gh0.get_result_async()
    while not res0_fut.done():
        if time.monotonic() > deadline:
            raise TimeoutError("get_result hang on cancelled goal")
        time.sleep(0.02)
    assert res0_fut.result().status == GoalStatus.STATUS_CANCELED

    # gh1 keeps going on its own timeline (still sleeping in our monkey-patch).
    # For this test we just verify it's still active and the server didn't
    # tear it down as collateral damage.
    assert node._fsm.snapshot()["shared_goal_count"] >= 1


# -----------------------------------------------------------------------
# Cancellation during a real (non-monkey-patched) action is a no-op race
# -----------------------------------------------------------------------

def test_cancel_on_fast_action_succeeds_as_done(sim_node_pipetting, ros_helpers):
    """If the action finishes before the cancel request lands, the result is
    SUCCEEDED (the chatterbox sim is instant, so this is the common case)."""
    _node, helper, _ex = sim_node_pipetting
    name = ros_helpers.NODE_NAME

    client = _client(helper, PickUpTips, name, "pick_up_tips")
    goal = PickUpTips.Goal(tip_rack=TIP_RACK, wells=["A1"], use_channels=[0])

    assert client.wait_for_server(timeout_sec=3.0)
    send_fut = client.send_goal_async(goal)
    deadline = time.monotonic() + 3.0
    while not send_fut.done():
        if time.monotonic() > deadline:
            raise TimeoutError("send timeout")
        time.sleep(0.01)
    gh = send_fut.result()
    assert gh.accepted

    # Race: try to cancel, but accept either CANCELED or SUCCEEDED as valid.
    cancel_fut = gh.cancel_goal_async()
    while not cancel_fut.done():
        if time.monotonic() > deadline:
            raise TimeoutError("cancel timeout")
        time.sleep(0.01)

    res_fut = gh.get_result_async()
    while not res_fut.done():
        if time.monotonic() > deadline:
            raise TimeoutError("result timeout")
        time.sleep(0.02)
    status = res_fut.result().status
    assert status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED)
