"""Stage-by-stage test driver for the CoRe II gripper via the ROS 2 action server.

Progression is deliberately slow so you can stop after any stage:

  stage 1  status snapshot only — zero motion
  stage 2  pick up the CoRe II gripper, then return it (contained
           motion in the gripper parking area)
  stage 3  pick up gripper → MoveResource (transport=core) → return
           gripper. Requires a deck with an actual plate.

Each stage also runs all earlier stages, so ``stage 2`` includes a
fresh status check. A y/N confirmation gates each physical action
unless ``--yes`` is passed.

Prereq: the action server must already be running (keep the USB
connection warm across stages — end-stop calibration on every cold
connect is slow and mechanically wearing):

    pixi run -e ros2 action-server \\
        deck_file:=$PWD/ros2_ws/src/hamilton_star_bringup/config/star_deck.json

Then in another terminal:

    pixi run -e ros2 bash -lc \\
        'source ros2_ws/install/setup.bash && python test_core_gripper_ros2.py 2'

Use ``--plate`` and ``--to`` in stage 3 to name the source / destination
resources on the deck (defaults: ``plate_01`` → ``trash``).
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from hamilton_star_msgs.action import (
    MoveResource,
    PickUpCoreGripper,
    ReturnCoreGripper,
)
from hamilton_star_msgs.srv import GetStatus


DEFAULT_NAMESPACE = "hamilton_star_action_server"
_SKIP_CONFIRM = False


def _confirm(prompt: str) -> None:
    if _SKIP_CONFIRM:
        print(f"\n{prompt} [auto-yes]")
        return
    ans = input(f"\n{prompt} [y/N] ").strip().lower()
    if ans not in ("y", "yes"):
        print("aborting.")
        sys.exit(1)


def _call_service(node: Node, client: Any, request: Any, timeout: float = 5.0) -> Any:
    if not client.wait_for_service(timeout_sec=timeout):
        raise TimeoutError(f"service {client.srv_name} unavailable")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError(f"service {client.srv_name} timed out after {timeout}s")
    return future.result()


def _send_goal(
    node: Node,
    client: ActionClient,
    goal: Any,
    result_timeout: float = 60.0,
    accept_timeout: float = 10.0,
) -> Any:
    if not client.wait_for_server(timeout_sec=accept_timeout):
        raise TimeoutError(f"action {client._action_name} unavailable")
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=accept_timeout)
    gh = send_future.result()
    if gh is None:
        raise RuntimeError(f"no response to goal on {client._action_name}")
    if not gh.accepted:
        raise RuntimeError(f"goal on {client._action_name} was rejected by the server")
    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=result_timeout)
    if not result_future.done():
        raise TimeoutError(
            f"result on {client._action_name} did not arrive in {result_timeout}s"
        )
    return result_future.result().result


def _print_status(s: Any) -> None:
    print(f"  op_state      : {s.op_state}")
    print(f"  connected     : {s.connected}")
    print(f"  initialized   : {s.initialized}")
    print(f"  deck_loaded   : {s.deck_loaded}")
    print(f"  num_channels  : {s.num_channels}")
    print(f"  core_parked   : {s.core_parked}")
    print(f"  core_channels : {list(s.core_channels)}")
    print(f"  busy_channels : {list(s.busy_channels)}")
    print(f"  exclusive_op  : {s.exclusive_op or '-'}")
    if s.error_reason:
        print(f"  error_reason  : {s.error_reason}")


def stage_status(node: Node, ns: str) -> Any:
    print("== stage 1: status snapshot ==")
    client = node.create_client(GetStatus, f"/{ns}/get_status")
    resp = _call_service(node, client, GetStatus.Request())
    _print_status(resp.status)
    if not resp.status.connected:
        raise RuntimeError("action server reports connected=false; is the STAR plugged in?")
    if not resp.status.initialized:
        raise RuntimeError("action server has not finished setup() yet; wait and retry")
    if resp.status.op_state == "Error":
        raise RuntimeError(f"node is in Error state: {resp.status.error_reason}")
    if not resp.status.core_parked:
        raise RuntimeError(
            "CoRe II gripper is not parked. Call /return_core_gripper before re-running."
        )
    return resp.status


def stage_gripper_cycle(node: Node, ns: str, front_channel: int) -> None:
    print(
        f"\n== stage 2: pick up + return CoRe II gripper "
        f"(front_channel={front_channel}, using channels "
        f"[{front_channel - 1}, {front_channel}]) =="
    )
    _confirm("Pick up CoRe II gripper tools?")

    pickup = ActionClient(node, PickUpCoreGripper, f"/{ns}/pick_up_core_gripper")
    r = _send_goal(node, pickup, PickUpCoreGripper.Goal(front_channel=front_channel))
    if not r.success:
        raise RuntimeError(f"pickup failed: {r.message}")
    print(f"  pickup ok; channels_used={list(r.channels_used)}")
    for w in r.warnings:
        print(f"  warn: {w}")

    # Short pause so you can visually verify the gripper was actually seized
    # before sending the return command.
    time.sleep(1.0)

    _confirm("Return CoRe II gripper tools?")
    ret = ActionClient(node, ReturnCoreGripper, f"/{ns}/return_core_gripper")
    r = _send_goal(node, ret, ReturnCoreGripper.Goal())
    if not r.success:
        raise RuntimeError(f"return failed: {r.message}")
    print("  return ok")
    for w in r.warnings:
        print(f"  warn: {w}")


def stage_plate_move(
    node: Node,
    ns: str,
    front_channel: int,
    resource: str,
    destination: str,
    pickup_direction: str,
    drop_direction: str,
) -> None:
    print(
        f"\n== stage 3: transport '{resource}' → '{destination}' "
        f"via CoRe II gripper =="
    )
    _confirm(
        f"Pick up gripper and move plate '{resource}' to '{destination}'?"
    )

    pickup = ActionClient(node, PickUpCoreGripper, f"/{ns}/pick_up_core_gripper")
    r = _send_goal(node, pickup, PickUpCoreGripper.Goal(front_channel=front_channel))
    if not r.success:
        raise RuntimeError(f"pickup failed: {r.message}")
    print(f"  pickup ok; channels_used={list(r.channels_used)}")

    try:
        move = ActionClient(node, MoveResource, f"/{ns}/move_resource")
        goal = MoveResource.Goal()
        goal.resource = resource
        goal.to = destination
        goal.transport = "core"
        goal.pickup_direction = pickup_direction
        goal.drop_direction = drop_direction
        r = _send_goal(node, move, goal, result_timeout=180.0)
        if not r.success:
            raise RuntimeError(f"move_resource failed: {r.message}")
        print(f"  move ok")
        for w in r.warnings:
            print(f"  warn: {w}")
    finally:
        # Always try to park the gripper, even if the move failed, so the
        # STAR isn't left holding the tools.
        ret = ActionClient(node, ReturnCoreGripper, f"/{ns}/return_core_gripper")
        try:
            r2 = _send_goal(node, ret, ReturnCoreGripper.Goal())
            if r2.success:
                print("  gripper returned")
            else:
                print(f"  !! gripper return failed: {r2.message}")
        except Exception as exc:  # noqa: BLE001
            print(f"  !! gripper return raised: {exc}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "stage", type=int, choices=[1, 2, 3],
        help="1=status, 2=gripper pickup+return, 3=gripper+plate move",
    )
    p.add_argument(
        "--namespace", default=DEFAULT_NAMESPACE,
        help=f"action server node name (default: {DEFAULT_NAMESPACE})",
    )
    p.add_argument(
        "--front-channel", type=int, default=7,
        help="front channel for gripper pickup (default: 7, i.e. channels 6+7)",
    )
    p.add_argument(
        "--plate", default="plate_01",
        help="resource name to move in stage 3 (default: plate_01)",
    )
    p.add_argument(
        "--to", default="trash", dest="destination",
        help="destination resource in stage 3 (default: trash)",
    )
    p.add_argument(
        "--pickup-direction", default="front",
        choices=["front", "back", "left", "right"],
    )
    p.add_argument(
        "--drop-direction", default="front",
        choices=["front", "back", "left", "right"],
    )
    p.add_argument(
        "--yes", action="store_true", help="skip all confirmation prompts",
    )
    args = p.parse_args()

    global _SKIP_CONFIRM
    _SKIP_CONFIRM = args.yes

    rclpy.init()
    node = rclpy.create_node("core_gripper_test_driver")
    try:
        stage_status(node, args.namespace)
        if args.stage >= 2:
            stage_gripper_cycle(node, args.namespace, args.front_channel)
        if args.stage >= 3:
            stage_plate_move(
                node, args.namespace, args.front_channel,
                args.plate, args.destination,
                args.pickup_direction, args.drop_direction,
            )
        print("\nok.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
