"""
Pure Python BehaviorTree executor for the Robot Skills Framework.

Parses BT.CPP v4 XML and executes trees by calling ROS2 action servers
directly via rclpy. No C++ dependency, no BT.CPP library needed.

Supports:
  - Control flow: Sequence, Fallback, Parallel, RetryUntilSuccessful, SubTree
  - Action nodes: Generic ROS2 action client wrapper (13 skill types)
  - Utility nodes: SetPose, ComputePreGraspPose, CheckGraspSuccess, etc.
  - Human interaction: HumanNotification, HumanConfirm, HumanInput, HumanTask
  - Blackboard: Dict-based variable passing with {var} references
"""

from __future__ import annotations

import asyncio
import copy
import enum
import time
import uuid
from typing import Any, Callable, Optional
from xml.etree import ElementTree

import rclpy.action
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_skills_msgs.msg import HumanPrompt, HumanResponse, LogEvent, TaskState
from robot_skills_msgs.srv import AcquireLease, ReleaseLease, RenewLease


# ═══════════════════════════════════════════════════════════════════════════════
# Node status
# ═══════════════════════════════════════════════════════════════════════════════

class NodeStatus(enum.Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


# ═══════════════════════════════════════════════════════════════════════════════
# Blackboard
# ═══════════════════════════════════════════════════════════════════════════════

class Blackboard:
    def __init__(self, parent: Optional[Blackboard] = None):
        self._data: dict[str, Any] = {}
        self._parent = parent

    def get(self, key: str, default=None):
        if key in self._data:
            return self._data[key]
        if self._parent:
            return self._parent.get(key, default)
        return default

    def set(self, key: str, value: Any):
        self._data[key] = value

    def resolve(self, value: str):
        """Resolve '{var}' references to blackboard values."""
        if isinstance(value, str) and value.startswith("{") and value.endswith("}"):
            return self.get(value[1:-1])
        return value


# ═══════════════════════════════════════════════════════════════════════════════
# Tree nodes
# ═══════════════════════════════════════════════════════════════════════════════

class TreeNode:
    def __init__(self, name: str, attrs: dict[str, str]):
        self.name = name
        self.attrs = attrs
        self.children: list[TreeNode] = []

    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        raise NotImplementedError

    async def halt(self, ctx: ExecutionContext):
        for child in self.children:
            await child.halt(ctx)


# ── Control flow ─────────────────────────────────────────────────────────────

class SequenceNode(TreeNode):
    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        for child in self.children:
            status = await child.tick(bb, ctx)
            if status != NodeStatus.SUCCESS:
                return status
        return NodeStatus.SUCCESS


class FallbackNode(TreeNode):
    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        for child in self.children:
            status = await child.tick(bb, ctx)
            if status != NodeStatus.FAILURE:
                return status
        return NodeStatus.FAILURE


class ParallelNode(TreeNode):
    def __init__(self, name, attrs):
        super().__init__(name, attrs)
        self.success_count = int(attrs.get("success_count", "-1"))
        self.failure_count = int(attrs.get("failure_count", "1"))

    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        # KNOWN LIMITATION: asyncio.create_task / asyncio.as_completed below
        # require an asyncio event loop, but rclpy's action-server callback
        # doesn't run inside one — same root cause as WaitForDuration's
        # "no running event loop" bug. None of the current test BTs use
        # Parallel; if you author one that does, this needs to switch to
        # rclpy.task.Future + parallel future-tracking (mirroring
        # _rclpy_sleep further down) or to spawning the children on a
        # MultiThreadedExecutor + callback groups.
        tasks = [asyncio.create_task(c.tick(bb, ctx)) for c in self.children]
        successes = 0
        failures = 0
        target_successes = (
            self.success_count if self.success_count > 0 else len(self.children)
        )

        for coro in asyncio.as_completed(tasks):
            status = await coro
            if status == NodeStatus.SUCCESS:
                successes += 1
                if successes >= target_successes:
                    for t in tasks:
                        t.cancel()
                    return NodeStatus.SUCCESS
            elif status == NodeStatus.FAILURE:
                failures += 1
                if failures >= self.failure_count:
                    for t in tasks:
                        t.cancel()
                    return NodeStatus.FAILURE

        return NodeStatus.SUCCESS if successes >= target_successes else NodeStatus.FAILURE


class RetryNode(TreeNode):
    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        attempts = int(self.attrs.get("num_attempts", "3"))
        for i in range(attempts):
            status = await self.children[0].tick(bb, ctx)
            if status == NodeStatus.SUCCESS:
                return NodeStatus.SUCCESS
            ctx.log(f"Retry {i + 1}/{attempts} failed for '{self.name}'", "warn")
        return NodeStatus.FAILURE


class SubTreeNode(TreeNode):
    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        tree_id = self.attrs.get("ID", "")
        subtree = ctx.trees.get(tree_id)
        if not subtree:
            ctx.log(f"SubTree '{tree_id}' not found", "error")
            return NodeStatus.FAILURE
        # _autoremap: share the same blackboard
        return await subtree.tick(bb, ctx)


# ── Action nodes (ROS2 action clients) ──────────────────────────────────────

class RosActionNode(TreeNode):
    def __init__(self, name, attrs, action_type, server_name, input_map,
                 output_map, goal_defaults=None, post_process=None):
        super().__init__(name, attrs)
        self.action_type = action_type
        self.server_name = attrs.get("server_name", server_name)
        self.input_map = input_map      # {xml_attr: goal_field}
        self.output_map = output_map    # {result_field: xml_attr}
        self.goal_defaults = goal_defaults or {}
        self.post_process = post_process
        self._client: Optional[ActionClient] = None
        self._goal_handle = None

    def _get_client(self, ctx: ExecutionContext) -> ActionClient:
        if self._client is None:
            self._client = ActionClient(
                ctx.ros_node, self.action_type, self.server_name
            )
        return self._client

    def _build_goal(self, bb: Blackboard):
        goal = self.action_type.Goal()
        for xml_attr, goal_field in self.input_map.items():
            if xml_attr in self.attrs:
                raw = self.attrs[xml_attr]
                resolved = bb.resolve(raw)
                if resolved is not None:
                    _set_goal_field(goal, goal_field, resolved)
            elif xml_attr in self.goal_defaults:
                _set_goal_field(goal, goal_field, self.goal_defaults[xml_attr])
        return goal

    def _store_outputs(self, result, bb: Blackboard):
        for result_field, xml_attr in self.output_map.items():
            if xml_attr in self.attrs:
                bb_key = self.attrs[xml_attr]
                if bb_key.startswith("{") and bb_key.endswith("}"):
                    bb.set(bb_key[1:-1], getattr(result, result_field, None))

    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        logger = ctx.ros_node.get_logger()
        # Cooperative abort: if an upstream signal (lease revoked, manual
        # stop) has set _abort_requested on the blackboard, refuse to start
        # new actions. In-flight goals are torn down via halt(), not here.
        if bb.get("_abort_requested"):
            logger.warn(
                f"[{self.name}] skipping send_goal — abort flagged: "
                f"{bb.get('_abort_reason', 'unspecified')}"
            )
            return NodeStatus.FAILURE

        # Mark this skill as currently running BEFORE wait_for_server / build,
        # so the dashboard's BT viewer highlights the node even if those
        # steps fail (e.g. action server missing → 5 s timeout). Otherwise
        # the user just sees the BT go RUNNING → 5 s of dead air → FAILURE
        # with no node ever lit up — making it impossible to tell which
        # step blocked.
        ctx.on_skill_started(self.name)

        client = self._get_client(ctx)
        if not client.wait_for_server(timeout_sec=5.0):
            logger.error(
                f"[{self.name}] action server '{self.server_name}' not available"
            )
            ctx.on_skill_failed(self.name)
            return NodeStatus.FAILURE

        try:
            goal = self._build_goal(bb)
        except Exception as e:
            import traceback
            logger.error(
                f"[{self.name}] _build_goal failed: {type(e).__name__}: {e}\n"
                f"{traceback.format_exc()}"
            )
            ctx.on_skill_failed(self.name)
            return NodeStatus.FAILURE

        send_future = client.send_goal_async(goal)
        try:
            # `await_or_halt` polls ctx.is_halted() every 50 ms so the operator's
            # Cancel reaches us within ~1 poll period instead of waiting for the
            # action server to acknowledge the goal.
            goal_handle = await await_or_halt(send_future, ctx)
        except HaltedError:
            logger.info(f"[{self.name}] halted before goal accepted: {ctx.halt_reason}")
            ctx.on_skill_failed(self.name)
            return NodeStatus.FAILURE
        except Exception as e:
            import traceback
            logger.error(
                f"[{self.name}] send_goal failed: {type(e).__name__}: {e}\n"
                f"{traceback.format_exc()}"
            )
            ctx.on_skill_failed(self.name)
            return NodeStatus.FAILURE
        if not goal_handle.accepted:
            logger.warn(
                f"[{self.name}] goal rejected by {self.server_name} "
                f"(check server state)"
            )
            ctx.on_skill_failed(self.name)
            return NodeStatus.FAILURE

        self._goal_handle = goal_handle
        ctx.inflight_goals.append(goal_handle)
        try:
            result_future = goal_handle.get_result_async()
            try:
                wrapped = await await_or_halt(result_future, ctx)
            except HaltedError:
                # Best-effort hard cancel of the in-flight goal so the action
                # server stops doing work. We don't await the cancel future —
                # the action server will tear down asynchronously, and our
                # tree is exiting anyway.
                logger.info(f"[{self.name}] halted mid-action: {ctx.halt_reason}")
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                ctx.on_skill_failed(self.name)
                return NodeStatus.FAILURE
        finally:
            try:
                ctx.inflight_goals.remove(goal_handle)
            except ValueError:
                pass
            self._goal_handle = None

        result = wrapped.result
        if getattr(result, "success", True):
            self._store_outputs(result, bb)
            if self.post_process:
                # post_process is already a resolved callable (looked up
                # against _POST_PROCESSORS by SkillDiscovery / static
                # registry); call it directly.
                self.post_process(result, self.attrs, bb)
            ctx.on_skill_completed(self.name)
            return NodeStatus.SUCCESS
        else:
            msg = getattr(result, "message", "")
            logger.warn(f"[{self.name}] action returned success=False: {msg}")
            ctx.on_skill_failed(self.name)
            return NodeStatus.FAILURE

    async def halt(self, ctx: ExecutionContext):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None


# ── Utility nodes (pure Python, synchronous) ────────────────────────────────

class SyncNode(TreeNode):
    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        # Fire skill events so the dashboard's BT viewer highlights this
        # node while it ticks. Without these, only RosActionNodes were
        # publishing task_state — non-action nodes (LogEvent,
        # ComputePreGraspPose, WaitForDuration, HumanConfirm, …) were
        # invisible to the live BT view, so HumanInteractionDemo and
        # similar trees showed no progress at all.
        ctx.on_skill_started(self.name)
        try:
            status = self.execute(bb, ctx)
        except Exception:
            ctx.on_skill_failed(self.name)
            raise
        if status == NodeStatus.SUCCESS:
            ctx.on_skill_completed(self.name)
        elif status == NodeStatus.FAILURE:
            ctx.on_skill_failed(self.name)
        return status

    def execute(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        raise NotImplementedError


class SetPoseNode(SyncNode):
    def execute(self, bb, ctx):
        pose = PoseStamped()
        pose.header.frame_id = self.attrs.get("frame_id", "world")
        pose.pose.position.x = float(self.attrs.get("x", "0.0"))
        pose.pose.position.y = float(self.attrs.get("y", "0.0"))
        pose.pose.position.z = float(self.attrs.get("z", "0.0"))
        pose.pose.orientation.w = 1.0
        out_key = self.attrs.get("pose", "")
        if out_key.startswith("{") and out_key.endswith("}"):
            bb.set(out_key[1:-1], pose)
        return NodeStatus.SUCCESS


class ComputePreGraspPoseNode(SyncNode):
    def execute(self, bb, ctx):
        input_key = self.attrs.get("input_pose", "")
        input_pose = bb.resolve(input_key)
        if input_pose is None:
            return NodeStatus.FAILURE
        z_offset = float(self.attrs.get("z_offset_m", "0.05"))
        out = copy.deepcopy(input_pose)
        out.pose.position.z += z_offset
        out_key = self.attrs.get("output_pose", "")
        if out_key.startswith("{") and out_key.endswith("}"):
            bb.set(out_key[1:-1], out)
        return NodeStatus.SUCCESS


class TransformPoseNode(SyncNode):
    def execute(self, bb, ctx):
        input_pose = bb.resolve(self.attrs.get("input_pose", ""))
        if input_pose is None:
            return NodeStatus.FAILURE
        target_frame = self.attrs.get("target_frame", "world")
        out = copy.deepcopy(input_pose)
        out.header.frame_id = target_frame
        out_key = self.attrs.get("output_pose", "")
        if out_key.startswith("{") and out_key.endswith("}"):
            bb.set(out_key[1:-1], out)
        return NodeStatus.SUCCESS


class CheckGraspSuccessNode(SyncNode):
    def execute(self, bb, ctx):
        grasped = bb.resolve(self.attrs.get("object_grasped", "false"))
        pos = bb.resolve(self.attrs.get("final_position", "0.0"))
        min_width = float(self.attrs.get("min_grasp_width", "0.001"))
        if isinstance(grasped, str):
            grasped = grasped.lower() == "true"
        pos = float(pos) if pos is not None else 0.0
        return NodeStatus.SUCCESS if (grasped and pos > min_width) else NodeStatus.FAILURE


async def _rclpy_sleep(node: Node, seconds: float) -> None:
    """`asyncio.sleep`-compatible wait that works inside an rclpy action-server
    callback (which doesn't run in an asyncio event loop). Uses a one-shot
    rclpy timer that completes an `rclpy.Future` so the await yields to the
    rclpy executor instead of asyncio's `events.get_running_loop()`.
    """
    import rclpy.task
    fut: rclpy.task.Future = rclpy.task.Future()

    def _on_timer():
        if not fut.done():
            fut.set_result(None)
        timer.cancel()

    timer = node.create_timer(seconds, _on_timer)
    try:
        await fut
    finally:
        timer.cancel()
        node.destroy_timer(timer)


# ── Reactive halt primitives ───────────────────────────────────────────────
#
# The async tree executor blocks on action / sleep / human-prompt futures via
# `await`. Without these helpers, a halt signal (cancel button, lease revoke)
# only takes effect once the underlying future resolves on its own — a 60 s
# wait or a long Meca move means up to that long until halt is acknowledged.
#
# `await_or_halt` and `sleep_or_halt` wrap the await in a 50 ms poll that
# also checks `ctx.is_halted()`, raising `HaltedError` as soon as halt fires.
# Callers translate that into FAILURE + on_skill_failed bookkeeping.
#
# Why poll instead of asyncio.Event: rclpy's action-server callback runs the
# user coroutine via its own executor, not an asyncio loop, so asyncio
# primitives that depend on `get_running_loop()` may not function. Polling on
# top of `_rclpy_sleep` is portable and bounded-latency (50 ms is fine for
# operator-perceptible halt; tighten if a tree needs faster reactivity).

class HaltedError(Exception):
    """Raised mid-await when ExecutionContext.is_halted() becomes true."""


async def await_or_halt(future, ctx: "ExecutionContext", poll_interval: float = 0.05):
    """Await an rclpy.Future, racing against the halt signal.

    Returns the future's result on completion; raises HaltedError if halt
    fires first. The future is NOT cancelled here — caller is responsible
    for any goal-handle teardown via ctx.inflight_goals.
    """
    while not future.done():
        if ctx.is_halted():
            raise HaltedError(ctx.halt_reason)
        await _rclpy_sleep(ctx.ros_node, poll_interval)
    return future.result()


async def sleep_or_halt(
    ctx: "ExecutionContext", seconds: float, poll_interval: float = 0.05
) -> None:
    """Sleep for `seconds`, raising HaltedError early if halt fires."""
    end = time.monotonic() + seconds
    while True:
        if ctx.is_halted():
            raise HaltedError(ctx.halt_reason)
        remaining = end - time.monotonic()
        if remaining <= 0:
            return
        await _rclpy_sleep(ctx.ros_node, min(remaining, poll_interval))


class WaitForDurationNode(SyncNode):
    async def tick(self, bb, ctx):
        # Override SyncNode.tick because we need an async sleep, but mirror
        # its skill-event firing so the dashboard highlights this node for
        # the whole wait duration (otherwise a 5 s wait shows nothing).
        seconds = float(self.attrs.get("seconds", "1.0"))
        ctx.on_skill_started(self.name)
        try:
            # sleep_or_halt polls every 50 ms — a 60 s wait that gets cancelled
            # at second 5 returns within ~50 ms instead of after another 55 s.
            await sleep_or_halt(ctx, seconds)
        except HaltedError:
            ctx.on_skill_failed(self.name)
            return NodeStatus.FAILURE
        except Exception:
            ctx.on_skill_failed(self.name)
            raise
        ctx.on_skill_completed(self.name)
        return NodeStatus.SUCCESS


class SetVelocityOverrideNode(SyncNode):
    def execute(self, bb, ctx):
        scaling = max(0.0, min(1.0, float(self.attrs.get("scaling", "1.0"))))
        out_key = self.attrs.get("velocity_override", "")
        if out_key.startswith("{") and out_key.endswith("}"):
            bb.set(out_key[1:-1], scaling)
        return NodeStatus.SUCCESS


class LogEventNode(SyncNode):
    def execute(self, bb, ctx):
        event_name = self.attrs.get("event_name", "")
        severity = self.attrs.get("severity", "info")
        message = self.attrs.get("message", "")
        tags_str = self.attrs.get("tags", "")
        tags = [t.strip() for t in tags_str.split(";") if t.strip()] or ["human"]
        ctx.publish_log_event(event_name, message, severity, tags=tags)
        return NodeStatus.SUCCESS


class LookupTransformNode(SyncNode):
    def execute(self, bb, ctx):
        pose = PoseStamped()
        pose.header.frame_id = self.attrs.get("target_frame", "world")
        pose.pose.orientation.w = 1.0
        out_key = self.attrs.get("pose", "")
        if out_key.startswith("{") and out_key.endswith("}"):
            bb.set(out_key[1:-1], pose)
        return NodeStatus.SUCCESS


class GetCurrentPoseNode(SyncNode):
    def execute(self, bb, ctx):
        pose = PoseStamped()
        pose.header.frame_id = self.attrs.get("frame_id", "world")
        pose.pose.position.x = 0.3
        pose.pose.position.z = 0.5
        pose.pose.orientation.w = 1.0
        out_key = self.attrs.get("pose", "")
        if out_key.startswith("{") and out_key.endswith("}"):
            bb.set(out_key[1:-1], pose)
        return NodeStatus.SUCCESS


class EmergencyStopNode(SyncNode):
    def execute(self, bb, ctx):
        ctx.log("[EMERGENCY STOP] All motion halted", "error")
        return NodeStatus.SUCCESS


class PublishStaticTFNode(SyncNode):
    def execute(self, bb, ctx):
        return NodeStatus.SUCCESS


# ── Human interaction nodes ──────────────────────────────────────────────────

class HumanNotificationNode(SyncNode):
    def execute(self, bb, ctx):
        ctx.publish_human_prompt(
            prompt_type="notification",
            title=self.attrs.get("title", "Notification"),
            message=self.attrs.get("message", ""),
            node_name=self.name,
        )
        return NodeStatus.SUCCESS


class HumanWarningNode(SyncNode):
    def execute(self, bb, ctx):
        ctx.publish_human_prompt(
            prompt_type="warning",
            title=self.attrs.get("title", "Warning"),
            message=self.attrs.get("message", ""),
            severity=self.attrs.get("severity", "warning"),
            node_name=self.name,
        )
        return NodeStatus.SUCCESS


class HumanBlockingNode(TreeNode):
    """Base for blocking human interaction nodes (Confirm, Input, Task)."""

    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        prompt_id = f"{self.name}_{uuid.uuid4().hex[:8]}"
        timeout = float(self.attrs.get("timeout_sec", "0"))

        # Highlight this node as "running" the whole time we're waiting on
        # the operator. Without this, the dashboard's BT viewer sees no
        # task_state update for human prompts and the node stays gray —
        # so the operator can't tell which step is asking for input.
        ctx.on_skill_started(self.name)

        self._publish_prompt(ctx, prompt_id)

        start = time.monotonic()
        while True:
            # Halt check first so a Cancel during a long human prompt
            # returns immediately instead of waiting for the operator.
            if ctx.is_halted():
                ctx.log(
                    f"Human prompt '{self.name}' halted: {ctx.halt_reason}",
                    "warn",
                )
                ctx.on_skill_failed(self.name)
                return NodeStatus.FAILURE
            response = ctx.check_human_response(prompt_id)
            if response is not None:
                self._store_response(response, bb)
                if response.accepted:
                    ctx.on_skill_completed(self.name)
                    return NodeStatus.SUCCESS
                ctx.on_skill_failed(self.name)
                return NodeStatus.FAILURE
            if timeout > 0 and (time.monotonic() - start) > timeout:
                ctx.log(f"Human prompt '{self.name}' timed out", "warn")
                ctx.on_skill_failed(self.name)
                return NodeStatus.FAILURE
            # Same rclpy-vs-asyncio reason as WaitForDurationNode: bare
            # `asyncio.sleep` raises "no running event loop" because rclpy's
            # action-server execute_callback isn't on an asyncio loop.
            await _rclpy_sleep(ctx.ros_node, 0.1)

    def _publish_prompt(self, ctx, prompt_id):
        raise NotImplementedError

    def _store_response(self, response, bb):
        pass


class HumanConfirmNode(HumanBlockingNode):
    def _publish_prompt(self, ctx, prompt_id):
        ctx.publish_human_prompt(
            prompt_type="confirm",
            title=self.attrs.get("title", "Confirm"),
            message=self.attrs.get("message", ""),
            timeout_sec=float(self.attrs.get("timeout_sec", "0")),
            node_name=self.name,
            prompt_id=prompt_id,
        )

    def _store_response(self, response, bb):
        out = self.attrs.get("confirmed", "")
        if out.startswith("{") and out.endswith("}"):
            bb.set(out[1:-1], response.accepted)


class HumanInputNode(HumanBlockingNode):
    def _publish_prompt(self, ctx, prompt_id):
        choices_str = self.attrs.get("choices", "")
        choices = [c.strip() for c in choices_str.split(";") if c.strip()]
        ctx.publish_human_prompt(
            prompt_type="input",
            title=self.attrs.get("title", "Input"),
            message=self.attrs.get("message", ""),
            input_type=self.attrs.get("input_type", "text"),
            choices=choices,
            default_value=self.attrs.get("default_value", ""),
            timeout_sec=float(self.attrs.get("timeout_sec", "0")),
            node_name=self.name,
            prompt_id=prompt_id,
        )

    def _store_response(self, response, bb):
        out = self.attrs.get("value", "")
        if out.startswith("{") and out.endswith("}"):
            bb.set(out[1:-1], response.value)


class HumanTaskNode(HumanBlockingNode):
    def _publish_prompt(self, ctx, prompt_id):
        ctx.publish_human_prompt(
            prompt_type="task",
            title=self.attrs.get("title", "Task"),
            message=self.attrs.get("message", ""),
            timeout_sec=float(self.attrs.get("timeout_sec", "0")),
            node_name=self.name,
            prompt_id=prompt_id,
        )

    def _store_response(self, response, bb):
        out_completed = self.attrs.get("completed", "")
        if out_completed.startswith("{") and out_completed.endswith("}"):
            bb.set(out_completed[1:-1], response.accepted)
        out_notes = self.attrs.get("notes", "")
        if out_notes.startswith("{") and out_notes.endswith("}"):
            bb.set(out_notes[1:-1], response.value)


# ── ScriptCondition ──────────────────────────────────────────────────────────

class ScriptConditionNode(SyncNode):
    def execute(self, bb, ctx):
        code = self.attrs.get("code", "")
        # Simple evaluation: "var == true", "var == false"
        code = code.strip()
        if "==" in code:
            var, val = [s.strip() for s in code.split("==", 1)]
            bb_val = bb.get(var)
            if val == "true":
                return NodeStatus.SUCCESS if bb_val else NodeStatus.FAILURE
            elif val == "false":
                return NodeStatus.SUCCESS if not bb_val else NodeStatus.FAILURE
            else:
                return NodeStatus.SUCCESS if str(bb_val) == val else NodeStatus.FAILURE
        return NodeStatus.FAILURE


# ── Lease nodes ──────────────────────────────────────────────────────────────

class AcquireLeaseNode(TreeNode):
    """Acquire an exclusive lease on a named resource.

    Attrs:
      resource       — resource_id (required). Free-form string; conventionally
                       a robots.yaml key ("meca500") or an action-server path.
      ttl            — lease TTL in seconds; keepalive renews at ttl/3 (default 10)
      wait           — "true" to block until available (default "false")
      wait_timeout   — max block time for wait=true (default 30)
      output_key     — "{bb_var}" to receive the minted lease_id
      holder_id      — optional caller tag (default: task_id or "bt_executor")
    """

    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        logger = ctx.ros_node.get_logger()
        resource = self.attrs.get("resource", "") or self.attrs.get(
            "resource_id", ""
        )
        if not resource:
            logger.error(f"[{self.name}] missing 'resource' attribute")
            return NodeStatus.FAILURE

        ttl = float(self.attrs.get("ttl", "10.0"))
        wait = self.attrs.get("wait", "false").lower() in ("true", "1", "yes")
        wait_timeout = float(self.attrs.get("wait_timeout", "30.0"))
        holder_id = (
            self.attrs.get("holder_id", "")
            or ctx.task_id
            or "bt_executor"
        )

        client = ctx.get_acquire_lease_client()
        if not client.wait_for_service(timeout_sec=5.0):
            logger.error(
                f"[{self.name}] /skill_server/acquire_lease unavailable"
            )
            return NodeStatus.FAILURE

        req = AcquireLease.Request()
        req.resource_id = resource
        req.holder_id = holder_id
        req.ttl_sec = ttl
        req.wait = wait
        req.wait_timeout_sec = wait_timeout

        try:
            resp = await client.call_async(req)
        except Exception as e:
            logger.error(f"[{self.name}] acquire_lease call failed: {e}")
            return NodeStatus.FAILURE

        if not resp.success:
            logger.warn(f"[{self.name}] acquire_lease refused: {resp.reason}")
            return NodeStatus.FAILURE

        # Publish lease_id onto the blackboard if output_key given.
        output_key = self.attrs.get("output_key", "")
        if output_key.startswith("{") and output_key.endswith("}"):
            bb.set(output_key[1:-1], resp.lease_id)

        ctx.start_lease_keepalive(resp.lease_id, resource, ttl)
        ctx.publish_log_event(
            "lease_acquired",
            f"Acquired lease on '{resource}' (ttl={ttl:.1f}s)",
            "info",
        )
        return NodeStatus.SUCCESS


class ReleaseLeaseNode(TreeNode):
    """Release a previously-acquired lease.

    Attrs:
      lease_id — either literal string or "{bb_var}" reference
      reason   — optional free-form release reason
    """

    async def tick(self, bb: Blackboard, ctx: ExecutionContext) -> NodeStatus:
        logger = ctx.ros_node.get_logger()
        raw = self.attrs.get("lease_id", "")
        lease_id = bb.resolve(raw) if raw else ""
        if not lease_id:
            logger.warn(f"[{self.name}] lease_id empty or unresolved")
            return NodeStatus.FAILURE

        # Stop keepalive first so a pending renew can't race the release.
        resource = ctx.stop_lease_keepalive(lease_id)

        client = ctx.get_release_lease_client()
        if not client.wait_for_service(timeout_sec=2.0):
            logger.warn(
                f"[{self.name}] /skill_server/release_lease unavailable"
            )
            return NodeStatus.FAILURE

        req = ReleaseLease.Request()
        req.lease_id = lease_id
        req.reason = self.attrs.get("reason", "")

        try:
            resp = await client.call_async(req)
        except Exception as e:
            logger.error(f"[{self.name}] release_lease call failed: {e}")
            return NodeStatus.FAILURE

        ctx.publish_log_event(
            "lease_released",
            f"Released lease on '{resource or 'unknown'}'",
            "info",
        )
        return NodeStatus.SUCCESS if resp.success else NodeStatus.FAILURE


# ═══════════════════════════════════════════════════════════════════════════════
# Execution context
# ═══════════════════════════════════════════════════════════════════════════════

class ExecutionContext:
    def __init__(
        self,
        ros_node: Node,
        trees: dict[str, TreeNode],
        task_id: str = "",
        task_name: str = "",
    ):
        self.ros_node = ros_node
        self.trees = trees
        self.task_id = task_id
        # task_name and started_at are populated by BtExecutor before tick;
        # they're echoed into every per-skill TaskState publish so the
        # dashboard's BT viewer can correlate updates with the running task.
        self.task_name = task_name
        self.started_at = ros_node.get_clock().now().to_msg()
        self.completed_skills: list[str] = []
        self.failed_skills: list[str] = []
        self.current_skill = ""
        self.total_action_nodes = 0
        self.cancelled = False

        # Abort-signalling — populated when a lease is revoked mid-tree or
        # when BtExecutor otherwise decides the tree must stop. Mirrored
        # onto the root blackboard so RosActionNode.tick can short-circuit.
        self.root_bb: Optional["Blackboard"] = None
        self.abort_requested = False
        self.abort_reason = ""
        # Unified halt reason for await_or_halt / sleep_or_halt. Populated by
        # whichever path triggers the halt; surfaced through HaltedError so
        # downstream logging can identify the cause.
        self.halt_reason = ""

        # Lease keepalive bookkeeping. Keys are lease_ids; values are the
        # rclpy timer driving the renewer and the resource_id it guards.
        self._lease_timers: dict[str, Any] = {}
        self._lease_resources: dict[str, str] = {}

        # In-flight action goal handles. RosActionNode registers its goal
        # handle on tick and removes on completion; BtExecutor walks this
        # list to hard-cancel when a lease is revoked mid-tree.
        # (ClientGoalHandle isn't hashable, so list + identity removal.)
        self.inflight_goals: list = []

        # Service clients for the lease broker (lazy-created).
        self._acquire_client = None
        self._renew_client = None
        self._release_client = None

        self._log_pub = ros_node.create_publisher(LogEvent, "/skill_server/log_events", 10)
        # task_state is also published by BtExecutor on overall start/end, but
        # those edges are 0% and 100%. Per-skill ticks publish here so the
        # dashboard's BT viewer can highlight the currently-executing node.
        self._task_state_pub = ros_node.create_publisher(
            TaskState, "/skill_server/task_state", 10
        )
        self._human_pub = ros_node.create_publisher(
            HumanPrompt, "/skill_server/human_prompts", 10
        )
        self._human_responses: dict[str, HumanResponse] = {}
        self._human_sub = ros_node.create_subscription(
            HumanResponse, "/skill_server/human_responses", self._on_human_response, 10
        )

    @property
    def progress(self) -> float:
        # SyncNodes / HumanBlockingNodes also bump completed_skills now (so
        # the BT viewer can paint a green trail through every node, not
        # just RosActionNodes). total_action_nodes still only counts
        # RosActionNodes for backwards compatibility, so clamp to [0, 1]
        # to avoid 200%+ progress bars when sync nodes outnumber actions.
        if self.total_action_nodes == 0:
            return 0.0
        return min(1.0, len(self.completed_skills) / self.total_action_nodes)

    def on_skill_started(self, name: str):
        self.current_skill = name
        self.publish_log_event("skill_started", f"Running: {_humanise(name)}", "debug")
        self._publish_task_state("RUNNING")

    def on_skill_completed(self, name: str):
        self.completed_skills.append(name)
        self.publish_log_event("skill_completed", f"Completed: {_humanise(name)}")
        # Stay in RUNNING — overall task hasn't ended yet. The dashboard
        # uses progress + completed_skills to update the BT visualizer
        # between this skill's completion and the next one starting.
        self._publish_task_state("RUNNING")

    def on_skill_failed(self, name: str):
        self.failed_skills.append(name)
        self.publish_log_event("skill_failed", f"Failed: {_humanise(name)}", "error")
        # Still RUNNING here too — a Fallback / RetryUntilSuccessful might
        # consume the failure and continue. BtExecutor publishes the
        # terminal FAILURE/SUCCESS once the whole tree.tick returns.
        self._publish_task_state("RUNNING")

    def _publish_task_state(self, status: str):
        if not self.task_id:
            return  # standalone use (tests) — no task to report against
        msg = TaskState()
        msg.task_id = self.task_id
        msg.task_name = self.task_name
        msg.status = status
        msg.current_skill = self.current_skill
        msg.current_bt_node = self.current_skill
        msg.progress = self.progress
        msg.started_at = self.started_at
        msg.updated_at = self.ros_node.get_clock().now().to_msg()
        # elapsed_sec is computed from started_at -> updated_at
        try:
            import rclpy.time
            msg.elapsed_sec = (
                self.ros_node.get_clock().now().nanoseconds
                - rclpy.time.Time.from_msg(self.started_at).nanoseconds
            ) / 1e9
        except Exception:
            msg.elapsed_sec = 0.0
        msg.completed_skills = list(self.completed_skills)
        msg.failed_skills = list(self.failed_skills)
        msg.error_message = ""
        self._task_state_pub.publish(msg)

    def log(self, message: str, severity: str = "info"):
        self.ros_node.get_logger().info(f"[tree] {message}")

    def publish_log_event(self, event_name: str, message: str,
                          severity: str = "info", tags: list[str] | None = None):
        msg = LogEvent()
        msg.stamp = self.ros_node.get_clock().now().to_msg()
        msg.event_name = event_name
        msg.severity = severity
        msg.message = message
        msg.skill_name = self.current_skill
        msg.tags = tags or ["human"]
        self._log_pub.publish(msg)

    def publish_human_prompt(self, prompt_type: str, title: str, message: str = "",
                             severity: str = "info", node_name: str = "",
                             prompt_id: str = "", input_type: str = "text",
                             choices: list[str] | None = None,
                             default_value: str = "", timeout_sec: float = 0.0):
        msg = HumanPrompt()
        msg.stamp = self.ros_node.get_clock().now().to_msg()
        msg.prompt_id = prompt_id or f"{node_name}_{uuid.uuid4().hex[:8]}"
        msg.prompt_type = prompt_type
        msg.title = title
        msg.message = message
        msg.severity = severity
        msg.input_type = input_type
        msg.choices = choices or []
        msg.default_value = default_value
        msg.timeout_sec = timeout_sec
        msg.bt_node_name = node_name
        self._human_pub.publish(msg)

    def check_human_response(self, prompt_id: str) -> Optional[HumanResponse]:
        return self._human_responses.pop(prompt_id, None)

    def _on_human_response(self, msg: HumanResponse):
        self._human_responses[msg.prompt_id] = msg

    # ── Lease broker clients + keepalive ──────────────────────────────────────

    def get_acquire_lease_client(self):
        if self._acquire_client is None:
            self._acquire_client = self.ros_node.create_client(
                AcquireLease, "/skill_server/acquire_lease"
            )
        return self._acquire_client

    def get_renew_lease_client(self):
        if self._renew_client is None:
            self._renew_client = self.ros_node.create_client(
                RenewLease, "/skill_server/renew_lease"
            )
        return self._renew_client

    def get_release_lease_client(self):
        if self._release_client is None:
            self._release_client = self.ros_node.create_client(
                ReleaseLease, "/skill_server/release_lease"
            )
        return self._release_client

    def start_lease_keepalive(
        self, lease_id: str, resource_id: str, ttl_sec: float
    ):
        """Begin periodic renewal via an rclpy timer.

        The renew period is ttl/3 (minimum 0.5 s). Renew failures are logged
        but do not themselves cancel the tree — the broker will publish a
        'revoked' LeaseEvent when the TTL actually lapses, and BtExecutor's
        lease-event subscriber is the single source of the cancel signal.
        """
        if lease_id in self._lease_timers:
            return  # already running
        self._lease_resources[lease_id] = resource_id
        period = max(ttl_sec / 3.0, 0.5)
        renew_client = self.get_renew_lease_client()
        node = self.ros_node
        logger = node.get_logger()

        def _fire():
            if lease_id not in self._lease_timers:
                return  # stopped while we were scheduled
            req = RenewLease.Request()
            req.lease_id = lease_id
            req.ttl_sec = ttl_sec
            try:
                fut = renew_client.call_async(req)
            except Exception as e:
                logger.warning(
                    f"[keepalive {lease_id[:8]}] call_async failed: {e}"
                )
                return

            def _on_done(f):
                try:
                    resp = f.result()
                except Exception as e:
                    logger.warning(
                        f"[keepalive {lease_id[:8]}] renew exception: {e}"
                    )
                    return
                if resp is None or not resp.success:
                    reason = (
                        getattr(resp, "reason", "no response")
                        if resp is not None
                        else "no response"
                    )
                    logger.warning(
                        f"[keepalive {lease_id[:8]}] renew failed: {reason}"
                    )

            fut.add_done_callback(_on_done)

        timer = node.create_timer(period, _fire)
        self._lease_timers[lease_id] = timer

    def stop_lease_keepalive(self, lease_id: str) -> str:
        """Cancel the keepalive timer for a lease. Returns the resource_id
        (or empty string) for logging."""
        timer = self._lease_timers.pop(lease_id, None)
        resource = self._lease_resources.pop(lease_id, "")
        if timer is not None:
            try:
                timer.cancel()
                self.ros_node.destroy_timer(timer)
            except Exception:
                pass
        return resource

    def active_lease_ids(self) -> list[str]:
        return list(self._lease_timers.keys())

    def resource_for_lease(self, lease_id: str) -> str:
        return self._lease_resources.get(lease_id, "")

    def request_abort(self, reason: str):
        """Flag the tree for cooperative + hard abort. Writes
        _abort_requested / _abort_reason into the root blackboard so
        RosActionNode.tick can short-circuit upcoming actions."""
        self.abort_requested = True
        self.abort_reason = reason or "unspecified"
        self.cancelled = True
        self.halt_reason = self.abort_reason
        if self.root_bb is not None:
            self.root_bb.set("_abort_requested", True)
            self.root_bb.set("_abort_reason", self.abort_reason)

    def is_halted(self) -> bool:
        """Single check covering all halt sources: action-server cancel
        (sets `cancelled`) and lease/abort signalling (sets `abort_requested`).
        Used by `await_or_halt` and `sleep_or_halt` to break out of long
        awaits as soon as the operator clicks Cancel."""
        return self.cancelled or self.abort_requested


UTILITY_REGISTRY: dict[str, type[TreeNode]] = {
    "SetPose": SetPoseNode,
    "ComputePreGraspPose": ComputePreGraspPoseNode,
    "TransformPose": TransformPoseNode,
    "CheckGraspSuccess": CheckGraspSuccessNode,
    "WaitForDuration": WaitForDurationNode,
    "SetVelocityOverride": SetVelocityOverrideNode,
    "LogEvent": LogEventNode,
    "LookupTransform": LookupTransformNode,
    "GetCurrentPose": GetCurrentPoseNode,
    "EmergencyStop": EmergencyStopNode,
    "PublishStaticTF": PublishStaticTFNode,
    "ScriptCondition": ScriptConditionNode,
    # Human interaction
    "HumanNotification": HumanNotificationNode,
    "HumanWarning": HumanWarningNode,
    "HumanConfirm": HumanConfirmNode,
    "HumanInput": HumanInputNode,
    "HumanTask": HumanTaskNode,
    # Resource ownership
    "AcquireLease": AcquireLeaseNode,
    "ReleaseLease": ReleaseLeaseNode,
}

CONTROL_REGISTRY: dict[str, type[TreeNode]] = {
    "Sequence": SequenceNode,
    "Fallback": FallbackNode,
    "ReactiveSequence": SequenceNode,
    "ReactiveFallback": FallbackNode,
    "Parallel": ParallelNode,
    "RetryUntilSuccessful": RetryNode,
    "SubTree": SubTreeNode,
}


# ═══════════════════════════════════════════════════════════════════════════════
# XML parser
# ═══════════════════════════════════════════════════════════════════════════════

class BehaviorTreeParseError(Exception):
    """Raised when a BT XML element references an unknown / ambiguous skill."""


def parse_trees(
    xml_string: str,
    discovery: Optional[Any] = None,
) -> dict[str, TreeNode]:
    """Parse BT.CPP v4 XML into a dict of {tree_id: root_node}.

    Action nodes are resolved against the runtime SkillDiscovery registry,
    keyed by ``(robot_id, bt_tag)``. Resolution rules for ``robot_id``:

    1. If the BT element has ``robot_id="..."`` → look up
       ``(robot_id, bt_tag)`` directly in the discovery.
    2. Else, inherit ``robot_id`` from a parent ``<SubTree>`` /
       ``<BehaviorTree>`` element that set it.
    3. Else, look up by ``bt_tag`` alone — exactly one entry across all
       robots → use it; multiple → :class:`BehaviorTreeParseError`.
    """
    root = ElementTree.fromstring(xml_string)
    trees: dict[str, TreeNode] = {}

    for bt_elem in root.findall("BehaviorTree"):
        tree_id = bt_elem.get("ID", "")
        children = list(bt_elem)
        if children:
            inherited_robot_id = bt_elem.get("robot_id", "")
            trees[tree_id] = _parse_node(children[0], discovery, inherited_robot_id)

    return trees


def _parse_node(
    elem: ElementTree.Element,
    discovery: Optional[Any],
    inherited_robot_id: str,
) -> TreeNode:
    tag = elem.tag
    attrs = dict(elem.attrib)
    name = attrs.pop("name", tag)

    # robot_id flows down the tree: an explicit attribute on the element wins,
    # otherwise the parent (SubTree / BehaviorTree) value is inherited.
    elem_robot_id = attrs.pop("robot_id", inherited_robot_id)

    if tag in CONTROL_REGISTRY:
        node = CONTROL_REGISTRY[tag](name, attrs)
        for child in elem:
            node.children.append(_parse_node(child, discovery, elem_robot_id))
        return node

    if tag in UTILITY_REGISTRY:
        return UTILITY_REGISTRY[tag](name, attrs)

    discovered = _resolve_via_discovery(tag, name, attrs, elem_robot_id, discovery)
    if discovered is not None:
        return discovered

    # Unknown tag — treat as always-success. Real "missing skill" errors
    # land as BehaviorTreeParseError from _resolve_via_discovery when a
    # tag is advertised but its msg pkg can't be imported, or when it's
    # ambiguous across robots.
    return SyncNode(name, attrs)


def _resolve_via_discovery(
    tag: str,
    name: str,
    attrs: dict,
    robot_id: str,
    discovery: Optional[Any],
) -> Optional[TreeNode]:
    """Resolve an action node from the runtime SkillDiscovery registry.

    Returns ``None`` if discovery isn't wired in (e.g. unit tests that
    construct trees directly) or if the tag is genuinely unknown; the
    caller treats those as always-success utility nodes. Raises
    :class:`BehaviorTreeParseError` for ambiguous matches or import
    failures so misconfigured deployments fail loud at parse time rather
    than silently at first tick.
    """
    if discovery is None:
        return None

    if robot_id:
        entry = discovery.get(robot_id, tag)
    else:
        candidates = discovery.get_by_tag(tag)
        if len(candidates) > 1:
            robots = sorted({e.robot_id or "<unset>" for e in candidates})
            raise BehaviorTreeParseError(
                f"BT tag {tag!r} is advertised by multiple robots ({robots}); "
                f"add robot_id=\"...\" to the action node or to its parent "
                f"<SubTree>/<BehaviorTree> element"
            )
        entry = candidates[0] if candidates else None

    if entry is None:
        return None

    if entry.import_error:
        raise BehaviorTreeParseError(
            f"Skill {tag!r} (robot_id={entry.robot_id!r}) advertises action "
            f"type {entry.action_type_str!r} but the Python msg pkg is not "
            f"installed in this env: {entry.import_error}"
        )

    server_name = attrs.pop("server_name", entry.server_name)

    post_process = (
        _POST_PROCESSORS.get(entry.post_process) if entry.post_process else None
    )

    return RosActionNode(
        name=name,
        attrs=attrs,
        action_type=entry.action_type,
        server_name=server_name,
        input_map=entry.inputs,
        output_map=entry.outputs,
        goal_defaults=entry.defaults,
        post_process=post_process,
    )


def get_main_tree_name(xml_string: str) -> str:
    root = ElementTree.fromstring(xml_string)
    return root.get("main_tree_to_execute", "")


def count_action_nodes(node: TreeNode) -> int:
    count = 1 if isinstance(node, RosActionNode) else 0
    for child in node.children:
        count += count_action_nodes(child)
    return count


# ═══════════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _detect_object_post(result, attrs, bb):
    """Extract best_object_pose from DetectObject detections."""
    out_key = attrs.get("best_object_pose", "")
    if out_key.startswith("{") and out_key.endswith("}") and result.detections:
        best = max(result.detections, key=lambda d: d.confidence)
        bb.set(out_key[1:-1], best.pose)


_POST_PROCESSORS = {
    "_detect_object_post": _detect_object_post,
}


def _humanise(s: str) -> str:
    return s.replace("_", " ").capitalize() if s else s


def _set_goal_field(goal, field_name: str, value):
    """Set a field on a ROS2 goal message, with type coercion."""
    import array as _array

    current = getattr(goal, field_name, None)
    if current is None:
        return
    if isinstance(current, bool):
        if isinstance(value, str):
            value = value.lower() in ("true", "1", "yes")
        setattr(goal, field_name, bool(value))
    elif isinstance(current, float):
        setattr(goal, field_name, float(value))
    elif isinstance(current, int):
        setattr(goal, field_name, int(float(value)))
    elif isinstance(current, str):
        setattr(goal, field_name, str(value))
    elif isinstance(current, (list, _array.array)):
        # ROS2 float64[] fields are array.array('d'), not list
        if isinstance(value, str):
            value = value.strip("[] ")
            if value:
                parts = [float(p.strip()) for p in value.replace(";", ",").split(",")]
            else:
                parts = []
        elif isinstance(value, (list, _array.array)):
            parts = list(value)
        else:
            parts = [value]
        if isinstance(current, _array.array):
            setattr(goal, field_name, _array.array(current.typecode, parts))
        else:
            setattr(goal, field_name, parts)
    else:
        # PoseStamped or other complex types — pass through
        setattr(goal, field_name, value)
