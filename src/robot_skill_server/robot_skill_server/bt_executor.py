"""
BtExecutor - Executes BehaviorTree XML as a ROS2 Action Server.

Parses BT.CPP v4 XML and executes trees directly in Python using ROS2
action clients. No C++ dependency, no subprocess, no BT.CPP library.

Action: /skill_server/execute_behavior_tree (ExecuteBehaviorTree.action)
Topic:  /skill_server/task_state (TaskState.msg)
Topic:  /skill_server/log_events (LogEvent.msg)
Topic:  /skill_server/active_bt_xml (String, transient_local)
"""

from __future__ import annotations

import asyncio
import json
import os
import time
import uuid
from pathlib import Path
from typing import Optional

import rclpy
import rclpy.time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from std_msgs.msg import String

from robot_skills_msgs.action import ExecuteBehaviorTree
from robot_skills_msgs.msg import DryRunStatus, LeaseEvent, LogEvent, TaskState
from robot_skills_msgs.srv import ApproveDryRun, ReleaseLease

from robot_skill_server.tree_executor import (
    Blackboard,
    ExecutionContext,
    NodeStatus,
    count_action_nodes,
    get_main_tree_name,
    parse_trees,
)


SIM_NAMESPACE_PREFIX = "/sim"


def _humanise(s: str) -> str:
    return s.replace("_", " ").capitalize() if s else s


class BtExecutor(Node):
    """Executes BT XML directly using the Python tree executor."""

    def __init__(self, skill_discovery=None):
        super().__init__("bt_executor")
        # Phase 3: when present, parse_trees consults this for
        # (robot_id, bt_tag) → (action_type, server_name, port_map) lookups
        # before falling back to the static ACTION_REGISTRY. None preserves
        # legacy behaviour for tests / standalone use.
        self._skill_discovery = skill_discovery

        callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            ExecuteBehaviorTree,
            "/skill_server/execute_behavior_tree",
            execute_callback=self._execute_bt,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        self._task_state_pub = self.create_publisher(
            TaskState, "/skill_server/task_state", 10
        )
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._active_bt_xml_pub = self.create_publisher(
            String, "/skill_server/active_bt_xml", latched_qos
        )
        self._log_event_pub = self.create_publisher(
            LogEvent, "/skill_server/log_events", 10
        )
        self._dryrun_status_pub = self.create_publisher(
            DryRunStatus, "/skill_server/dryrun_status", latched_qos
        )

        # Approval gate state for SIM_THEN_REAL goals. Exactly one goal can be
        # parked at a time — _execute_bt is async-serialised by the goal
        # callback. The service handler completes the future to release the
        # await in the running coroutine.
        self._pending_approval_task_id: Optional[str] = None
        self._pending_approval_future: Optional[asyncio.Future] = None
        self._approve_service = self.create_service(
            ApproveDryRun,
            "/skill_server/approve_dry_run",
            self._on_approve_dry_run,
            callback_group=callback_group,
        )

        # Listen for lease lifecycle; 'revoked' on a lease we hold triggers
        # hard-cancel of the executing tree.
        self._lease_event_sub = self.create_subscription(
            LeaseEvent,
            "/skill_server/lease_events",
            self._on_lease_event,
            20,
        )

        self._current_task_id: Optional[str] = None
        self._current_task_name: Optional[str] = None
        self._current_ctx: Optional[ExecutionContext] = None
        self._task_started_at = None
        self._total_tasks_executed = 0
        self._last_task_result = "IDLE"

        # Default tick rate for the heartbeat publisher. The action goal's
        # `tick_rate_hz` field overrides this per-tree. 5 Hz keeps the
        # dashboard responsive (200 ms updates) while staying well below
        # any rosbridge / WebSocket bandwidth concerns.
        self.declare_parameter("default_tick_rate_hz", 5.0)
        # Periodic state-publish loop. The tree itself runs async/await and
        # publishes state on transitions; this timer ensures the dashboard
        # also gets a heartbeat at a fixed cadence even when a long-running
        # action is in flight (e.g. a 5 s Meca move). Created once and
        # gated by `_current_task_id` — no overhead while idle.
        self._heartbeat_timer = None
        self._heartbeat_period = 1.0 / max(
            0.1, self.get_parameter("default_tick_rate_hz").value
        )
        self._install_heartbeat_timer(self._heartbeat_period)

        # ── Available trees (scanned from disk, published as latched JSON) ──
        latched_trees_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._available_trees_pub = self.create_publisher(
            String, "/skill_server/available_trees", latched_trees_qos
        )
        self._trees_dir = self._find_trees_dir()
        self._last_tree_scan: dict[str, float] = {}  # filename → mtime
        self._publish_available_trees()
        # Re-scan every 2 seconds for new/changed files
        self.create_timer(2.0, self._check_trees_changed)

        self._diag_updater = Updater(self)
        self._diag_updater.setHardwareID("bt_executor")
        self._diag_updater.add("executor_status", self._produce_diagnostics)

        self.get_logger().info(
            "BtExecutor started on /skill_server/execute_behavior_tree"
        )

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received BT execution request: '{goal_request.tree_name}'"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("BT execution cancel requested")
        if self._current_ctx:
            self._current_ctx.cancelled = True
            # `halt_reason` is read by HaltedError so logs can identify why
            # the tree halted (operator cancel vs. lease revoke vs. ...).
            if not self._current_ctx.halt_reason:
                self._current_ctx.halt_reason = "operator cancel"
        return CancelResponse.ACCEPT

    async def _execute_bt(self, goal_handle) -> ExecuteBehaviorTree.Result:
        goal = goal_handle.request
        result = ExecuteBehaviorTree.Result()
        task_id = "t" + uuid.uuid4().hex[:8]
        self._current_task_id = task_id
        self._current_task_name = goal.tree_name
        self._task_started_at = self.get_clock().now().to_msg()
        overall_start = time.time()

        target_mode = int(getattr(goal, "target_mode", 0))
        is_sim_only = target_mode == ExecuteBehaviorTree.Goal.MODE_SIM
        is_sim_then_real = target_mode == ExecuteBehaviorTree.Goal.MODE_SIM_THEN_REAL

        # Honour the goal's tick_rate_hz if non-zero; falls back to the node
        # default. Clamp to a sensible range so a goal can't accidentally DoS
        # rosbridge with a 1000 Hz heartbeat or stall the dashboard at 0.01 Hz.
        requested_hz = float(getattr(goal, "tick_rate_hz", 0.0) or 0.0)
        default_hz = float(self.get_parameter("default_tick_rate_hz").value)
        effective_hz = max(0.5, min(20.0, requested_hz if requested_hz > 0 else default_hz))
        new_period = 1.0 / effective_hz
        if abs(new_period - self._heartbeat_period) > 1e-3:
            self._install_heartbeat_timer(new_period)

        mode_label = {
            ExecuteBehaviorTree.Goal.MODE_REAL: "real",
            ExecuteBehaviorTree.Goal.MODE_SIM: "sim",
            ExecuteBehaviorTree.Goal.MODE_SIM_THEN_REAL: "sim_then_real",
        }.get(target_mode, "real")
        self.get_logger().info(
            f"Executing BT '{goal.tree_name}' (id={task_id}) "
            f"mode={mode_label} heartbeat={effective_hz:.1f}Hz"
        )

        # Active XML displayed on the dashboard reflects the tree about to run
        # next — for SIM_THEN_REAL this is initially the sim variant.
        sim_xml = goal.sim_tree_xml if (is_sim_only or is_sim_then_real) and goal.sim_tree_xml else goal.tree_xml
        if is_sim_only or is_sim_then_real:
            self._active_bt_xml_pub.publish(String(data=sim_xml))
        else:
            self._active_bt_xml_pub.publish(String(data=goal.tree_xml))

        self._publish_log_event(
            "task_started",
            f"Task '{goal.tree_name}' started (mode={mode_label})",
            task_id=task_id,
        )

        sim_status_str = ""
        real_status_str = ""

        try:
            # ── Sim phase ────────────────────────────────────────────────
            if is_sim_only or is_sim_then_real:
                self._publish_task_state(
                    task_id=task_id,
                    task_name=goal.tree_name,
                    status="RUNNING",
                )
                phase_start = time.time()
                sim_status_str, sim_ctx = await self._run_phase(
                    xml=sim_xml,
                    task_id=task_id,
                    task_name=goal.tree_name,
                    sim_namespace_prefix=SIM_NAMESPACE_PREFIX,
                )
                sim_elapsed = time.time() - phase_start

                # SIM_ONLY: terminate after the sim phase.
                if is_sim_only:
                    return self._finalise_goal(
                        goal_handle=goal_handle,
                        result=result,
                        task_id=task_id,
                        tree_name=goal.tree_name,
                        ctx=sim_ctx,
                        final_status=sim_status_str,
                        elapsed=time.time() - overall_start,
                        sim_status=sim_status_str,
                        real_status="",
                    )

                # SIM_THEN_REAL: publish dry-run status and wait for approval.
                self._publish_dryrun_status(
                    task_id=task_id,
                    tree_name=goal.tree_name,
                    sim_status=sim_status_str,
                    sim_elapsed=sim_elapsed,
                    ctx=sim_ctx,
                )

                if sim_status_str != "SUCCESS":
                    # Sim failed — never promote to real. Surface the sim
                    # outcome as the overall result.
                    return self._finalise_goal(
                        goal_handle=goal_handle,
                        result=result,
                        task_id=task_id,
                        tree_name=goal.tree_name,
                        ctx=sim_ctx,
                        final_status=sim_status_str,
                        elapsed=time.time() - overall_start,
                        sim_status=sim_status_str,
                        real_status="SKIPPED",
                    )

                approved = await self._await_approval(task_id, goal_handle)
                if not approved:
                    # Operator rejected (or cancelled). Sim succeeded; we
                    # report SUCCESS overall but flag real as SKIPPED.
                    return self._finalise_goal(
                        goal_handle=goal_handle,
                        result=result,
                        task_id=task_id,
                        tree_name=goal.tree_name,
                        ctx=sim_ctx,
                        final_status="SUCCESS",
                        elapsed=time.time() - overall_start,
                        sim_status=sim_status_str,
                        real_status="SKIPPED",
                    )

            # ── Real phase ────────────────────────────────────────────────
            self._active_bt_xml_pub.publish(String(data=goal.tree_xml))
            self._publish_task_state(
                task_id=task_id,
                task_name=goal.tree_name,
                status="RUNNING",
            )
            real_status_str, real_ctx = await self._run_phase(
                xml=goal.tree_xml,
                task_id=task_id,
                task_name=goal.tree_name,
                sim_namespace_prefix="",
            )

            return self._finalise_goal(
                goal_handle=goal_handle,
                result=result,
                task_id=task_id,
                tree_name=goal.tree_name,
                ctx=real_ctx,
                final_status=real_status_str,
                elapsed=time.time() - overall_start,
                sim_status=sim_status_str if is_sim_then_real else "",
                real_status=real_status_str if is_sim_then_real else "",
            )

        except Exception as e:
            import traceback
            if self._current_ctx is not None:
                self._release_surviving_leases(self._current_ctx)
            self._current_ctx = None
            elapsed = time.time() - overall_start
            self.get_logger().error(
                f"Tree execution error: {type(e).__name__}: {e}\n"
                f"{traceback.format_exc()}"
            )
            result.success = False
            result.final_status = "ERROR"
            result.message = f"Tree execution error: {e}"
            result.total_execution_time_sec = elapsed
            result.sim_final_status = sim_status_str
            result.real_final_status = real_status_str
            self._publish_log_event(
                "task_error", str(e), severity="error", task_id=task_id
            )
            self._current_task_id = None
            self._current_task_name = None
            self._clear_pending_approval()
            self._publish_task_state(
                task_id=task_id,
                task_name=goal.tree_name,
                status="FAILURE",
                error_message=str(e),
            )
            goal_handle.abort(result)
            return result

    async def _run_phase(
        self,
        xml: str,
        task_id: str,
        task_name: str,
        sim_namespace_prefix: str,
    ) -> tuple[str, ExecutionContext]:
        """Parse and tick a single tree to completion.

        Returns ``(final_status_str, ctx)`` where final_status_str is one of
        "SUCCESS" / "FAILURE" / "HALTED". Raises on parse / setup errors so
        the caller can fold them into the goal-level ERROR path.
        """
        trees = parse_trees(
            xml,
            discovery=self._skill_discovery,
            sim_namespace_prefix=sim_namespace_prefix,
        )
        main_tree_name = get_main_tree_name(xml)
        main_tree = trees.get(main_tree_name)
        if not main_tree:
            raise ValueError(
                f"Main tree '{main_tree_name}' not found in XML. "
                f"Available: {list(trees.keys())}"
            )

        ctx = ExecutionContext(
            self, trees, task_id=task_id, task_name=task_name,
        )
        ctx.started_at = self._task_started_at
        ctx.total_action_nodes = count_action_nodes(main_tree)
        self._current_ctx = ctx

        bb = Blackboard()
        ctx.root_bb = bb
        try:
            tree_status = await main_tree.tick(bb, ctx)
        finally:
            self._release_surviving_leases(ctx)

        if ctx.cancelled:
            await main_tree.halt(ctx)
            return "HALTED", ctx
        if tree_status == NodeStatus.SUCCESS:
            return "SUCCESS", ctx
        return "FAILURE", ctx

    def _finalise_goal(
        self,
        goal_handle,
        result: ExecuteBehaviorTree.Result,
        task_id: str,
        tree_name: str,
        ctx: ExecutionContext,
        final_status: str,
        elapsed: float,
        sim_status: str,
        real_status: str,
    ) -> ExecuteBehaviorTree.Result:
        success = final_status == "SUCCESS"
        result.success = success
        result.final_status = final_status
        result.total_execution_time_sec = elapsed
        result.sim_final_status = sim_status
        result.real_final_status = real_status
        suffix = ""
        if sim_status or real_status:
            suffix = f" [sim={sim_status or '-'} real={real_status or '-'}]"
        result.message = (
            f"BT '{tree_name}' completed: {final_status} in {elapsed:.2f}s{suffix}"
        )

        self._total_tasks_executed += 1
        self._last_task_result = final_status
        self.get_logger().info(result.message)

        self._current_task_id = None
        self._current_task_name = None
        self._current_ctx = None
        self._clear_pending_approval()

        self._publish_task_state(
            task_id=task_id,
            task_name=tree_name,
            status=final_status,
            current_node=ctx.current_skill if not success else "",
            progress=1.0 if success else ctx.progress,
            completed_skills=ctx.completed_skills,
            failed_skills=ctx.failed_skills,
            error_message=(
                ctx.failed_skills[-1] if ctx.failed_skills else ""
            ) if not success else "",
        )

        if final_status == "HALTED" and goal_handle.is_cancel_requested:
            self._publish_log_event(
                "task_cancelled",
                f"Task '{tree_name}' cancelled after {elapsed:.1f}s",
                severity="warn",
                task_id=task_id,
            )
            goal_handle.canceled(result)
        elif success:
            self._publish_log_event(
                "task_completed",
                f"Task '{tree_name}' completed in {elapsed:.1f}s{suffix}",
                task_id=task_id,
            )
            goal_handle.succeed(result)
        else:
            self._publish_log_event(
                "task_failed",
                f"Task '{tree_name}' failed after {elapsed:.1f}s{suffix}",
                severity="error",
                task_id=task_id,
            )
            goal_handle.abort(result)
        return result

    # ── Dry-run approval gate ───────────────────────────────────────────────

    async def _await_approval(self, task_id: str, goal_handle) -> bool:
        """Block until /skill_server/approve_dry_run is called for ``task_id``,
        or the goal is cancelled. Returns True if approved, False otherwise.
        """
        loop = asyncio.get_event_loop()
        future: asyncio.Future = loop.create_future()
        self._pending_approval_task_id = task_id
        self._pending_approval_future = future

        self._publish_task_state(
            task_id=task_id,
            task_name=self._current_task_name or "",
            status="AWAITING_APPROVAL",
        )
        self._publish_log_event(
            "dryrun_awaiting_approval",
            f"Sim phase succeeded; awaiting /skill_server/approve_dry_run for {task_id}",
            task_id=task_id,
        )

        # Poll for cancel: if the goal is cancelled while we're parked,
        # treat it as rejection so the coroutine unwinds cleanly.
        while not future.done():
            if goal_handle.is_cancel_requested:
                self._clear_pending_approval()
                return False
            try:
                return await asyncio.wait_for(asyncio.shield(future), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            except asyncio.CancelledError:
                self._clear_pending_approval()
                raise
        return future.result()

    def _on_approve_dry_run(self, request, response):
        """Service handler for /skill_server/approve_dry_run."""
        task_id = self._pending_approval_task_id
        future = self._pending_approval_future
        if task_id is None or future is None or future.done():
            response.accepted = False
            response.message = "no goal awaiting approval"
            return response
        if request.task_id and request.task_id != task_id:
            response.accepted = False
            response.message = (
                f"task_id {request.task_id!r} does not match the parked goal {task_id!r}"
            )
            return response

        # Resolving on the executor's loop is unsafe from a sync service
        # callback — schedule via call_soon_threadsafe.
        try:
            loop = future.get_loop()
            loop.call_soon_threadsafe(future.set_result, bool(request.approve))
        except Exception as e:
            response.accepted = False
            response.message = f"failed to release gate: {e}"
            return response

        verb = "approved" if request.approve else "rejected"
        self._publish_log_event(
            "dryrun_decision",
            f"Dry-run {verb} for {task_id}"
            + (f" — {request.reason}" if request.reason else ""),
            task_id=task_id,
        )
        response.accepted = True
        response.message = verb
        return response

    def _publish_dryrun_status(
        self,
        task_id: str,
        tree_name: str,
        sim_status: str,
        sim_elapsed: float,
        ctx: ExecutionContext,
    ):
        msg = DryRunStatus()
        msg.task_id = task_id
        msg.tree_name = tree_name
        msg.sim_status = sim_status
        msg.message = (
            ctx.failed_skills[-1] if ctx.failed_skills and sim_status != "SUCCESS"
            else ""
        )
        msg.sim_duration_sec = sim_elapsed
        msg.completed_skills = list(ctx.completed_skills)
        msg.failed_skills = list(ctx.failed_skills)
        msg.published_at = self.get_clock().now().to_msg()
        self._dryrun_status_pub.publish(msg)

    def _clear_pending_approval(self):
        if self._pending_approval_task_id is None and self._pending_approval_future is None:
            return
        self._pending_approval_task_id = None
        self._pending_approval_future = None
        # Latch a "no goal pending" status so subscribers stop showing the
        # approval modal once the gate has been released or the goal ended.
        cleared = DryRunStatus()
        cleared.task_id = ""
        cleared.published_at = self.get_clock().now().to_msg()
        self._dryrun_status_pub.publish(cleared)

    # ── Lease integration ────────────────────────────────────────────────────

    def _on_lease_event(self, msg: LeaseEvent):
        """React to lease lifecycle messages.

        Only 'revoked' events matter here: if the revoked lease belongs to
        the currently-running tree, trigger hard-cancel. Other events
        (acquired/renewed/released) are informational — the broker is the
        source of truth.
        """
        if msg.event != "revoked":
            return
        ctx = self._current_ctx
        if ctx is None:
            return
        if msg.lease_id not in ctx.active_lease_ids():
            return
        reason = (
            f"lease_revoked: {msg.resource_id} ({msg.reason or 'unknown'})"
        )
        self.get_logger().warning(
            f"Lease {msg.lease_id[:8]} on '{msg.resource_id}' revoked mid-tree "
            f"— hard-cancelling (reason={msg.reason})"
        )
        ctx.request_abort(reason)
        # Interrupt any in-flight action so the RosActionNode await returns.
        self._cancel_inflight(ctx)

    def _cancel_inflight(self, ctx: ExecutionContext):
        """Best-effort cancel of every goal the tree currently has in flight.
        cancel_goal_async is non-blocking; the action server responds with
        CANCELED, the await in RosActionNode.tick resumes with a failed
        result, and the tree unwinds via FAILURE propagation."""
        inflight = getattr(ctx, "inflight_goals", None)
        if not inflight:
            return
        for gh in list(inflight):
            try:
                gh.cancel_goal_async()
            except Exception as e:
                self.get_logger().debug(
                    f"cancel_goal_async on {gh}: {type(e).__name__}: {e}"
                )

    def _release_surviving_leases(self, ctx: ExecutionContext):
        """Release any leases the tree still holds at exit. Prevents zombie
        leases when a tree is cancelled, fails, or raises mid-execution."""
        client = ctx.get_release_lease_client()
        for lease_id in list(ctx.active_lease_ids()):
            resource = ctx.stop_lease_keepalive(lease_id)
            try:
                req = ReleaseLease.Request()
                req.lease_id = lease_id
                req.reason = "tree_ended"
                client.call_async(req)  # fire-and-forget
                self.get_logger().info(
                    f"Released surviving lease {lease_id[:8]} on '{resource}'"
                )
            except Exception as e:
                self.get_logger().warning(
                    f"Failed to release surviving lease {lease_id[:8]}: {e}"
                )

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _install_heartbeat_timer(self, period_s: float) -> None:
        """(Re)create the periodic state-publish timer at the given period."""
        if self._heartbeat_timer is not None:
            self._heartbeat_timer.destroy()
        self._heartbeat_period = period_s
        self._heartbeat_timer = self.create_timer(period_s, self._tick_heartbeat)

    def _tick_heartbeat(self):
        """Fixed-rate publish of the live TaskState while a tree is running.
        No-op when idle. Mirrors a BT.CPP-style tickRoot cadence externally
        even though the underlying tree execution is async/await."""
        ctx = self._current_ctx
        if ctx is None or self._current_task_id is None:
            return
        # Don't republish terminal states — those are emitted exactly once by
        # _execute_bt at the end. Heartbeating after that would clobber the
        # final SUCCESS/FAILURE message with a stale RUNNING.
        if ctx.cancelled:
            return
        self._publish_task_state(
            task_id=self._current_task_id,
            task_name=self._current_task_name or "",
            status="RUNNING",
            current_node=ctx.current_skill,
            progress=ctx.progress,
            completed_skills=list(ctx.completed_skills),
            failed_skills=list(ctx.failed_skills),
        )

    def _publish_task_state(
        self,
        task_id: str,
        task_name: str,
        status: str,
        current_node: str = "",
        progress: float = 0.0,
        completed_skills: list[str] | None = None,
        failed_skills: list[str] | None = None,
        error_message: str = "",
    ):
        now = self.get_clock().now().to_msg()
        msg = TaskState()
        msg.task_id = task_id
        msg.task_name = task_name
        msg.status = status
        msg.current_skill = current_node
        msg.current_bt_node = current_node
        msg.progress = progress
        msg.started_at = self._task_started_at or now
        msg.updated_at = now
        msg.elapsed_sec = (
            (self.get_clock().now().nanoseconds
             - rclpy.time.Time.from_msg(msg.started_at).nanoseconds) / 1e9
            if self._task_started_at else 0.0
        )
        msg.completed_skills = completed_skills or []
        msg.failed_skills = failed_skills or []
        msg.error_message = error_message
        self._task_state_pub.publish(msg)

    def _publish_log_event(
        self,
        event_name: str,
        message: str,
        severity: str = "info",
        task_id: str = "",
    ):
        log_msg = LogEvent()
        log_msg.stamp = self.get_clock().now().to_msg()
        log_msg.event_name = event_name
        log_msg.severity = severity
        log_msg.message = message
        log_msg.task_id = task_id
        log_msg.tags = ["human"]
        self._log_event_pub.publish(log_msg)

    # ── Tree discovery ─────────────────────────────────────────────────────

    def _find_trees_dir(self) -> Optional[str]:
        """Find the behavior trees directory. Prefer source (volume-mounted)
        over installed share so new files appear immediately."""
        # Source directory (volume-mounted, always up to date)
        for candidate in [
            "/home/ws/src/robot_behaviors/trees",
            os.path.join(os.path.dirname(__file__), "..", "..", "..",
                         "robot_behaviors", "trees"),
        ]:
            if os.path.isdir(candidate):
                return os.path.realpath(candidate)
        # Fallback: installed share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            d = os.path.join(
                get_package_share_directory("robot_behaviors"), "trees"
            )
            if os.path.isdir(d):
                return d
        except Exception:
            pass
        return None

    def _scan_trees(self) -> list[dict]:
        """Scan the trees directory and return metadata + XML for each tree."""
        if not self._trees_dir or not os.path.isdir(self._trees_dir):
            return []

        trees = []
        for f in sorted(Path(self._trees_dir).glob("*.xml")):
            try:
                xml_text = f.read_text()
                tree_name = get_main_tree_name(xml_text)
                # Derive a human-readable label from filename
                label = f.stem.replace("_", " ").title()
                trees.append({
                    "name": tree_name or f.stem,
                    "label": label,
                    "filename": f.name,
                    "xml": xml_text,
                })
            except Exception as e:
                self.get_logger().warning(f"Failed to parse {f.name}: {e}")
        return trees

    def _publish_available_trees(self):
        """Publish the current tree list as JSON."""
        trees = self._scan_trees()
        msg = String()
        msg.data = json.dumps(trees)
        self._available_trees_pub.publish(msg)
        # Track mtimes for change detection
        if self._trees_dir:
            self._last_tree_scan = {
                f.name: f.stat().st_mtime
                for f in Path(self._trees_dir).glob("*.xml")
            }
        self.get_logger().info(
            f"Published {len(trees)} available trees from {self._trees_dir}"
        )

    def _check_trees_changed(self):
        """Re-publish if any tree files were added, removed, or modified."""
        if not self._trees_dir or not os.path.isdir(self._trees_dir):
            return
        current = {
            f.name: f.stat().st_mtime
            for f in Path(self._trees_dir).glob("*.xml")
        }
        if current != self._last_tree_scan:
            self._publish_available_trees()

    def _produce_diagnostics(self, stat):
        if self._current_task_id:
            stat.summary(
                DiagnosticStatus.OK,
                f"Running task {self._current_task_id}",
            )
        elif self._last_task_result == "FAILURE":
            stat.summary(DiagnosticStatus.WARN, "Last task failed")
        else:
            stat.summary(DiagnosticStatus.OK, "Idle")

        stat.add("current_task", self._current_task_id or "none")
        stat.add("total_executed", str(self._total_tasks_executed))
        stat.add("last_result", self._last_task_result)
        return stat
