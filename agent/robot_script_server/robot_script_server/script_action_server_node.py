#!/usr/bin/env python3
"""Agent-host scripted action server.

Lives on the agent's machine alongside bt_executor_mcp. When an MCP client
registers a script via /script_action_server_<session>/register_script,
this node spins up a /scripts_<session>/<name> action server (RunScript
type) and republishes the per-session SkillManifest so the orchestrator's
SkillDiscovery picks the new skill up like any hardware proxy.

Trust model: agent-authored code runs HERE, on the agent's machine, never
on the orchestrator. The orchestrator only dispatches to ROS endpoints; it
does not import or exec the script source.

Phase 5 ships the ``bt_xml`` dispatcher: stored BT XML is invoked by
round-tripping through the orchestrator's
``/skill_server/execute_behavior_tree`` action. ``action_sequence`` and
``python_callable`` are deferred behind ``--allow-python-scripts`` gating.

Naming: each run gets a short uuid suffix so multiple agents on the same
DDS graph never collide.
"""

from __future__ import annotations

import json
import os
import re
import threading
import uuid
from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_skill_advertise import SkillAdvertiser
from robot_skills_msgs.action import ExecuteBehaviorTree, RunScript
from robot_skills_msgs.msg import (
    KeyValue,
    SkillAdvertisement,
    SkillDescription,
)
from robot_skills_msgs.srv import RegisterScript


def _pascal_case(name: str) -> str:
    return "".join(p.capitalize() for p in re.split(r"[_\-\s]+", name) if p)


def _kvs_to_dict(items) -> Dict[str, str]:
    return {kv.key: kv.value for kv in items}


class ScriptActionServer(Node):
    def __init__(self, session_id: Optional[str] = None) -> None:
        super().__init__("script_action_server")

        self.declare_parameter("persist_dir", "~/.ros2_robot_skills/scripts_local")
        self.declare_parameter("session_id", session_id or "")

        sid = self.get_parameter("session_id").get_parameter_value().string_value
        self._session_id = sid or uuid.uuid4().hex[:8]

        persist_dir = os.path.expanduser(
            self.get_parameter("persist_dir").get_parameter_value().string_value
        )
        self._persist_dir = Path(persist_dir)
        self._persist_dir.mkdir(parents=True, exist_ok=True)

        self._lock = threading.RLock()
        self._scripts: Dict[str, dict] = {}
        self._action_servers: Dict[str, ActionServer] = {}

        # Internal client used by the bt_xml dispatcher to round-trip back
        # through the orchestrator's BT executor. The orchestrator is *the*
        # tree runner; we just hand it the stored XML.
        self._bt_client = ActionClient(
            self, ExecuteBehaviorTree, "/skill_server/execute_behavior_tree"
        )

        self._register_srv = self.create_service(
            RegisterScript,
            f"/script_action_server_{self._session_id}/register_script",
            self._handle_register_script,
        )

        # SkillAdvertiser publishes on `~/skills` — i.e.
        # /script_action_server/skills (no session segment in topic — DDS
        # doesn't distinguish, but action server names below carry the
        # session id so cross-agent collision is impossible).
        self._advertiser = SkillAdvertiser(self, [])

        self._load_persisted()

        self.get_logger().info(
            f"script_action_server up (session={self._session_id}, "
            f"persist={self._persist_dir})"
        )

    # ── Registration ────────────────────────────────────────────────────────

    def _handle_register_script(
        self, request: RegisterScript.Request, response: RegisterScript.Response
    ) -> RegisterScript.Response:
        name = request.name.strip()
        if not re.match(r"^[a-zA-Z_][a-zA-Z0-9_]*$", name):
            response.success = False
            response.message = (
                f"name {name!r} must match [a-zA-Z_][a-zA-Z0-9_]*"
            )
            return response

        if request.kind not in ("bt_xml",):
            response.success = False
            response.message = (
                f"unsupported kind {request.kind!r}; phase 5 ships bt_xml only"
            )
            return response

        with self._lock:
            self._scripts[name] = {
                "name": name,
                "kind": request.kind,
                "source": request.source,
                "input_schema_json": request.input_schema_json,
                "output_schema_json": request.output_schema_json,
                "description": request.description,
            }
            self._spawn_action_server(name)
            self._persist_script(self._scripts[name])
            self._republish_manifest()

        response.success = True
        response.message = (
            f"registered {name!r} on /scripts_{self._session_id}/{name}"
        )
        self.get_logger().info(response.message)
        return response

    # ── Action server pool ──────────────────────────────────────────────────

    def _spawn_action_server(self, name: str) -> None:
        action_path = f"/scripts_{self._session_id}/{name}"
        if name in self._action_servers:
            return
        self._action_servers[name] = ActionServer(
            self,
            RunScript,
            action_path,
            execute_callback=lambda gh, n=name: self._execute_script(n, gh),
        )

    def _execute_script(self, name: str, goal_handle):
        with self._lock:
            entry = self._scripts.get(name)
        if entry is None:
            goal_handle.abort()
            res = RunScript.Result()
            res.success = False
            res.message = f"unknown script {name!r}"
            return res

        if entry["kind"] == "bt_xml":
            return self._execute_bt_xml(name, entry, goal_handle)

        # Should never reach: register_script gates kinds.
        goal_handle.abort()
        res = RunScript.Result()
        res.success = False
        res.message = f"dispatcher not implemented for kind={entry['kind']!r}"
        return res

    def _execute_bt_xml(self, name: str, entry: dict, goal_handle):
        """Round-trip the stored BT XML through the orchestrator."""
        if not self._bt_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            res = RunScript.Result()
            res.success = False
            res.message = "/skill_server/execute_behavior_tree not available"
            return res

        bt_goal = ExecuteBehaviorTree.Goal()
        bt_goal.tree_xml = entry["source"]
        bt_goal.tree_name = f"script_{name}"

        send_future = self._bt_client.send_goal_async(bt_goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_h = send_future.result()
        if goal_h is None or not goal_h.accepted:
            goal_handle.abort()
            res = RunScript.Result()
            res.success = False
            res.message = "BT executor rejected goal"
            return res

        result_future = goal_h.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        bt_result = result_future.result().result

        res = RunScript.Result()
        res.success = bool(bt_result.success)
        res.message = bt_result.message
        if res.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return res

    # ── Manifest publication ────────────────────────────────────────────────

    def _republish_manifest(self) -> None:
        ads: list[SkillAdvertisement] = []
        with self._lock:
            for name, entry in sorted(self._scripts.items()):
                ad = SkillAdvertisement()
                ad.description = SkillDescription(
                    name=name,
                    display_name=entry["description"] or name,
                    description=entry["description"],
                    version="1.0.0",
                    robot_id=f"agent.{self._session_id}",
                    category="compound",
                    tags=["agent-script", entry["kind"]],
                    action_server_name=f"/scripts_{self._session_id}/{name}",
                    action_type="robot_skills_msgs/action/RunScript",
                    parameters_schema=entry["input_schema_json"],
                )
                ad.bt_tag = _pascal_case(name)
                ads.append(ad)
        self._advertiser.set_skills(ads)

    # ── Persistence ─────────────────────────────────────────────────────────

    def _persist_script(self, entry: dict) -> None:
        path = self._persist_dir / f"{entry['name']}.yaml"
        with open(path, "w") as f:
            yaml.safe_dump(entry, f)

    def _load_persisted(self) -> None:
        if not self._persist_dir.exists():
            return
        for path in sorted(self._persist_dir.glob("*.yaml")):
            try:
                with open(path) as f:
                    entry = yaml.safe_load(f)
                if not entry or "name" not in entry:
                    continue
                self._scripts[entry["name"]] = entry
                self._spawn_action_server(entry["name"])
            except Exception as e:
                self.get_logger().warning(
                    f"failed to load persisted script {path.name}: {e}"
                )
        if self._scripts:
            self._republish_manifest()


def main(args=None):
    rclpy.init(args=args)
    node = ScriptActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
