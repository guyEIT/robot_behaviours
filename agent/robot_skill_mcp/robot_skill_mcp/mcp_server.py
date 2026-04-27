#!/usr/bin/env python3
"""MCP server wrapping the BT executor's ROS2 endpoints.

Exposes the skill server's discovery, execution, and authoring capabilities
as Model Context Protocol tools so an LLM client (Claude Code, etc.) can
drive the robot via stdio JSON-RPC.

Usage:
  ros2 run robot_skill_mcp bt_executor_mcp
  python -m robot_skill_mcp.mcp_server

Wire into Claude Code via .mcp.json:
  {
    "mcpServers": {
      "bt-executor": {
        "command": "pixi",
        "args": ["run", "-e", "<env-with-mcp>", "bt-mcp"],
        "cwd": "/home/eit/robot_behaviours"
      }
    }
  }
"""

from __future__ import annotations

import sys
from typing import Any

from mcp.server.fastmcp import FastMCP

from robot_skill_mcp.mcp_ros_bridge import MCPRosBridge


_bridge: MCPRosBridge | None = None
mcp = FastMCP("bt-executor")


def _b() -> MCPRosBridge:
    if _bridge is None:
        raise RuntimeError("ROS bridge not initialized")
    return _bridge


@mcp.tool()
def list_trees() -> list[dict[str, Any]]:
    """List behavior trees auto-discovered from src/robot_behaviors/trees/.

    Returns one entry per tree: name, label, filename. Use get_tree_xml
    to fetch the full XML for a specific tree before executing.
    """
    return _b().list_trees()


@mcp.tool()
def get_tree_xml(name: str) -> dict[str, Any]:
    """Fetch the BT.CPP v4 XML for a discovered tree by name."""
    xml = _b().get_tree_xml(name)
    if xml is None:
        return {"found": False, "message": f"tree '{name}' not in discovered set"}
    return {"found": True, "name": name, "xml": xml}


@mcp.tool()
async def list_skills(
    filter_categories: list[str] | None = None,
    filter_tags: list[str] | None = None,
    include_compounds: bool = True,
    include_pddl: bool = False,
) -> dict[str, Any]:
    """List skills registered with the SkillRegistry.

    Calls /skill_server/get_skill_descriptions. Use category/tag filters to
    narrow results; set include_pddl=True for PDDL action fragments.
    """
    return await _b().call_get_skill_descriptions(
        filter_categories or [],
        filter_tags or [],
        include_compounds,
        include_pddl,
    )


@mcp.tool()
def get_active_tree_xml() -> dict[str, Any]:
    """Return the XML of the tree currently being executed (latched topic)."""
    xml = _b().get_active_tree_xml()
    return {"xml": xml, "is_executing": bool(xml)}


@mcp.tool()
def get_task_status() -> dict[str, Any]:
    """Return the latest TaskState heartbeat from the BT executor.

    Empty until the first tree starts. Status values: IDLE, RUNNING, SUCCESS,
    FAILURE, CANCELLED. Progress is 0.0–1.0.
    """
    snap = _b().get_task_status()
    if snap is None:
        return {"available": False, "message": "no TaskState received yet"}
    return {"available": True, **snap}


@mcp.tool()
def get_recent_log_events(limit: int = 50) -> list[dict[str, Any]]:
    """Tail the /skill_server/log_events stream (ring buffer, max 200)."""
    return _b().get_recent_log_events(limit)


@mcp.tool()
async def execute_tree(
    name: str | None = None,
    tree_xml: str | None = None,
    tree_name: str | None = None,
    tick_rate_hz: float = 5.0,
    wait_for_completion: bool = True,
    timeout_sec: float = 300.0,
    target_mode: str = "real",
    sim_tree_xml: str | None = None,
) -> dict[str, Any]:
    """Submit a behavior tree to /skill_server/execute_behavior_tree.

    Provide exactly one of:
      - name: look up an auto-discovered tree's XML by name (preferred)
      - tree_xml: pass full BT.CPP v4 XML directly (e.g. compose_task output)

    tick_rate_hz controls the TaskState heartbeat publish rate (clamped
    server-side to [0.5, 20]). It does NOT control tree execution cadence.

    target_mode is one of:
      - "real" (default): run only against real action servers.
      - "sim": run only against /sim/* action servers.
      - "sim_then_real": run sim first; on success the orchestrator parks in
        AWAITING_APPROVAL and publishes /skill_server/dryrun_status. Call
        get_dryrun_status() to inspect the outcome and approve_dry_run() to
        release the gate.

    sim_tree_xml is an optional override BT XML for the sim phase only —
    useful when the dry-run variant elides multi-hour steps. Empty falls back
    to tree_xml.

    When wait_for_completion=True (default), blocks until the action
    completes or timeout_sec elapses. When False, returns immediately after
    the goal is accepted; poll get_task_status for progress and call
    cancel_execution to abort.
    """
    if (name is None) == (tree_xml is None):
        return {
            "accepted": False,
            "message": "provide exactly one of `name` or `tree_xml`",
        }
    if name is not None:
        xml = _b().get_tree_xml(name)
        if xml is None:
            return {
                "accepted": False,
                "message": f"tree '{name}' not in discovered set; call list_trees",
            }
        tree_xml = xml
        if tree_name is None:
            tree_name = name
    if tree_name is None:
        tree_name = "mcp_tree"
    mode_map = {"real": 0, "sim": 1, "sim_then_real": 2}
    mode = mode_map.get(target_mode.lower())
    if mode is None:
        return {
            "accepted": False,
            "message": f"target_mode must be one of {sorted(mode_map)}; got {target_mode!r}",
        }
    return await _b().execute_tree(
        tree_xml=tree_xml,
        tree_name=tree_name,
        tick_rate_hz=tick_rate_hz,
        wait_for_completion=wait_for_completion,
        timeout_sec=timeout_sec if wait_for_completion else None,
        target_mode=mode,
        sim_tree_xml=sim_tree_xml or "",
    )


@mcp.tool()
def get_dryrun_status() -> dict[str, Any] | None:
    """Return the latest /skill_server/dryrun_status, or None if no goal is
    awaiting approval. Latched topic — populates immediately on subscribe.
    """
    return _b().get_dryrun_status()


@mcp.tool()
async def approve_dry_run(
    approve: bool,
    task_id: str = "",
    reason: str = "",
) -> dict[str, Any]:
    """Release the SIM_THEN_REAL approval gate.

    approve=true → orchestrator runs the real phase. approve=false → real
    phase is skipped (overall result remains the sim's SUCCESS).

    task_id is optional: empty matches whichever goal is currently parked.
    Pass it to assert that you're approving a specific run.
    """
    return await _b().approve_dry_run(
        approve=approve, task_id=task_id, reason=reason,
    )


@mcp.tool()
async def cancel_execution() -> dict[str, Any]:
    """Cancel the currently-running behavior tree.

    Halt is reactive: the executor polls for cancellation every 50 ms during
    long awaits, so cancellation typically lands within ~50 ms even on
    long-running skills. Returns {cancelled, reason}.
    """
    return await _b().cancel_execution()


@mcp.tool()
async def compose_task(
    task_name: str,
    steps: list[dict[str, Any]],
    task_description: str = "",
    sequential: bool = True,
    add_precondition_checks: bool = False,
) -> dict[str, Any]:
    """Generate BT XML from an ordered list of skill steps.

    Each step is a dict with keys mirroring robot_skills_msgs/TaskStep:
      skill_name (str, required), parameters_json (str, JSON object),
      input_blackboard_keys (list[str]), output_blackboard_keys (list[str]),
      retry_on_failure (bool), max_retries (int),
      condition_expression (str), description (str), robot_id (str).

    Returns {success, message, bt_xml, warnings}. The XML is not executed —
    review it then call execute_tree(tree_xml=...) to run it, or
    register_compound_skill to persist it as a reusable skill.
    """
    return await _b().call_compose_task(
        task_name=task_name,
        task_description=task_description,
        steps=steps,
        sequential=sequential,
        add_precondition_checks=add_precondition_checks,
    )


@mcp.tool()
async def register_compound_skill(
    name: str,
    bt_xml: str,
    description: str = "",
    tags: list[str] | None = None,
    persist: bool = True,
) -> dict[str, Any]:
    """Register a compound skill in the SkillRegistry.

    Persistent, vetted shared skill. Lives on the orchestrator and persists
    in ~/.ros2_robot_skills/compound_skills/{name}.yaml. Use this for
    long-lived skills that everyone on the lab DDS graph should see.
    For ad-hoc agent-authored skills, prefer register_script (lives on the
    agent host, blast radius limited to this machine).
    """
    return await _b().call_register_compound_skill(
        name=name,
        bt_xml=bt_xml,
        description=description,
        tags=tags or [],
        persist=persist,
    )


@mcp.tool()
async def register_script(
    name: str,
    source: str,
    description: str = "",
    input_schema: str = "{}",
    output_schema: str = "{}",
    kind: str = "bt_xml",
) -> dict[str, Any]:
    """Register an ad-hoc skill on the agent's local script_action_server.

    Agent-authored code runs on the agent's machine, never on the
    orchestrator. The script is exposed as /scripts_<session>/<name> and
    advertised so the orchestrator's SkillDiscovery picks it up like any
    hardware proxy. The skill is alive only as long as this MCP session.

    kind="bt_xml" (currently the only supported kind): source is BT XML
    that the orchestrator's tree executor will run when the script's
    action is invoked.
    """
    return await _b().call_register_script(
        name=name,
        kind=kind,
        source=source,
        input_schema=input_schema,
        output_schema=output_schema,
        description=description,
    )


def main() -> None:
    global _bridge
    try:
        _bridge = MCPRosBridge()
    except Exception as exc:
        print(f"failed to initialize ROS bridge: {exc}", file=sys.stderr)
        raise
    print("MCP server ready on stdio (bt-executor)", file=sys.stderr)
    try:
        mcp.run()
    finally:
        if _bridge is not None:
            _bridge.shutdown()


if __name__ == "__main__":
    main()
