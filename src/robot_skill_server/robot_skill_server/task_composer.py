"""
TaskComposer - Generates BehaviorTree.CPP v4 XML from skill step lists.

Allows agents to compose new compound tasks by specifying:
  - An ordered list of skill steps
  - Parameters for each step
  - Retry logic, conditions, blackboard data flow

The generated XML is valid BT.CPP v4 format and can be:
  - Executed directly via ExecuteBehaviorTree action
  - Registered as a named compound skill via RegisterCompoundSkill
  - Saved to disk as a reusable tree definition

Services:
  - /skill_server/compose_task  (ComposeTask.srv)
"""

import json
import re
from typing import List, Optional
from xml.dom import minidom
from xml.etree import ElementTree as ET

import rclpy
from rclpy.node import Node

from robot_skills_msgs.msg import SkillDescription, TaskStep
from robot_skills_msgs.srv import ComposeTask

from .skill_registry import SkillRegistry


# Maps skill category to a sensible BT node type name
SKILL_NODE_TYPE_MAP = {
    "move_to_named_config": "MoveToNamedConfig",
    "move_to_cartesian_pose": "MoveToCartesianPose",
    "gripper_control": "GripperControl",
    "detect_object": "DetectObject",
    "execute_behavior_tree": "ExecuteBehaviorTree",
}


class TaskComposer(Node):
    """
    Composes BehaviorTree XML from skill step lists.

    Agent workflow:
      1. Call GetSkillDescriptions to discover available skills
      2. Build a TaskStep[] list with skill names and parameters
      3. Call ComposeTask → receive BT XML
      4. Optionally call RegisterCompoundSkill to save as named skill
      5. Call ExecuteBehaviorTree to run the task
    """

    def __init__(self, skill_registry: SkillRegistry):
        super().__init__("task_composer")
        self._registry = skill_registry

        self._compose_srv = self.create_service(
            ComposeTask,
            "/skill_server/compose_task",
            self._handle_compose_task,
        )

        self.get_logger().info("TaskComposer started on /skill_server/compose_task")

    # ── Service handler ───────────────────────────────────────────────────────

    def _handle_compose_task(
        self,
        request: ComposeTask.Request,
        response: ComposeTask.Response,
    ) -> ComposeTask.Response:
        """Generate BT XML from a list of skill steps."""
        if not request.steps:
            response.success = False
            response.message = "No steps provided"
            return response

        warnings = []

        try:
            bt_xml = self._generate_bt_xml(
                task_name=request.task_name or "ComposedTask",
                steps=list(request.steps),
                sequential=request.sequential,
                add_precondition_checks=request.add_precondition_checks,
                warnings=warnings,
            )
            response.success = True
            response.bt_xml = bt_xml
            response.warnings = warnings
            response.message = (
                f"Generated BT with {len(request.steps)} step(s)"
                + (f" ({len(warnings)} warning(s))" if warnings else "")
            )
            self.get_logger().info(
                f"Composed task '{request.task_name}' with {len(request.steps)} steps"
            )

        except Exception as e:
            response.success = False
            response.message = f"XML generation failed: {e}"
            self.get_logger().error(f"TaskComposer error: {e}")

        return response

    # ── XML generation ────────────────────────────────────────────────────────

    def _generate_bt_xml(
        self,
        task_name: str,
        steps: List[TaskStep],
        sequential: bool = True,
        add_precondition_checks: bool = False,
        warnings: Optional[List[str]] = None,
    ) -> str:
        """Generate valid BehaviorTree.CPP v4 XML."""
        if warnings is None:
            warnings = []

        # Sanitize tree name for XML ID
        tree_id = re.sub(r"[^A-Za-z0-9_]", "_", task_name)

        root = ET.Element("root")
        root.set("BTCPP_format", "4")
        root.set("main_tree_to_execute", tree_id)

        bt_elem = ET.SubElement(root, "BehaviorTree")
        bt_elem.set("ID", tree_id)

        # Top-level container: Sequence or Parallel
        container_tag = "Sequence" if sequential else "Parallel"
        container = ET.SubElement(bt_elem, container_tag)
        container.set("name", task_name)

        for step in steps:
            skill_desc = self._registry.get_skill(step.skill_name)
            if skill_desc is None:
                warnings.append(
                    f"Skill '{step.skill_name}' not found in registry "
                    f"(will try to use as-is in XML)"
                )

            # Determine the BT node XML tag name
            # Prefer registry info, fall back to CamelCase conversion
            if skill_desc and skill_desc.is_compound:
                # Compound skills use SubTree nodes
                step_elem = self._make_compound_step_element(
                    step, skill_desc, add_precondition_checks
                )
            else:
                step_elem = self._make_atom_step_element(
                    step, skill_desc, add_precondition_checks, warnings
                )

            container.append(step_elem)

        return self._pretty_xml(root)

    def _make_atom_step_element(
        self,
        step: TaskStep,
        skill_desc: Optional[SkillDescription],
        add_precondition_checks: bool,
        warnings: List[str],
    ) -> ET.Element:
        """Create a BT XML element for a primitive skill step."""
        # Derive BT node type name from skill name
        node_type = SKILL_NODE_TYPE_MAP.get(step.skill_name)
        if node_type is None:
            # Convert snake_case to CamelCase
            node_type = "".join(
                word.capitalize() for word in step.skill_name.split("_")
            )
            warnings.append(
                f"No explicit BT node type for '{step.skill_name}', "
                f"using derived name '{node_type}'"
            )

        # Parse parameters JSON
        params = {}
        if step.parameters_json:
            try:
                params = json.loads(step.parameters_json)
            except json.JSONDecodeError as e:
                warnings.append(
                    f"Step '{step.skill_name}': invalid parameters_json: {e}"
                )

        # Optionally wrap in retry decorator
        if step.retry_on_failure and step.max_retries > 0:
            wrapper = ET.Element("RetryUntilSuccessful")
            wrapper.set("name", f"retry_{step.skill_name}")
            wrapper.set("num_attempts", str(step.max_retries))

            if add_precondition_checks and skill_desc:
                inner = self._make_precondition_sequence(step, skill_desc, node_type, params)
                wrapper.append(inner)
            else:
                inner = ET.SubElement(wrapper, node_type)
                inner.set("name", step.description or step.skill_name)
                self._set_bt_params(inner, params, step)
            return wrapper

        if add_precondition_checks and skill_desc and skill_desc.preconditions:
            return self._make_precondition_sequence(step, skill_desc, node_type, params)

        elem = ET.Element(node_type)
        elem.set("name", step.description or step.skill_name)
        self._set_bt_params(elem, params, step)
        return elem

    def _make_compound_step_element(
        self,
        step: TaskStep,
        skill_desc: SkillDescription,
        add_precondition_checks: bool,
    ) -> ET.Element:
        """Create a SubTree element for a compound skill."""
        elem = ET.Element("SubTree")
        elem.set("ID", re.sub(r"[^A-Za-z0-9_]", "_", skill_desc.name))
        elem.set("name", step.description or step.skill_name)
        return elem

    def _make_precondition_sequence(
        self,
        step: TaskStep,
        skill_desc: SkillDescription,
        node_type: str,
        params: dict,
    ) -> ET.Element:
        """Wrap skill in a Sequence with precondition checks."""
        seq = ET.Element("Sequence")
        seq.set("name", f"prechecked_{step.skill_name}")

        # Add a simple Script node to check preconditions
        # In production, these would be real Condition nodes
        for precond in skill_desc.preconditions[:3]:  # limit to 3 for readability
            check = ET.SubElement(seq, "ScriptCondition")
            check.set("name", f"check_{precond}")
            check.set("code", f"'{precond}' == 'satisfied'")  # placeholder

        action = ET.SubElement(seq, node_type)
        action.set("name", step.description or step.skill_name)
        self._set_bt_params(action, params, step)
        return seq

    @staticmethod
    def _set_bt_params(elem: ET.Element, params: dict, step: TaskStep):
        """Set BT node parameters from parsed JSON and blackboard keys."""
        for key, value in params.items():
            if isinstance(value, bool):
                elem.set(key, "true" if value else "false")
            elif isinstance(value, (int, float)):
                elem.set(key, str(value))
            elif isinstance(value, str):
                elem.set(key, value)

        # Wire blackboard keys
        for bb_key in step.input_blackboard_keys:
            if ":" in bb_key:
                port, key = bb_key.split(":", 1)
                elem.set(port, "{" + key + "}")

        for bb_key in step.output_blackboard_keys:
            if ":" in bb_key:
                port, key = bb_key.split(":", 1)
                elem.set(port, "{" + key + "}")

    @staticmethod
    def _pretty_xml(root: ET.Element) -> str:
        """Return indented XML string."""
        raw = ET.tostring(root, encoding="unicode")
        parsed = minidom.parseString(raw)
        return parsed.toprettyxml(indent="  ", newl="\n")
