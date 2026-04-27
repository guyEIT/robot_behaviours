"""
Unit tests for TaskComposer BT XML generation.

Tests exercise pure-logic methods without a live ROS2 system by mocking the
rclpy.node.Node base class. Run via: colcon test --packages-select robot_skill_server
"""

import sys
import types
import unittest
from unittest.mock import MagicMock, patch
from xml.etree import ElementTree as ET


# ---------------------------------------------------------------------------
# Minimal stubs so the module can be imported without a running ROS2 context
# ---------------------------------------------------------------------------
def _stub_ros_modules():
    for mod_name in [
        "rclpy", "rclpy.node", "rclpy.time",
        "diagnostic_updater", "diagnostic_msgs", "diagnostic_msgs.msg",
        "robot_skills_msgs", "robot_skills_msgs.msg",
        "robot_skills_msgs.srv",
    ]:
        if mod_name not in sys.modules:
            sys.modules[mod_name] = types.ModuleType(mod_name)

    # rclpy.node.Node stub
    class _Node:
        def __init__(self, *a, **kw):
            pass
        def get_logger(self):
            return MagicMock()
        def get_clock(self):
            return MagicMock()
        def declare_parameter(self, *a, **kw):
            pass
        def get_parameter(self, *a, **kw):
            m = MagicMock()
            m.value = 10.0
            return m
        def create_publisher(self, *a, **kw):
            return MagicMock()

    sys.modules["rclpy.node"].Node = _Node
    sys.modules["rclpy.time"].Time = MagicMock

    # diagnostic_updater.Updater stub
    sys.modules["diagnostic_updater"].Updater = MagicMock

    # diagnostic_msgs.msg stub
    sys.modules["diagnostic_msgs.msg"].DiagnosticStatus = MagicMock

    # SkillDescription and TaskStep stubs
    class _SkillDescription:
        def __init__(self):
            self.name = ""
            self.display_name = ""
            self.description = ""
            self.category = ""
            self.tags = []
            self.preconditions = []
            self.postconditions = []
            self.effects = []
            self.constraints = []
            self.parameters_schema = ""
            self.is_compound = False
            self.bt_xml = ""
            self.component_skills = []
            self.pddl_action = ""
            self.action_server_name = ""
            self.action_type = ""

    class _TaskStep:
        def __init__(self):
            self.skill_name = ""
            self.parameters_json = ""
            self.input_blackboard_keys = []
            self.output_blackboard_keys = []
            self.retry_on_failure = False
            self.max_retries = 0
            self.condition_expression = ""
            self.description = ""

    msg_mod = sys.modules["robot_skills_msgs.msg"]
    msg_mod.SkillDescription = _SkillDescription
    msg_mod.TaskStep = _TaskStep

    # Service stubs
    srv_mod = sys.modules["robot_skills_msgs.srv"]
    for svc_name in [
        "ComposeTask", "RegisterSkill", "GetSkillDescriptions",
        "RegisterCompoundSkill", "ValidatePlan",
    ]:
        setattr(srv_mod, svc_name, MagicMock())

    # std_msgs stub
    if "std_msgs" not in sys.modules:
        sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    if "std_msgs.msg" not in sys.modules:
        sys.modules["std_msgs.msg"] = types.ModuleType("std_msgs.msg")
    sys.modules["std_msgs.msg"].String = MagicMock

    sys.modules["robot_skills_msgs"].msg = msg_mod


_stub_ros_modules()

# Now import the module under test
from robot_skill_server.task_composer import TaskComposer  # noqa: E402


class TestPrecondExpressionMapping(unittest.TestCase):
    """Test that precondition strings map to correct BT expressions."""

    def _make_composer(self):
        with patch("rclpy.node.Node.__init__", return_value=None):
            composer = TaskComposer.__new__(TaskComposer)
            composer._logger = MagicMock()
            return composer

    def test_known_preconditions_generate_correct_expressions(self):
        from robot_skills_msgs.msg import SkillDescription, TaskStep
        composer = self._make_composer()

        skill = SkillDescription()
        skill.name = "gripper_control"
        skill.preconditions = ["object_detected", "gripper_open"]

        step = TaskStep()
        step.skill_name = "gripper_control"
        step.description = ""
        step.parameters_json = ""
        step.input_blackboard_keys = []
        step.output_blackboard_keys = []
        step.retry_on_failure = False
        step.max_retries = 0

        warnings = []
        seq = composer._make_precondition_sequence(step, skill, "GripperControlNode", {}, warnings)

        self.assertEqual(seq.tag, "Sequence")
        conditions = [c for c in seq if c.tag == "ScriptCondition"]
        self.assertEqual(len(conditions), 2)
        self.assertEqual(conditions[0].get("code"), "object_detected == true")
        self.assertEqual(conditions[1].get("code"), "gripper_state == 'open'")
        self.assertEqual(warnings, [])

    def test_unknown_precondition_warns_and_passes(self):
        from robot_skills_msgs.msg import SkillDescription, TaskStep
        composer = self._make_composer()

        skill = SkillDescription()
        skill.name = "custom_skill"
        skill.preconditions = ["unknown_precond"]

        step = TaskStep()
        step.skill_name = "custom_skill"
        step.description = ""
        step.parameters_json = ""
        step.input_blackboard_keys = []
        step.output_blackboard_keys = []

        warnings = []
        seq = composer._make_precondition_sequence(step, skill, "CustomNode", {}, warnings)

        conditions = [c for c in seq if c.tag == "ScriptCondition"]
        self.assertEqual(len(conditions), 1)
        # Should fail-open (always pass)
        self.assertEqual(conditions[0].get("code"), "true")
        # Must warn
        self.assertTrue(len(warnings) > 0)
        self.assertIn("unknown_precond", warnings[0])

    def test_more_than_max_preconditions_warns_and_truncates(self):
        from robot_skills_msgs.msg import SkillDescription, TaskStep
        composer = self._make_composer()

        skill = SkillDescription()
        skill.name = "big_skill"
        skill.preconditions = [
            "object_detected", "gripper_open", "at_home", "object_grasped"
        ]  # 4 > MAX_PRECONDITIONS (3)

        step = TaskStep()
        step.skill_name = "big_skill"
        step.description = ""
        step.parameters_json = ""
        step.input_blackboard_keys = []
        step.output_blackboard_keys = []

        warnings = []
        seq = composer._make_precondition_sequence(step, skill, "BigNode", {}, warnings)

        conditions = [c for c in seq if c.tag == "ScriptCondition"]
        self.assertLessEqual(len(conditions), TaskComposer._MAX_PRECONDITIONS)
        self.assertTrue(any("4" in w or "precondition" in w.lower() for w in warnings))

    def test_no_preconditions_generates_no_conditions(self):
        from robot_skills_msgs.msg import SkillDescription, TaskStep
        composer = self._make_composer()

        skill = SkillDescription()
        skill.name = "simple_skill"
        skill.preconditions = []

        step = TaskStep()
        step.skill_name = "simple_skill"
        step.description = ""
        step.parameters_json = ""
        step.input_blackboard_keys = []
        step.output_blackboard_keys = []

        warnings = []
        seq = composer._make_precondition_sequence(step, skill, "SimpleNode", {}, warnings)

        conditions = [c for c in seq if c.tag == "ScriptCondition"]
        self.assertEqual(len(conditions), 0)
        self.assertEqual(warnings, [])


class TestSetBtParams(unittest.TestCase):
    """Test BT node parameter attribute setting."""

    def test_bool_params_serialised_as_lowercase(self):
        elem = ET.Element("TestNode")
        TaskComposer._set_bt_params(elem, {"flag": True, "other": False}, MagicMock())
        self.assertEqual(elem.get("flag"), "true")
        self.assertEqual(elem.get("other"), "false")

    def test_numeric_params_serialised_as_string(self):
        elem = ET.Element("TestNode")
        TaskComposer._set_bt_params(elem, {"speed": 0.5, "count": 3}, MagicMock())
        self.assertEqual(elem.get("speed"), "0.5")
        self.assertEqual(elem.get("count"), "3")

    def test_string_params_passed_through(self):
        elem = ET.Element("TestNode")
        TaskComposer._set_bt_params(elem, {"config": "home"}, MagicMock())
        self.assertEqual(elem.get("config"), "home")


if __name__ == "__main__":
    unittest.main()
