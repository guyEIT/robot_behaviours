"""
Unit tests for SkillRegistry persistence and skill management.

Tests exercise pure-logic methods without a live ROS2 system.
Run via: colcon test --packages-select robot_skill_server
"""

import sys
import tempfile
import types
import unittest
import yaml
from pathlib import Path
from unittest.mock import MagicMock, patch


# ---------------------------------------------------------------------------
# Minimal stubs — must match test_task_composer.py stubs
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

    class _Node:
        def __init__(self, *a, **kw):
            pass
        def get_logger(self):
            return MagicMock()
        def get_clock(self):
            clk = MagicMock()
            clk.now.return_value.to_msg.return_value = MagicMock()
            return clk
        def declare_parameter(self, *a, **kw):
            pass
        def get_parameter(self, name, *a, **kw):
            m = MagicMock()
            m.value = ""
            return m
        def create_publisher(self, *a, **kw):
            return MagicMock()
        def create_service(self, *a, **kw):
            return MagicMock()
        def create_timer(self, *a, **kw):
            return MagicMock()

    sys.modules["rclpy.node"].Node = _Node
    sys.modules["rclpy.time"].Time = MagicMock
    sys.modules["diagnostic_updater"].Updater = MagicMock
    sys.modules["diagnostic_msgs.msg"].DiagnosticStatus = MagicMock

    class _SkillDescription:
        def __init__(self):
            self.name = ""
            self.display_name = ""
            self.description = ""
            self.version = ""
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
            self.created_at = None
            self.updated_at = None

    sys.modules["robot_skills_msgs.msg"].SkillDescription = _SkillDescription

    # Service stubs
    srv_mod = sys.modules["robot_skills_msgs.srv"]
    for svc_name in ["RegisterSkill", "GetSkillDescriptions", "RegisterCompoundSkill", "ComposeTask"]:
        setattr(srv_mod, svc_name, MagicMock())

    # std_msgs stub
    if "std_msgs" not in sys.modules:
        sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    if "std_msgs.msg" not in sys.modules:
        sys.modules["std_msgs.msg"] = types.ModuleType("std_msgs.msg")
    sys.modules["std_msgs.msg"].String = MagicMock


_stub_ros_modules()

from robot_skill_server.skill_registry import SkillRegistry  # noqa: E402


def _make_registry(persist_dir: str = "") -> SkillRegistry:
    """Create a SkillRegistry with mocked ROS2 infrastructure."""
    with patch("rclpy.node.Node.__init__", return_value=None):
        reg = SkillRegistry.__new__(SkillRegistry)
        reg._skills = {}
        reg._lock = __import__("threading").RLock()
        reg._logger = MagicMock()
        mock_clock = MagicMock()
        mock_clock.now.return_value.to_msg.return_value = MagicMock()
        reg._clock = mock_clock
        reg.get_logger = MagicMock(return_value=MagicMock())
        reg.get_clock = MagicMock(return_value=mock_clock)
        reg._persist_dir = Path(persist_dir) if persist_dir else Path(tempfile.mkdtemp())
        reg._skills_pub = MagicMock()
        return reg


class TestSkillRegistryPersistence(unittest.TestCase):
    """Test compound skill persistence round-trip."""

    def test_load_valid_yaml_file(self):
        """A well-formed YAML skill file is loaded successfully."""
        with tempfile.TemporaryDirectory() as tmpdir:
            skill_data = {
                "name": "pick_and_place",
                "display_name": "Pick and Place",
                "description": "Pick an object and place it",
                "category": "compound",
                "tags": ["manipulation"],
                "preconditions": ["object_detected"],
                "postconditions": ["object_placed"],
                "effects": [],
                "constraints": [],
                "parameters_schema": "{}",
                "component_skills": ["detect_object", "gripper_control"],
                "bt_xml": "<BehaviorTree/>",
            }
            skill_file = Path(tmpdir) / "pick_and_place.yaml"
            skill_file.write_text(yaml.dump(skill_data))

            reg = _make_registry(tmpdir)
            reg._load_persisted_skills()

            self.assertIn("pick_and_place", reg._skills)
            loaded = reg._skills["pick_and_place"]
            self.assertEqual(loaded.display_name, "Pick and Place")
            self.assertEqual(loaded.bt_xml, "<BehaviorTree/>")

    def test_corrupt_yaml_file_is_skipped_with_warning(self):
        """A file with invalid YAML is skipped and a warning is logged."""
        with tempfile.TemporaryDirectory() as tmpdir:
            bad_file = Path(tmpdir) / "corrupt.yaml"
            bad_file.write_text("this: is: not: valid: yaml: [")

            reg = _make_registry(tmpdir)
            reg._load_persisted_skills()

            self.assertEqual(len(reg._skills), 0)
            reg.get_logger().warning.assert_called()

    def test_yaml_missing_required_name_field_is_skipped(self):
        """A YAML file without 'name' key raises KeyError and is skipped."""
        with tempfile.TemporaryDirectory() as tmpdir:
            bad_data = {"display_name": "No Name Field"}
            bad_file = Path(tmpdir) / "no_name.yaml"
            bad_file.write_text(yaml.dump(bad_data))

            reg = _make_registry(tmpdir)
            reg._load_persisted_skills()

            self.assertEqual(len(reg._skills), 0)
            reg.get_logger().warning.assert_called()

    def test_empty_persist_dir_loads_nothing(self):
        """An empty persistence directory results in zero loaded skills."""
        with tempfile.TemporaryDirectory() as tmpdir:
            reg = _make_registry(tmpdir)
            reg._load_persisted_skills()
            self.assertEqual(len(reg._skills), 0)

    def test_nonexistent_persist_dir_is_handled(self):
        """A non-existent persist directory does not raise."""
        reg = _make_registry("/tmp/definitely_does_not_exist_xyz123")
        try:
            reg._load_persisted_skills()
        except Exception as exc:
            self.fail(f"_load_persisted_skills raised unexpectedly: {exc}")

    def test_multiple_valid_files_all_loaded(self):
        """Multiple valid YAML files are all loaded."""
        with tempfile.TemporaryDirectory() as tmpdir:
            for skill_name in ["skill_a", "skill_b", "skill_c"]:
                data = {
                    "name": skill_name,
                    "display_name": skill_name.title(),
                    "bt_xml": "<BehaviorTree/>",
                }
                (Path(tmpdir) / f"{skill_name}.yaml").write_text(yaml.dump(data))

            reg = _make_registry(tmpdir)
            reg._load_persisted_skills()

            self.assertEqual(len(reg._skills), 3)
            self.assertIn("skill_a", reg._skills)
            self.assertIn("skill_b", reg._skills)
            self.assertIn("skill_c", reg._skills)


class TestSkillRegistryGetSkill(unittest.TestCase):
    """Test skill lookup."""

    def test_get_registered_skill_returns_description(self):
        from robot_skills_msgs.msg import SkillDescription
        reg = _make_registry()

        desc = SkillDescription()
        desc.name = "move_to_named_config"
        reg._skills["move_to_named_config"] = desc

        result = reg.get_skill("move_to_named_config")
        self.assertIsNotNone(result)
        self.assertEqual(result.name, "move_to_named_config")

    def test_get_unregistered_skill_returns_none(self):
        reg = _make_registry()
        result = reg.get_skill("nonexistent_skill")
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
