"""
SkillRegistry - Central catalog of all robot skills.

Maintains a registry of:
  - Atom skills (registered by C++ skill servers on startup)
  - Compound skills (registered by agents via RegisterCompoundSkill)

Provides services:
  - /skill_server/register_skill        (internal, called by skill atoms)
  - /skill_server/get_skill_descriptions (agent-facing)
  - /skill_server/register_compound_skill (agent-facing)
"""

import json
import os
import threading
import yaml
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from robot_skills_msgs.msg import SkillDescription
from robot_skills_msgs.srv import (
    GetSkillDescriptions,
    RegisterCompoundSkill,
    RegisterSkill,
)
from std_msgs.msg import String


class SkillRegistry(Node):
    """
    Central skill catalog for the Robot Skills Framework.

    Atom skills self-register on startup. Agents can then:
    1. Query all available skills (atoms + compounds)
    2. Register new compound skills (BT XML + metadata)
    3. Use skill metadata for task planning (PDDL, LLM prompting)
    """

    def __init__(self):
        super().__init__("skill_registry")

        self._lock = threading.RLock()
        self._skills: Dict[str, SkillDescription] = {}

        # Persistence directory for compound skills
        self.declare_parameter("persist_dir", "~/.ros2_robot_skills/compound_skills")
        persist_dir = os.path.expanduser(
            self.get_parameter("persist_dir").get_parameter_value().string_value
        )
        self._persist_dir = Path(persist_dir)

        # Services
        self._register_srv = self.create_service(
            RegisterSkill,
            "/skill_server/register_skill",
            self._handle_register_skill,
        )
        self._get_skills_srv = self.create_service(
            GetSkillDescriptions,
            "/skill_server/get_skill_descriptions",
            self._handle_get_skill_descriptions,
        )
        self._register_compound_srv = self.create_service(
            RegisterCompoundSkill,
            "/skill_server/register_compound_skill",
            self._handle_register_compound_skill,
        )

        # Publish available skills (latched-like: publish on every change)
        self._skills_pub = self.create_publisher(
            String, "/skill_server/available_skills", 10
        )

        # Load persisted compound skills from disk
        self._load_persisted_skills()

        self.get_logger().info(
            "SkillRegistry started. "
            "Waiting for skill atoms to register on /skill_server/register_skill"
        )

    # ── Service handlers ─────────────────────────────────────────────────────

    def _handle_register_skill(
        self,
        request: RegisterSkill.Request,
        response: RegisterSkill.Response,
    ) -> RegisterSkill.Response:
        """Called by skill atom servers on startup to register themselves."""
        desc = request.description
        with self._lock:
            self._skills[desc.name] = desc
            self.get_logger().info(
                f"Registered skill atom: '{desc.name}' "
                f"[{desc.category}] -> {desc.action_server_name}"
            )
        self._publish_skill_list()
        response.success = True
        response.message = f"Skill '{desc.name}' registered"
        return response

    def _handle_get_skill_descriptions(
        self,
        request: GetSkillDescriptions.Request,
        response: GetSkillDescriptions.Response,
    ) -> GetSkillDescriptions.Response:
        """Return all registered skills, optionally filtered."""
        with self._lock:
            skills = list(self._skills.values())

        # Filter by category
        if request.filter_categories:
            skills = [
                s for s in skills if s.category in request.filter_categories
            ]

        # Filter by tags
        if request.filter_tags:
            skills = [
                s for s in skills
                if any(tag in s.tags for tag in request.filter_tags)
            ]

        # Optionally exclude compound skills
        if not request.include_compounds:
            skills = [s for s in skills if not s.is_compound]

        # Optionally strip PDDL (saves bandwidth)
        if not request.include_pddl:
            for s in skills:
                s.pddl_action = ""

        response.skills = skills
        response.success = True
        response.message = f"Returning {len(skills)} skills"
        return response

    def _handle_register_compound_skill(
        self,
        request: RegisterCompoundSkill.Request,
        response: RegisterCompoundSkill.Response,
    ) -> RegisterCompoundSkill.Response:
        """Register an agent-composed compound skill."""
        desc = request.skill_description
        bt_xml = request.bt_xml

        if not desc.name:
            response.success = False
            response.message = "skill_description.name must not be empty"
            return response

        if not bt_xml:
            response.success = False
            response.message = "bt_xml must not be empty"
            return response

        # Mark as compound and store XML
        desc.is_compound = True
        desc.bt_xml = bt_xml
        desc.category = desc.category or "compound"

        with self._lock:
            self._skills[desc.name] = desc
            self.get_logger().info(
                f"Registered compound skill: '{desc.name}' "
                f"(components: {desc.component_skills})"
            )

        # Persist to disk if requested
        if request.persist:
            self._persist_compound_skill(desc, bt_xml)

        self._publish_skill_list()

        response.success = True
        response.registered_name = desc.name
        response.message = f"Compound skill '{desc.name}' registered successfully"
        return response

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _publish_skill_list(self):
        """Publish a JSON summary of available skills."""
        with self._lock:
            summary = [
                {
                    "name": s.name,
                    "display_name": s.display_name,
                    "category": s.category,
                    "is_compound": s.is_compound,
                    "action_server": s.action_server_name,
                }
                for s in self._skills.values()
            ]
        msg = String()
        msg.data = json.dumps(summary, indent=2)
        self._skills_pub.publish(msg)

    def _persist_compound_skill(self, desc: SkillDescription, bt_xml: str):
        """Save compound skill to disk for persistence across restarts."""
        self._persist_dir.mkdir(parents=True, exist_ok=True)
        skill_file = self._persist_dir / f"{desc.name}.yaml"

        data = {
            "name": desc.name,
            "display_name": desc.display_name,
            "description": desc.description,
            "category": desc.category,
            "tags": list(desc.tags),
            "preconditions": list(desc.preconditions),
            "postconditions": list(desc.postconditions),
            "effects": list(desc.effects),
            "constraints": list(desc.constraints),
            "component_skills": list(desc.component_skills),
            "parameters_schema": desc.parameters_schema,
            "bt_xml": bt_xml,
            "saved_at": datetime.utcnow().isoformat(),
        }

        with open(skill_file, "w") as f:
            yaml.dump(data, f, default_flow_style=False)

        self.get_logger().info(f"Persisted compound skill to {skill_file}")

    def _load_persisted_skills(self):
        """Load compound skills saved from previous sessions."""
        if not self._persist_dir.exists():
            return

        count = 0
        for skill_file in self._persist_dir.glob("*.yaml"):
            try:
                with open(skill_file) as f:
                    data = yaml.safe_load(f)

                desc = SkillDescription()
                desc.name = data["name"]
                desc.display_name = data.get("display_name", desc.name)
                desc.description = data.get("description", "")
                desc.category = data.get("category", "compound")
                desc.tags = data.get("tags", [])
                desc.preconditions = data.get("preconditions", [])
                desc.postconditions = data.get("postconditions", [])
                desc.effects = data.get("effects", [])
                desc.constraints = data.get("constraints", [])
                desc.component_skills = data.get("component_skills", [])
                desc.parameters_schema = data.get("parameters_schema", "")
                desc.is_compound = True
                desc.bt_xml = data.get("bt_xml", "")

                with self._lock:
                    self._skills[desc.name] = desc
                count += 1

            except Exception as e:
                self.get_logger().warn(
                    f"Failed to load persisted skill from {skill_file}: {e}"
                )

        if count > 0:
            self.get_logger().info(
                f"Loaded {count} persisted compound skill(s) from {self._persist_dir}"
            )
            self._publish_skill_list()

    def get_skill(self, name: str) -> Optional[SkillDescription]:
        """Get a skill description by name (for internal use by other components)."""
        with self._lock:
            return self._skills.get(name)

    def get_all_skills(self) -> List[SkillDescription]:
        """Get all registered skills (for internal use)."""
        with self._lock:
            return list(self._skills.values())
