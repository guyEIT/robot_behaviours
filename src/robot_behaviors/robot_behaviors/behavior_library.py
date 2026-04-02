"""
BehaviorLibrary - Agent-friendly interface to pre-built behavior trees.

Allows agents and scripts to:
  1. Discover available pre-built behaviors with descriptions
  2. Load tree XML by name
  3. Execute a named behavior via the ExecuteBehaviorTree action

Usage:
  library = BehaviorLibrary(node)
  behaviors = library.list_behaviors()       # list all available trees
  xml = library.get_tree_xml("seed_collection")
  await library.execute_behavior("seed_collection", groot_monitor=True)
"""

import asyncio
from pathlib import Path
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory


# Metadata for pre-built behaviors
BEHAVIOR_CATALOG = {
    "move_to_home": {
        "name": "move_to_home",
        "display_name": "Move to Home",
        "description": (
            "Safely return the robot to the home configuration. "
            "Opens gripper first, then moves arm to home position."
        ),
        "category": "utility",
        "tags": ["home", "safety", "reset"],
        "preconditions": ["robot_initialized"],
        "postconditions": ["robot_at_home", "gripper_open"],
        "xml_file": "move_to_home.xml",
    },
    "seed_collection": {
        "name": "seed_collection",
        "display_name": "Seed Collection",
        "description": (
            "Full seed pickup sequence: move to observe pose, detect seed, "
            "compute pregrasp, descend, grasp, verify, retreat, return home."
        ),
        "category": "task",
        "tags": ["seed", "grasp", "perception", "pick"],
        "preconditions": [
            "robot_initialized",
            "camera_running",
            "seeds_in_workspace",
        ],
        "postconditions": ["seed_grasped", "robot_at_home"],
        "xml_file": "seed_collection.xml",
    },
    "pick_and_place": {
        "name": "pick_and_place",
        "display_name": "Pick and Place",
        "description": (
            "Generic pick and place with retry logic. "
            "Detects object by class, grasps it, places at target pose. "
            "Requires {object_class} and {place_pose} in blackboard."
        ),
        "category": "task",
        "tags": ["pick", "place", "generic", "retry"],
        "preconditions": [
            "robot_initialized",
            "camera_running",
            "object_class_set_in_blackboard",
            "place_pose_set_in_blackboard",
        ],
        "postconditions": ["object_placed_at_target", "robot_at_home"],
        "xml_file": "pick_and_place.xml",
    },
    "full_demo": {
        "name": "full_demo",
        "display_name": "Full Demo",
        "description": (
            "Comprehensive demo exercising all skills: robot enable, scene setup, "
            "point cloud capture, pick with retry, collision-safe transport, "
            "place, and cleanup. No external inputs needed."
        ),
        "category": "demo",
        "tags": [
            "demo", "pick", "place", "perception", "planning_scene",
            "collision", "digital_io", "rosbag", "full",
        ],
        "preconditions": ["robot_initialized"],
        "postconditions": ["robot_at_home", "gripper_open"],
        "xml_file": "full_demo.xml",
    },
    "human_interaction_demo": {
        "name": "human_interaction_demo",
        "display_name": "Human Interaction Demo",
        "description": (
            "Demonstrates all human interaction nodes: notification, warning, "
            "confirm (blocks for operator), input (blocks for value), and "
            "task assignment (blocks until operator reports done)."
        ),
        "category": "demo",
        "tags": ["demo", "human", "notification", "confirm", "input", "task"],
        "preconditions": ["robot_initialized"],
        "postconditions": ["robot_at_home"],
        "xml_file": "human_interaction_demo.xml",
    },
}


class BehaviorLibrary:
    """
    Agent-friendly interface to the pre-built behavior tree catalog.
    """

    def __init__(self):
        trees_dir = Path(get_package_share_directory("robot_behaviors")) / "trees"
        self._trees_dir = trees_dir

    def list_behaviors(self) -> List[Dict]:
        """Return all available pre-built behaviors with metadata."""
        behaviors = []
        for name, meta in BEHAVIOR_CATALOG.items():
            xml_path = self._trees_dir / meta["xml_file"]
            entry = dict(meta)
            entry["available"] = xml_path.exists()
            entry["xml_path"] = str(xml_path)
            behaviors.append(entry)
        return behaviors

    def get_tree_xml(self, behavior_name: str) -> Optional[str]:
        """
        Load and return the XML for a named behavior.

        Args:
            behavior_name: Key from BEHAVIOR_CATALOG

        Returns:
            XML string, or None if not found
        """
        meta = BEHAVIOR_CATALOG.get(behavior_name)
        if meta is None:
            return None

        xml_path = self._trees_dir / meta["xml_file"]
        if not xml_path.exists():
            return None

        return xml_path.read_text()

    def get_metadata(self, behavior_name: str) -> Optional[Dict]:
        """Get metadata for a named behavior."""
        return BEHAVIOR_CATALOG.get(behavior_name)

    def __repr__(self):
        names = list(BEHAVIOR_CATALOG.keys())
        return f"BehaviorLibrary(behaviors={names})"
