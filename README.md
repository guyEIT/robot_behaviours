# Robot Skills Framework — ROS2 Jazzy

A hierarchical, extensible, agent-callable robot control framework for the **Meca500 manipulator** with **Intel RealSense** camera, built on **ROS2 Jazzy** + **MoveIt2** + **BehaviorTree.CPP v4**.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  AI Agent / LLM / PlanSys2 / External Scheduler             │
│  QuerySkills → ComposeTask → RegisterSkill → Execute         │
└──────────────────┬───────────────────────────────────────────┘
                   │  ROS2 Services + Actions
┌──────────────────▼───────────────────────────────────────────┐
│  robot_skill_server (Python)                                 │
│  SkillRegistry │ TaskComposer │ BtExecutor                   │
└──────┬───────────────────────────────────┬────────────────────┘
       │ runs BT trees                     │ direct calls
┌──────▼──────────┐         ┌──────────────▼──────────────────┐
│ robot_behaviors  │         │ robot_skill_atoms (C++)          │
│ (XML trees)     │         │ MoveToNamedConfig                │
└──────┬──────────┘         │ MoveToCartesianPose              │
       │                    │ GripperControl                   │
┌──────▼──────────┐         │ DetectObject                     │
│ robot_bt_nodes  │─────────│                                  │
│ (C++ BT plugins)│         └──────────────┬───────────────────┘
└─────────────────┘                        │
              ┌────────────────────────────┼──────────────┐
       ┌──────▼──────┐  ┌─────────────────▼──┐  ┌────────▼──────┐
       │   MoveIt2   │  │ realsense2_camera   │  │ ros2_control  │
       └─────────────┘  └────────────────────┘  └───────────────┘
```

## Quick Start

### 1. Open in DevContainer (macOS)

```bash
# Install XQuartz for display forwarding
brew install xquartz
open -a XQuartz  # Enable: Security → Allow connections from network clients
xhost +localhost

# Open in VSCode with DevContainer extension
code /path/to/workspace
# → "Reopen in Container"
```

### 2. Build

```bash
# Inside the container:
cd /home/ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch

```bash
# Simulation mode (default)
ros2 launch robot_skill_server skill_server.launch.py

# Real hardware + RealSense
ros2 launch robot_skill_server skill_server.launch.py \
  hardware_mode:=real \
  use_realsense:=true \
  robot_ip:=192.168.0.100

# With Groot2 monitoring (run groot2 separately, connect to port 1666)
ros2 launch robot_skill_server skill_server.launch.py \
  groot_port:=1666
```

### 4. Call Skills (CLI)

```bash
# List all available skills
ros2 service call /skill_server/get_skill_descriptions \
  robot_skills_msgs/srv/GetSkillDescriptions \
  '{include_compounds: true, include_pddl: false}'

# Move to home
ros2 action send_goal /skill_server/execute_behavior_tree \
  robot_skills_msgs/action/ExecuteBehaviorTree \
  '{tree_name: "home", tree_xml: "$(cat src/robot_behaviors/trees/move_to_home.xml)", enable_groot_monitor: true}'

# Run seed collection
ros2 action send_goal /skill_server/execute_behavior_tree \
  robot_skills_msgs/action/ExecuteBehaviorTree \
  '{tree_name: "seed_collection", tree_xml: "$(cat src/robot_behaviors/trees/seed_collection.xml)"}'
```

### 5. Agent Composition

```bash
# Compose a custom task from skill steps
ros2 service call /skill_server/compose_task \
  robot_skills_msgs/srv/ComposeTask \
  '{
    task_name: "my_task",
    task_description: "Custom pick sequence",
    sequential: true,
    steps: [
      {skill_name: "move_to_named_config", parameters_json: "{\"config_name\": \"observe\"}"},
      {skill_name: "detect_object", parameters_json: "{\"object_class\": \"seed\", \"timeout_sec\": 5.0}"},
      {skill_name: "gripper_control", parameters_json: "{\"command\": \"open\"}"}
    ]
  }'

# Register the composed task as a named compound skill
ros2 service call /skill_server/register_compound_skill \
  robot_skills_msgs/srv/RegisterCompoundSkill \
  '{skill_description: {name: "my_task", category: "compound"}, bt_xml: "<...>", persist: true}'
```

### 6. Groot2 Monitoring

```bash
# Open Groot2 in a terminal inside the container
groot2 --no-sandbox &

# In Groot2:
# → Connect → ZMQ Server → Port 1666
# You'll see the live behavior tree with node status colors
```

## Package Structure

| Package | Language | Role |
|---------|----------|------|
| `robot_skills_msgs` | — | ROS2 message/service/action definitions |
| `robot_skill_atoms` | C++ | Primitive skill action servers |
| `robot_bt_nodes` | C++ | BT.CPP v4 leaf node plugins |
| `robot_skill_server` | Python | Orchestrator: SkillRegistry, TaskComposer, BtExecutor |
| `robot_behaviors` | XML/Python | Pre-built behavior trees |

## Skill Atoms

| Skill | Action Topic | Description |
|-------|-------------|-------------|
| `move_to_named_config` | `/skill_atoms/move_to_named_config` | Move to named joint config |
| `move_to_cartesian_pose` | `/skill_atoms/move_to_cartesian_pose` | Move EEF to 6-DOF pose |
| `gripper_control` | `/skill_atoms/gripper_control` | Open/close/position gripper |
| `detect_object` | `/skill_atoms/detect_object` | Detect objects with RealSense |

## Pre-built Behaviors

| Behavior | File | Description |
|----------|------|-------------|
| `move_to_home` | `trees/move_to_home.xml` | Safe home return |
| `seed_collection` | `trees/seed_collection.xml` | Full seed pick sequence |
| `pick_and_place` | `trees/pick_and_place.xml` | Generic pick+place with retry |

## Self-Describing Skills for Planning

Each skill exposes PDDL-compatible metadata via `GetSkillDescriptions`:
- **preconditions**: what must be true before executing
- **postconditions**: what is guaranteed after success
- **effects**: PDDL-style world state changes
- **constraints**: safety and operational limits
- **parameters_schema**: JSON Schema for goal parameters

This metadata feeds directly into:
- LLM agent prompting (skill discovery)
- PlanSys2 PDDL domain generation
- Runtime precondition checking

## Hardware Modes

| Mode | MoveIt2 | RealSense | Hardware |
|------|---------|-----------|---------|
| `sim` | Fake controllers | Not required | None |
| `real` | Real controllers | Required | Physical Meca500 |

Switch with: `ros2 launch robot_skill_server skill_server.launch.py hardware_mode:=real`

## Adding New Skills

1. Create a new action definition in `robot_skills_msgs/action/`
2. Implement `YourSkill : SkillBase<YourAction>` in `robot_skill_atoms/src/`
3. Add a `YourSkillNode : BT::RosActionNode<YourAction>` in `robot_bt_nodes/src/`
4. Register in `bt_runner.cpp` and `bt_nodes_plugin.cpp`
5. The skill auto-appears in `GetSkillDescriptions` after `colcon build`

## Dependencies

- ROS2 Jazzy
- BehaviorTree.CPP v4 (`ros-jazzy-behaviortree-cpp`)
- BehaviorTree.ROS2 (`ros-jazzy-behaviortree-ros2`)
- MoveIt2 (`ros-jazzy-moveit`)
- ros2_control (`ros-jazzy-ros2-control`)
- realsense2_camera (`ros-jazzy-realsense2-camera`)
- PlanSys2 (`ros-jazzy-plansys2-*`) — optional, for PDDL task planning
- Groot2 — download from https://www.behaviortree.dev/groot/
