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

### Prerequisites

- Pixi
- Linux desktop for the local ROS2 Jazzy runtime

Docker and Docker Compose are still available as a fallback, but the normal
development loop is local through Pixi.

### 1. Build and start locally

```bash
pixi run dev
```

This builds the dashboard, builds the lite ROS workspace with `colcon`, and
launches the local lite stack:

1. mock skill atoms
2. skill server, task composer, and BT executor
3. robot state publisher, diagnostics, rosbridge, and dashboard

The dashboard is served at `http://localhost:8080`. Stop the local stack with
`Ctrl+C`.

### 2. Local development loop

```bash
# Rebuild ROS packages after Python/XML/C++ changes
pixi run dev-build-fast

# Relaunch without rebuilding
pixi run dev-run

# Open a sourced local ROS shell
pixi run dev-shell

# Inspect the local ROS graph
pixi run status
```

Use `pixi run dashboard-dev` when you want the React dev server with hot
reload. Use `pixi run dashboard-build` before `dev-build-fast` when frontend
assets should be installed into the ROS package.

### 3. Run tests

```bash
pixi run test
```

### 4. Call skills

```bash
# Open a local ROS shell
pixi run dev-shell

# List all available skills
ros2 service call /skill_server/get_skill_descriptions \
  robot_skills_msgs/srv/GetSkillDescriptions \
  '{include_compounds: true, include_pddl: false}'

# Move to home
ros2 action send_goal /skill_server/execute_behavior_tree \
  robot_skills_msgs/action/ExecuteBehaviorTree \
  '{tree_name: "home", tree_xml: "'"$(cat src/robot_behaviors/trees/move_to_home.xml)"'"}'

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
```

### 5. Groot2 (BT monitoring)

Groot2 is no longer managed through Pixi. If you run Groot2 separately, connect
it to the running skill server with `Connect -> ZMQ Server -> Port 1666`.

### 6. Native pixi build (no Docker)

`pixi-build-ros` builds each ROS package as a conda package inside a pixi
environment. Sibling packages (e.g. `robot_mock_skill_atoms` including message
headers from `robot_skills_msgs`) can't be chained through pixi-build's build
environments in pixi ≤ 0.67, so `robot_skills_msgs` is hosted in a local conda
channel at `~/channel`. Every other package is declared as a path-dependency
and rebuilt by pixi-build-ros on source change.

#### First time per machine

```bash
bash scripts/bootstrap-msgs.sh
```

Creates `~/channel/` and builds `robot_skills_msgs` into it. Run this once on
a clean machine. It bypasses `pixi run` because pixi 0.67 eagerly resolves all
declared environments, and the `lite-native` / `real-native` envs reference
`~/channel` — so no `pixi run *` task can start until `~/channel` exists.

#### Run natively

After the bootstrap, one command handles everything:

```bash
pixi run lite-native-up   # rebuild msgs + dashboard + all path-deps, then launch lite stack
pixi run real-native-up   # same, but builds skill_atoms (MoveIt2) instead of mock atoms
```

Both tasks `depends-on` `update-msgs` and `dashboard-build`, so you don't need
to chain commands — source edits anywhere in the workspace are picked up on
the next `*-native-up`.

`real-native-up` expects the host's `move_group` (Meca500 MoveIt2) to already
be running; it connects over the shared DDS domain.

#### Common workflows

```bash
# Rebuild only msgs (after editing a .msg/.srv/.action):
pixi run update-msgs

# Launch without rebuilding anything:
pixi run lite-native-run          # or real-native-run (auto-picks the right env)

# Open a shell in the native env:
pixi run lite-native-shell        # or real-native-shell

# Wipe the native envs (keeps ~/channel):
pixi run native-clean

# Nuke everything and start over:
pixi run native-clean && rm -rf ~/channel && bash scripts/bootstrap-msgs.sh
```

The native tasks launch the dashboard on **port 8081** (matching the Docker-lite
convention) and rosbridge on **port 9090**. Override with a launch arg:

```bash
pixi run lite-native-run dashboard_port:=8082 rosbridge_port:=9091
```

If the Docker lite container is running, stop it first to free the ports:

```bash
pixi run lite-down && pixi run lite-native-up
```

### 7. Docker fallback

The Docker workflows remain in Pixi for comparison and fallback:

```bash
pixi run lite-up
pixi run lite-logs
pixi run lite-down

pixi run real-up
pixi run real-logs
pixi run real-down

pixi run docker-status
pixi run docker-test
```

## Docker Fallback

| Container | Image | Purpose |
|-----------|-------|---------|
| `ros2_robot_skills_lite` | `ros2-jazzy-robot-skills-lite` | Lite mock skill stack |
| `ros2_robot_skills_dev` | `ros2-jazzy-robot-skills` | Real robot skill server container |

Docker containers use host networking for ROS2 DDS discovery and bind-mount
`./src` for source edits. They are kept for fallback while local Pixi dev is
the preferred path.

## Package Structure

| Package | Language | Role |
|---------|----------|------|
| `robot_skills_msgs` | — | ROS2 message/service/action definitions |
| `robot_skill_atoms` | C++ | Primitive skill action servers |
| `robot_bt_nodes` | C++ | BT.CPP v4 leaf node plugins |
| `robot_skill_server` | Python/C++ | Orchestrator: SkillRegistry, TaskComposer, BtExecutor |
| `robot_behaviors` | XML | Pre-built behavior trees |
| `robot_sim_config` | — | MoveIt2 + ros2_control config (URDF, SRDF, controllers) |

## Skill Atoms

| Skill | Action Topic | Description |
|-------|-------------|-------------|
| `move_to_named_config` | `/skill_atoms/move_to_named_config` | Move to named joint config (home, ready, stow, observe) |
| `move_to_cartesian_pose` | `/skill_atoms/move_to_cartesian_pose` | Move EEF to 6-DOF pose |
| `gripper_control` | `/skill_atoms/gripper_control` | Open/close/position gripper with force control |
| `detect_object` | `/skill_atoms/detect_object` | Detect objects with RealSense (stub — needs ML backend) |

## Pre-built Behaviors

| Behavior | File | Description |
|----------|------|-------------|
| `move_to_home` | `trees/move_to_home.xml` | Safe home return (open gripper + move to home config) |
| `seed_collection` | `trees/seed_collection.xml` | Full seed pick sequence (uses detect_object) |
| `pick_and_place` | `trees/pick_and_place.xml` | Generic pick+place with retry logic |

## Adding New Skills

1. Create a new action definition in `robot_skills_msgs/action/`
2. Implement `YourSkill : SkillBase<YourAction>` in `robot_skill_atoms/src/`
3. Add a `YourSkillNode : BT::RosActionNode<YourAction>` in `robot_bt_nodes/src/`
4. Register in `bt_runner.cpp` and `bt_nodes_plugin.cpp`
5. The skill auto-appears in `GetSkillDescriptions` after `colcon build`
