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

- Docker and Docker Compose
- X11 display (Linux desktop, or XQuartz on macOS)

### 1. Build and start

```bash
# Build images (compiles all ROS2 packages — only needed once or after source changes)
docker compose build

# Allow containers to access your display (run once per session, for Groot2/RViz)
DISPLAY=:1 xhost +local:

# Launch everything
docker compose up -d
```

That's it. `docker compose up` launches the full stack:
1. **sim container** — MoveIt2 + ros2_control simulation (Franka Panda mock hardware)
2. **dev container** — skill server (orchestrator + skill atoms + diagnostics)

Watch the logs with `docker compose logs -f`.

### 2. Shell into the dev container

```bash
docker exec -it ros2_robot_skills_dev bash
```

Everything is already sourced via `.bashrc`. Useful aliases: `cb` (build), `cbt <pkg>` (build one package), `ct` (test).

### 3. Run tests

```bash
docker exec ros2_robot_skills_dev bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/bt_ros2_ws/install/setup.bash && \
   source /home/ws/install/setup.bash && \
   cd /home/ws && colcon test && colcon test-result --verbose"
```

### 5. Call skills

```bash
# Shell into the dev container
docker exec -it ros2_robot_skills_dev bash

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

### 6. Groot2 (BT monitoring)

On the host, Pixi manages a platform-specific Groot2 binary under `.local/bin`.
It checks the configured path first and only downloads when the binary is
missing. The download metadata lives in `config/groot2.json`.

Note: the official download page may not provide a macOS installer. If that
applies, place your own `groot2` binary at `.local/bin/groot2` and `pixi run groot2`
will use it.

```bash
# Show the resolved host config for your platform
pixi run groot2-config

# Download or refresh the host-native Groot2 binary
pixi run groot2-download

# Launch Groot2 on the host (auto-downloads if missing)
pixi run groot2

# Optional: print the managed binary path
pixi run groot2-path
```

In Groot2: `Connect -> ZMQ Server -> Port 1666`.

### 7. RViz2 (robot visualization)

RViz2 is available in both containers. From the dev container:

```bash
docker exec -it ros2_robot_skills_dev bash
source /home/ws/install/setup.bash
rviz2
```

To launch the sim with RViz enabled (default is disabled in the sim container):

```bash
# Stop and restart sim with RViz
docker exec ros2_robot_sim bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /home/ws/install/setup.bash && \
   ros2 launch robot_sim_config sim.launch.py use_rviz:=true"
```

### Stopping

```bash
docker compose down
```

## Docker Setup

| Container | Image | Purpose |
|-----------|-------|---------|
| `ros2_robot_sim` | `ros2-jazzy-robot-sim` | MoveIt2 + ros2_control simulation (Franka Panda with mock hardware) |
| `ros2_robot_skills_dev` | `ros2-jazzy-robot-skills` | Full dev toolchain: BT.CPP, MoveIt2, Groot2, RealSense drivers |

Both containers use **host networking** for ROS2 DDS discovery and share `/tmp/.X11-unix` for GUI forwarding. All packages are pre-built during `docker compose build` so startup is fast.

The dev container bind-mounts `./src` from the host for development. On startup it runs a fast incremental rebuild (instant if source hasn't changed). After editing source files, run `cb` inside the container to rebuild.

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
