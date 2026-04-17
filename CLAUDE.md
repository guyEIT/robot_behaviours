# Robot Skills Framework

## Development Environment

This is a ROS2 Jazzy workspace. All builds, tests, and ROS commands must run inside the Docker container.

### Starting the containers

**Real robot** (Meca500 with MoveIt2 running on host):
```bash
docker compose up -d
```

This starts the dev container which connects to the host's MoveIt2 instance via host networking. The skill server auto-configures for the Meca500 (planning group `meca500_arm`, controller `/joint_trajectory_controller`).

**Simulation** (Panda mock hardware):
```bash
docker compose --profile sim up -d
# or: ROBOT_HARDWARE_MODE=sim docker compose --profile sim up -d
```

This starts the sim container (MoveIt2 + ros2_control with Panda), waits for `move_group` to be healthy, then starts the dev container with sim-mode parameters.

### Running commands inside the container

Use `docker exec` to run commands:

```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash 2>/dev/null; <COMMAND>"
```

### Common commands

Build all packages:
```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && cd /home/ws && colcon build --symlink-install"
```

Build a single package:
```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && cd /home/ws && colcon build --symlink-install --packages-select <PACKAGE>"
```

Run tests:
```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash && cd /home/ws && colcon test && colcon test-result --verbose"
```

### Launch the skill server

```bash
docker exec -it ros2_robot_skills_dev bash -c "source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash && ros2 launch robot_skill_server skill_server.launch.py"
```

### Lite sim mode (no MoveIt2, single container)

For dashboard and orchestration development without the full MoveIt2 stack:

```bash
pixi run lite-up        # start
pixi run lite-down      # stop
pixi run lite-restart   # restart
pixi run lite-logs      # follow logs
pixi run lite-shell     # shell into container
```

Dashboard: http://localhost:8081 | rosbridge: ws://localhost:9090

After frontend changes (`vite build`), update the container symlinks:
```bash
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && cd /home/ws && colcon build --symlink-install --packages-select robot_dashboard"
```

After C++ changes in the container, rebuild and restart:
```bash
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && source /opt/bt_ros2_ws/install/setup.bash && cd /home/ws && colcon build --packages-select <PACKAGE>"
pixi run lite-restart
```

### Live updating of skills and behavior trees

The workspace is designed so that Python, XML, and frontend changes take effect without rebuilding Docker images.

**How it works:**

1. **Volume mounts** — Docker Compose mounts `./src:/home/ws/src:rw` into the container. Edits on the host are immediately visible inside the container.
2. **Symlink install** — `colcon build --symlink-install` creates symlinks from `install/` back to `src/` for Python packages and XML share files. This means Python code and behavior tree XMLs are served directly from the mounted source.
3. **Behavior tree auto-discovery** — `BtExecutor` (`bt_executor.py`) prefers the source directory `/home/ws/src/robot_behaviors/trees` over the installed share. A 2-second polling timer (`_check_trees_changed`) detects new, modified, or deleted `.xml` files and re-publishes the tree list to the latched `/skill_server/available_trees` topic.
4. **Frontend live subscription** — The dashboard subscribes to `/skill_server/available_trees` via rosbridge (WebSocket on port 9090). New trees appear in the Preset selector within ~2 seconds of the file being saved.

**What requires what:**

| Change type | Action needed |
|---|---|
| **New/edited behavior tree XML** | Save the `.xml` file in `src/robot_behaviors/trees/` — auto-discovered in ~2s |
| **Python skill server code** | Save the file — symlink-install means it's already live. Restart the node if it caches state at startup |
| **Frontend (React/Vite)** | Run `vite build` then `colcon build --symlink-install --packages-select robot_dashboard` inside the container (or use `pixi run dashboard-dev` for Vite hot-reload during development) |
| **C++ skill atoms or BT nodes** | Rebuild the package inside the container (`colcon build --packages-select <PKG>`) and restart the relevant nodes |
| **New C++ package or Dockerfile change** | Rebuild the Docker image |

**Uploading skills from the dashboard:**

The frontend provides three execution modes in the Behavior Executor panel:

- **Preset** — Select from auto-discovered tree XML files (from `src/robot_behaviors/trees/`)
- **Compose** — Build a task from registered skill steps. Calls `/skill_server/compose_task` to generate BT XML, then executes via `/skill_server/execute_behavior_tree`
- **Raw** — Paste arbitrary BT.CPP v4 XML and execute it directly

Compound skills can be registered at runtime via `/skill_server/register_compound_skill` and are persisted to `~/.ros2_robot_skills/compound_skills/{name}.yaml` so they survive container restarts. Atom skills self-register with the SkillRegistry when their action servers start.

**Key ROS2 topics and services:**

| Endpoint | Type | Purpose |
|---|---|---|
| `/skill_server/available_trees` | Topic (latched) | JSON list of discovered tree files and their XML content |
| `/skill_server/active_bt_xml` | Topic (latched) | XML of the currently-executing behavior tree |
| `/skill_server/task_state` | Topic | Execution progress (current node, completed/failed skills) |
| `/skill_server/execute_behavior_tree` | Action | Execute arbitrary BT XML at runtime |
| `/skill_server/compose_task` | Service | Generate BT XML from an ordered list of TaskSteps |
| `/skill_server/register_compound_skill` | Service | Register and persist a new compound skill |
| `/skill_server/get_skill_descriptions` | Service | Query the skill registry (supports category/tag filters) |

### Packages

- `robot_skills_msgs` — ROS2 interfaces (actions/msgs/srvs)
- `robot_skill_atoms` — C++ (or Python) primitive skill action servers
- `robot_mock_skill_atoms` — C++ mock skill atoms for lite sim mode
- `robot_bt_nodes` — C++ BehaviorTree.CPP v4 leaf node plugins
- `robot_skill_server` — Python orchestrator (SkillRegistry, TaskComposer, BtExecutor) + C++ bt_runner
- `robot_behaviors` — XML behavior tree definitions
- `robot_dashboard` — React web dashboard (Vite + Three.js + rosbridge)
- `robot_sim_config` — MoveIt2 + ros2_control configuration (URDF, SRDF, controllers)

### Documentation

- `docs/adding-skills.md` — How to add new skills (atoms, mocks, BT nodes, compound skills)
- `.claude/commands/new-skill-atom.md` — Claude command: create a new skill atom
- `.claude/commands/new-compound-skill.md` — Claude command: create a compound skill
- `.claude/commands/new-behavior-tree.md` — Claude command: create a behavior tree
- `.claude/commands/debug-skill.md` — Claude command: diagnose skill/BT failures
