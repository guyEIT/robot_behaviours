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
