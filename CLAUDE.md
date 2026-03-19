# Robot Skills Framework

## Development Environment

This is a ROS2 Jazzy workspace. All builds, tests, and ROS commands must run inside the Docker container.

### Starting the containers

```bash
docker compose up -d
```

This starts the sim container first (MoveIt2 + ros2_control), waits for `move_group` to be healthy, then starts the dev container.

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

### Packages

- `robot_skills_msgs` — ROS2 interfaces (actions/msgs/srvs)
- `robot_skill_atoms` — C++ primitive skill action servers
- `robot_bt_nodes` — C++ BehaviorTree.CPP v4 leaf node plugins
- `robot_skill_server` — Python orchestrator (SkillRegistry, TaskComposer, BtExecutor)
- `robot_behaviors` — XML behavior tree definitions
- `robot_sim_config` — MoveIt2 + ros2_control configuration (URDF, SRDF, controllers)
