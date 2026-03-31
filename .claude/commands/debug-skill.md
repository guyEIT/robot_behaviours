Diagnose why a skill or behavior tree is failing.

Ask the user what's going wrong — a specific error message, a skill that hangs, a tree that returns FAILURE, etc.

## Diagnostic steps

### 1. Check the container is running

```bash
docker ps --format '{{.Names}}' | grep lite
```

If not running: `pixi run lite-up`

### 2. Check action servers are available

```bash
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && ros2 action list"
```

Expected: `/skill_atoms/gripper_control`, `/skill_atoms/move_to_named_config`, `/skill_atoms/move_to_cartesian_pose`, `/skill_atoms/detect_object`, `/skill_server/execute_behavior_tree`

If a skill atom is missing, check the launch file and container logs:
```bash
docker logs ros2_robot_skills_lite 2>&1 | grep -i error | tail -20
```

### 3. Check the skill registry

```bash
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && source /home/ws/install/setup.bash && ros2 service call /skill_server/get_skill_descriptions robot_skills_msgs/srv/GetSkillDescriptions '{include_compounds: true}'"
```

### 4. Test a BT tree directly with bt_runner

Write the XML to a file in the container and run:
```bash
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && source /opt/bt_ros2_ws/install/setup.bash && source /home/ws/install/setup.bash && /home/ws/install/robot_skill_server/lib/robot_skill_server/bt_runner --tree-file /tmp/test.xml --tick-rate 10.0 --task-id tdbg1"
```

### 5. Common errors and fixes

**"port with name [X] is found in the XML but not in providedPorts()"**
The BT XML uses a port that doesn't exist on the node. Check `src/robot_bt_nodes/include/robot_bt_nodes/bt_skill_nodes.hpp` for the correct port names.

**"RosActionNode: no client was specified"**
The BT node doesn't know which action server to connect to. Check `src/robot_skill_server/src/bt_runner.cpp` — the node must be registered with `make_params("/skill_atoms/<name>")`.

**"Action server with name X is not reachable"**
The skill atom isn't running. Check `ros2 action list` and the container logs.

**"topic name token must not start with a number"**
The task ID starts with a digit. This was fixed by prefixing with `"t"` in `bt_executor.py`. If it recurs, check `bt_executor.py` line that generates `task_id`.

**"terminate called after throwing an instance of 'BT::RuntimeError'"**
A BT node crashed. The error message after `what():` tells you the exact cause. Usually a missing blackboard variable or wrong port type.

**Tree returns FAILURE but no error**
A node returned FAILURE (not an exception). Check which node failed by looking at the bt_runner output — the last `current_bt_node` before failure. Then check the mock's logic (e.g., DetectObject returns FAILURE if `object_class` doesn't match `mock_detection_class` in `mock_params.yaml`).

**Dashboard shows "Executing..." but nothing happens**
Check if rosbridge is forwarding the action. Look at container logs for `send_action_goal`. If the action goal is accepted but bt_runner crashes, you'll see the error in `docker logs`.

### 6. Check rosbridge connectivity

```bash
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic info /skill_server/task_state -v"
```

Verify the publisher (bt_executor) and subscriber (rosbridge_websocket) have matching QoS (both RELIABLE).

### 7. Rebuild after fixes

```bash
# Rebuild specific package
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && source /opt/bt_ros2_ws/install/setup.bash && cd /home/ws && colcon build --packages-select <package>"

# Restart
docker compose -f docker-compose.lite.yml restart

# For frontend changes, also rebuild dashboard symlinks
cd src/robot_dashboard/frontend && npx vite build
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && cd /home/ws && colcon build --symlink-install --packages-select robot_dashboard"
```
