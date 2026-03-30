Create a new skill atom for the Robot Skills Framework.

Ask the user for:
1. Skill name (snake_case, e.g. "measure_force")
2. Brief description of what it does
3. Goal parameters (what the caller sends)
4. Result fields (what comes back)
5. Feedback fields (what streams during execution)
6. Python or C++ implementation
7. Whether to also create a mock version

Then create ALL of the following:

## Action definition

Create `src/robot_skills_msgs/action/<SkillName>.action` with the three sections (Goal, Result, Feedback). Always include `bool success` and `string message` in Result.

Add the action to `src/robot_skills_msgs/CMakeLists.txt` in the `rosidl_generate_interfaces()` call.

## Skill atom implementation

### If Python:

Create `src/robot_skill_atoms/robot_skill_atoms/<skill_name>.py` with:
- A class inheriting from `rclpy.node.Node`
- An `ActionServer` on `/skill_atoms/<skill_name>`
- `_on_goal`, `_on_cancel`, and `_execute` callbacks
- Feedback publishing during execution
- Cancellation support
- A `main()` entry point

Add the console_scripts entry point in `src/robot_skill_atoms/setup.py`.

### If C++:

Create `src/robot_skill_atoms/src/<skill_name>.cpp` inheriting from `robot_skill_atoms::SkillBase<ActionT>` (see `src/robot_skill_atoms/include/robot_skill_atoms/skill_base.hpp`).

Implement: constructor, `getDescription()`, `checkPreconditions()`, `executeGoal()`.

Add executable to `src/robot_skill_atoms/CMakeLists.txt`.

## BT node wrapper

Add a new class to `src/robot_bt_nodes/include/robot_bt_nodes/bt_skill_nodes.hpp` inheriting from `BT::RosActionNode<ActionT>`.

Implement: `providedPorts()`, `setGoal()`, `onResultReceived()`, `onFailure()`.

Register in `src/robot_bt_nodes/src/bt_nodes_plugin.cpp`.

Register in `src/robot_skill_server/src/bt_runner.cpp` using `make_params("/skill_atoms/<skill_name>")`.

## Mock (if requested)

Create `src/robot_mock_skill_atoms/src/mock_<skill_name>.cpp` following the pattern of existing mocks (e.g. `mock_gripper_control.cpp`).

Include:
- Configurable delay parameter
- Plausible mock results
- If it moves the robot: publish to `/mock/joint_target` or `/mock/gripper_target`
- Feedback publishing during simulated execution
- Component registration via `RCLCPP_COMPONENTS_REGISTER_NODE`

Add to `src/robot_mock_skill_atoms/CMakeLists.txt` as a shared library with component registration.

Add parameters to `src/robot_mock_skill_atoms/config/mock_params.yaml`.

Add the node to `src/robot_skill_server/launch/skill_server_lite.launch.py`.

## After creating all files

Build and verify:
```bash
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && cd /home/ws && colcon build --packages-select robot_skills_msgs && colcon build"
```

Restart the container and verify the action server appears:
```bash
docker compose -f docker-compose.lite.yml restart
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && ros2 action list | grep <skill_name>"
```

Write a test BT XML and run it through bt_runner to verify end-to-end.
