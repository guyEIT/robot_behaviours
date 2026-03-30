Create a new behavior tree task for the Robot Skills Framework.

Ask the user for:
1. What the robot should do (natural language description)
2. Any specific constraints (retry logic, failure handling, parallel execution)
3. Whether it should be a dashboard preset

## Understand the request

Map the user's description to available BT nodes. Read `src/robot_skill_server/src/bt_runner.cpp` for the current registered nodes and their action server mappings.

Available nodes:
- **MoveToNamedConfig** — move to named joint config (home, ready, stow, observe)
- **MoveToCartesianPose** — move end-effector to XYZ pose
- **GripperControl** — open/close gripper
- **DetectObject** — find objects with camera
- **ComputePreGraspPose** — offset a pose along Z
- **SetPose** — put a fixed pose on the blackboard
- **ScriptCondition** — evaluate a blackboard expression (BT.CPP built-in)

Check existing trees in `src/robot_behaviors/trees/` for reference patterns.

## Write the tree

Create `src/robot_behaviors/trees/<task_name>.xml`.

Follow BehaviorTree.CPP v4 format. Use blackboard variables (`{var}`) for data flow between nodes. Use SubTrees for reusable sequences.

Every tree must have:
```xml
<root BTCPP_format="4" main_tree_to_execute="TreeID">
  <BehaviorTree ID="TreeID">
    ...
  </BehaviorTree>
</root>
```

## Test it

Run through bt_runner to verify:
```bash
docker exec ros2_robot_skills_lite bash -c "cat > /tmp/test.xml << 'EOF'
<the xml>
EOF
source /opt/ros/jazzy/setup.bash && source /opt/bt_ros2_ws/install/setup.bash && source /home/ws/install/setup.bash && \
timeout 30 /home/ws/install/robot_skill_server/lib/robot_skill_server/bt_runner --tree-file /tmp/test.xml --tick-rate 10.0 --task-id ttest1"
```

Common errors:
- "port with name [X] is found in the XML but not in providedPorts()" — the XML uses a port that doesn't exist on the BT node. Check bt_skill_nodes.hpp for the correct port names.
- "missing input_pose" — a blackboard variable wasn't set by a preceding node. Trace the data flow.
- "Action server not reachable" — the skill atom isn't running or the action server name is wrong.

## Optionally add as a dashboard preset

If requested, add to the PRESET_TREES array in `src/robot_dashboard/frontend/src/components/executor/BehaviorExecutorPanel.tsx` and rebuild the frontend.

## Optionally register as a compound skill

Use `/skill_server/register_compound_skill` service to make it available in the skill registry. See the new-compound-skill command for details.
