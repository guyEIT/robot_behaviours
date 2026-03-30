Create a new compound skill that combines existing skill atoms into a reusable behavior tree.

Ask the user for:
1. Compound skill name (snake_case)
2. Description of the overall task
3. Which existing skills to combine (in order)
4. Parameters for each step
5. Whether steps should be sequential or have branching/retry logic
6. Whether data needs to flow between steps (blackboard variables)
7. Whether to persist the skill (survives restarts)

## Discover available skills first

Read the existing skill atoms to understand what's available:
- Check `src/robot_bt_nodes/include/robot_bt_nodes/bt_skill_nodes.hpp` for all BT node classes and their ports
- Check `src/robot_skill_server/src/bt_runner.cpp` for registered node names and action server mappings
- Check existing trees in `src/robot_behaviors/trees/` for patterns

The available BT nodes and their ports:
- **MoveToNamedConfig**: config_name, velocity_scaling, acceleration_scaling
- **MoveToCartesianPose**: target_pose, velocity_scaling, acceleration_scaling
- **GripperControl**: command (open/close), force_limit → object_grasped, final_position
- **DetectObject**: object_class, confidence_threshold, timeout_sec → detected_objects, best_object_pose
- **ComputePreGraspPose**: input_pose, z_offset_m → output_pose (sync, no action server)
- **SetPose**: x, y, z, frame_id → pose (sync, no action server)
- **ScriptCondition**: code (BT.CPP built-in condition evaluator)

## Write the BT XML

Create `src/robot_behaviors/trees/<skill_name>.xml` in BehaviorTree.CPP v4 format.

Structure:
```xml
<root BTCPP_format="4" main_tree_to_execute="SkillName">
  <BehaviorTree ID="SkillName">
    <!-- Use Sequence for ordered steps -->
    <Sequence name="main">
      <!-- Steps go here -->
    </Sequence>
  </BehaviorTree>
</root>
```

Key patterns:
- Use `{variable_name}` for blackboard data flow between nodes
- Use `<SubTree ID="...">` to reference other trees defined in the same file
- Use `<RetryUntilSuccessful num_attempts="N">` around unreliable steps
- Use `<Fallback>` for try-alternative patterns
- Use `<ScriptCondition code="var == true"/>` for conditional execution

## Test the tree directly

```bash
# Write the XML to a temp file in the container
docker exec ros2_robot_skills_lite bash -c "cat > /tmp/test_compound.xml << 'EOF'
<the xml>
EOF
source /opt/ros/jazzy/setup.bash && source /opt/bt_ros2_ws/install/setup.bash && source /home/ws/install/setup.bash && \
timeout 30 /home/ws/install/robot_skill_server/lib/robot_skill_server/bt_runner --tree-file /tmp/test_compound.xml --tick-rate 10.0 --task-id ttest1"
```

Verify it completes with SUCCESS. If it fails, read the error output to identify which node failed and why (missing ports, wrong action server, blackboard key not set, etc.).

## Optionally register as a persistent compound skill

If the user wants the skill to appear in the registry and be reusable:

```bash
ros2 service call /skill_server/register_compound_skill \
  robot_skills_msgs/srv/RegisterCompoundSkill "{
    skill_description: {
      name: '<skill_name>',
      display_name: '<Display Name>',
      description: '<description>',
      category: 'compound',
      tags: ['compound', '<relevant_tags>'],
      is_compound: true,
      component_skills: ['<atom1>', '<atom2>']
    },
    bt_xml: '<the full xml string>',
    persist: true
  }"
```

## Optionally add as a dashboard preset

If this is a commonly used task, add it to the PRESET_TREES array in `src/robot_dashboard/frontend/src/components/executor/BehaviorExecutorPanel.tsx`:

```typescript
{
  name: "SkillName",
  label: "Display Name",
  description: "What this compound skill does",
  xml: `<the full xml>`,
},
```

Then rebuild the frontend:
```bash
cd src/robot_dashboard/frontend && npx vite build
docker exec ros2_robot_skills_lite bash -c "source /opt/ros/jazzy/setup.bash && cd /home/ws && colcon build --symlink-install --packages-select robot_dashboard"
```
