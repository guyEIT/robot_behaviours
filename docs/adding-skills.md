# Adding Skills to the Robot Skills Framework

This guide covers how to add new capabilities to the robot. There are several levels of abstraction:

| What | Language | When to use |
|------|----------|-------------|
| **Skill atom** | Python or C++ | A new primitive capability (e.g., "move arm", "read sensor") |
| **Mock skill atom** | C++ | Simulated version of an atom for testing without hardware |
| **BT node** | C++ | Wraps a skill atom so it can be used in behavior trees |
| **Compound skill** | XML or service call | Combines existing skills into a reusable sequence |

The typical workflow for a completely new skill: define the action interface, write the atom, add a mock, create the BT node, then use it in trees.

---

## 1. Define the action interface

Every skill communicates via a ROS2 action. Define one in `src/robot_skills_msgs/action/`.

**Example:** `src/robot_skills_msgs/action/GripperControl.action`

```
# Goal
string command              # "open", "close", "set_position"
float64 position            # 0.0=fully closed, 1.0=fully open
float64 force_limit         # Max force in Newtons, 0.0 = default
float64 speed               # 0.0-1.0
---
# Result
bool success
string message
float64 final_position
bool object_grasped
---
# Feedback
float64 current_position
float64 current_force
```

Three sections separated by `---`: what the caller sends (Goal), what it gets back (Result), and what it receives during execution (Feedback). Always include `bool success` and `string message` in the Result.

Register the new file in `src/robot_skills_msgs/CMakeLists.txt`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... existing actions ...
  "action/YourNewSkill.action"
  # ... existing msgs/srvs ...
)
```

Rebuild `robot_skills_msgs` before anything else — other packages depend on it.

---

## 2. Write a skill atom

### Python (recommended for most skills)

A skill atom is a ROS2 action server. Python is the simplest path.

```python
# src/robot_arm_skills/robot_arm_skills/your_skill.py

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from robot_skills_msgs.action import YourNewSkill


class YourSkillNode(Node):
    def __init__(self):
        super().__init__("your_skill_node")

        self._action_server = ActionServer(
            self,
            YourNewSkill,
            "/skill_atoms/your_skill",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )
        self.get_logger().info("YourSkill action server started")

    def _on_goal(self, goal_request):
        # Validate the goal. Return ACCEPT or REJECT.
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        goal = goal_handle.request
        result = YourNewSkill.Result()
        feedback = YourNewSkill.Feedback()

        # --- Do the actual work ---
        # Publish feedback during long operations:
        feedback.progress = 0.5
        feedback.status_message = "Halfway there"
        goal_handle.publish_feedback(feedback)

        # Check for cancellation:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "Cancelled"
            return result

        # --- Done ---
        result.success = True
        result.message = "Completed"
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = YourSkillNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

Add a console script entry point in `src/robot_arm_skills/setup.py` (or `setup.cfg`):

```python
entry_points={
    "console_scripts": [
        "your_skill_node = robot_arm_skills.your_skill:main",
    ],
},
```

Then add it to the launch file so it starts with the system.

### C++ (for performance-critical or hardware-driver skills)

Inherit from `SkillBase<ActionT>` in `src/robot_arm_skills/include/robot_arm_skills/skill_base.hpp`. This base class handles:
- Action server lifecycle
- Automatic registration with the skill registry
- Diagnostic publishing
- Precondition checking

```cpp
#include "robot_arm_skills/skill_base.hpp"
#include "robot_skills_msgs/action/your_new_skill.hpp"

class YourSkill
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::YourNewSkill>
{
public:
  using YourNewSkill = robot_skills_msgs::action::YourNewSkill;
  using Base = robot_arm_skills::SkillBase<YourNewSkill>;

  explicit YourSkill(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("your_skill_node", "/skill_atoms/your_skill", options)
  {
    // Declare parameters, initialize hardware, etc.
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "your_skill";
    desc.display_name = "Your Skill";
    desc.description = "What this skill does, in plain language.";
    desc.category = "motion";  // motion | perception | manipulation | utility
    desc.action_type = "robot_skills_msgs/action/YourNewSkill";
    return desc;
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<YourNewSkill::Result>();
    const auto & goal = goal_handle->get_goal();

    // Do the work, publish feedback, check for cancellation.

    result->success = true;
    result->message = "Done";
    return result;
  }
};
```

Register in `src/robot_arm_skills/CMakeLists.txt`:

```cmake
add_executable(your_skill_node src/your_skill.cpp)
ament_target_dependencies(your_skill_node rclcpp robot_skills_msgs ...)
install(TARGETS your_skill_node DESTINATION lib/${PROJECT_NAME})
```

---

## 3. Add a mock skill atom

Mock atoms let you test behavior trees without real hardware. They live in `src/robot_mock_skill_atoms/`.

A mock should:
- Accept the same action interface as the real skill
- Simulate execution with a configurable delay
- Return plausible results (e.g., mock detection returns a fixed pose)
- Publish joint targets to `/mock/joint_target` or `/mock/gripper_target` so the TF viewer shows the robot moving

```cpp
// src/robot_mock_skill_atoms/src/mock_your_skill.cpp

class MockYourSkill
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::YourNewSkill>
{
public:
  explicit MockYourSkill(const rclcpp::NodeOptions & options)
  : Base("mock_your_skill", "/skill_atoms/your_skill", options)
  {
    this->declare_parameter("your_skill_delay", 1.0);
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Result>();
    double delay = this->get_parameter("your_skill_delay").as_double();

    // Simulate with feedback
    auto feedback = std::make_shared<Feedback>();
    int steps = 10;
    for (int i = 1; i <= steps; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled";
        return result;
      }
      feedback->progress = static_cast<double>(i) / steps;
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(
        std::chrono::duration<double>(delay / steps));
    }

    result->success = true;
    result->message = "[MOCK] Your skill completed";
    return result;
  }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockYourSkill)
```

Register in `src/robot_mock_skill_atoms/CMakeLists.txt`:

```cmake
add_library(mock_your_skill SHARED src/mock_your_skill.cpp)
ament_target_dependencies(mock_your_skill ${MOCK_DEPS})
rclcpp_components_register_node(mock_your_skill
  PLUGIN "robot_mock_skill_atoms::MockYourSkill"
  EXECUTABLE mock_your_skill_node)
```

Add any configurable parameters to `src/robot_mock_skill_atoms/config/mock_params.yaml`:

```yaml
mock_your_skill:
  ros__parameters:
    your_skill_delay: 1.0
```

Add the node to the lite launch file (`src/robot_skill_server/launch/skill_server_lite.launch.py`).

---

## 4. Create a BT node wrapper

BT nodes wrap skill atoms for use in behavior trees. This part must be C++ because BehaviorTree.CPP is a C++ library.

Add your node class to `src/robot_bt_nodes/include/robot_bt_nodes/bt_skill_nodes.hpp`:

```cpp
class YourSkillNode
  : public BT::RosActionNode<robot_skills_msgs::action::YourNewSkill>
{
public:
  using YourNewSkill = robot_skills_msgs::action::YourNewSkill;

  YourSkillNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<YourNewSkill>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("some_param", "default_value", "Description"),
      BT::OutputPort<std::string>("some_result", "Description"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    goal.some_param = getInput<std::string>("some_param").value_or("default");
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    if (result.result->success) {
      setOutput("some_result", result.result->some_field);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "YourSkill connection/timeout error: %d",
      static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }
};
```

Ports are how data flows in and out of BT nodes via the blackboard:
- `InputPort` reads from the blackboard or from XML attributes
- `OutputPort` writes to the blackboard for downstream nodes to read
- In XML: `some_param="literal_value"` or `some_param="{blackboard_key}"`

Register it in `src/robot_bt_nodes/src/bt_nodes_plugin.cpp` and in `src/robot_skill_server/src/bt_runner.cpp`:

```cpp
// bt_runner.cpp — in the registration section
factory.registerNodeType<robot_bt_nodes::YourSkillNode>(
  "YourSkill", make_params("/skill_atoms/your_skill"));
```

The second argument connects the BT node to the correct ROS2 action server.

---

## 5. Write behavior tree XML

Trees are written in BehaviorTree.CPP v4 XML format.

```xml
<root BTCPP_format="4" main_tree_to_execute="MyTask">
  <BehaviorTree ID="MyTask">
    <Sequence name="my_task_sequence">
      <MoveToNamedConfig name="go_observe"
                         config_name="observe"
                         velocity_scaling="0.5"/>
      <DetectObject name="find_seed"
                    object_class="seed"
                    best_object_pose="{seed_pose}"/>
      <GripperControl name="grasp"
                      command="close"
                      object_grasped="{grasped}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

Key constructs:

| Node | Behaviour |
|------|-----------|
| `<Sequence>` | Runs children left to right. Stops on first failure. |
| `<Fallback>` | Runs children left to right. Stops on first success. |
| `<Parallel>` | Runs children concurrently. |
| `<RetryUntilSuccessful num_attempts="3">` | Retries child up to N times. |
| `<SubTree ID="OtherTree">` | Calls another tree defined in the same file. |
| `<ScriptCondition code="x == true"/>` | Evaluates a blackboard expression. |

Blackboard variables are referenced with curly braces: `target_pose="{seed_pose}"`. A node's output port writes to the blackboard, and a downstream node's input port reads from it.

Save trees in `src/robot_behaviors/trees/`. They can also be generated programmatically (see next section).

---

## 6. Create compound skills

Compound skills combine existing atoms into reusable sequences without writing C++. There are two approaches.

### Via the ComposeTask service

Call `/skill_server/compose_task` to generate BT XML from a list of steps:

```bash
ros2 service call /skill_server/compose_task robot_skills_msgs/srv/ComposeTask "{
  task_name: 'open_and_home',
  task_description: 'Open gripper then move home',
  steps: [
    {skill_name: 'gripper_control', parameters_json: '{\"command\": \"open\"}',
     input_blackboard_keys: [], output_blackboard_keys: [],
     retry_on_failure: false, max_retries: 0,
     condition_expression: '', description: 'Open gripper'},
    {skill_name: 'move_to_named_config', parameters_json: '{\"config_name\": \"home\"}',
     input_blackboard_keys: [], output_blackboard_keys: [],
     retry_on_failure: false, max_retries: 0,
     condition_expression: '', description: 'Move to home'}
  ],
  sequential: true,
  add_precondition_checks: false
}"
```

The response contains valid BT XML. You can execute it directly or register it as a compound skill.

### Register as a reusable compound skill

Call `/skill_server/register_compound_skill` with the generated XML:

```bash
ros2 service call /skill_server/register_compound_skill \
  robot_skills_msgs/srv/RegisterCompoundSkill "{
    skill_description: {
      name: 'open_and_home',
      display_name: 'Open and Home',
      description: 'Opens the gripper and moves the arm to the home configuration.',
      category: 'compound',
      is_compound: true
    },
    bt_xml: '<root ...>...</root>',
    persist: true
  }"
```

Setting `persist: true` saves the skill to disk (`~/.ros2_robot_skills/compound_skills/`). It will be available across restarts. The skill then appears in `GetSkillDescriptions` alongside atom skills and can be used in further compositions.

### Via the dashboard

The Execute panel's "Compose" mode provides a UI for building task steps from registered skills. Select skills, set parameters, click "Compose BT XML", then execute or save.

---

## 7. File checklist

### New atom skill (full stack)

```
src/robot_skills_msgs/action/YourSkill.action          # action definition
src/robot_skills_msgs/CMakeLists.txt                    # register in rosidl_generate_interfaces

src/robot_arm_skills/your_skill.py (or .cpp)           # action server implementation
src/robot_arm_skills/setup.py (or CMakeLists.txt)      # build registration

src/robot_mock_skill_atoms/src/mock_your_skill.cpp      # mock implementation
src/robot_mock_skill_atoms/config/mock_params.yaml      # mock parameters
src/robot_mock_skill_atoms/CMakeLists.txt               # build registration

src/robot_bt_nodes/include/.../bt_skill_nodes.hpp       # BT node class
src/robot_bt_nodes/src/bt_nodes_plugin.cpp              # plugin registration
src/robot_skill_server/src/bt_runner.cpp                # bt_runner registration

src/robot_skill_server/launch/skill_server_lite.launch.py  # add mock node
src/robot_skill_server/launch/skill_server.launch.py       # add real node
```

### New compound skill (no code changes)

```
# Option A: service call
ros2 service call /skill_server/compose_task ...
ros2 service call /skill_server/register_compound_skill ...

# Option B: manual XML
src/robot_behaviors/trees/your_task.xml
```

### New mock only (for existing skill)

```
src/robot_mock_skill_atoms/src/mock_your_skill.cpp
src/robot_mock_skill_atoms/config/mock_params.yaml
src/robot_mock_skill_atoms/CMakeLists.txt
src/robot_skill_server/launch/skill_server_lite.launch.py
```

---

## 8. Build and test

```bash
# Rebuild messages first (other packages depend on them)
pixi run build  # or: docker exec ... colcon build

# Test a specific package
docker exec ros2_robot_skills_lite bash -c \
  "source /opt/ros/jazzy/setup.bash && cd /home/ws && \
   colcon build --packages-select robot_skills_msgs robot_mock_skill_atoms robot_bt_nodes robot_skill_server"

# Restart to pick up changes
pixi run lite-restart

# Verify your skill atom is running
ros2 action list | grep your_skill

# Test the BT node directly
cat > /tmp/test.xml << 'EOF'
<root BTCPP_format="4" main_tree_to_execute="Test">
  <BehaviorTree ID="Test">
    <YourSkill name="test" some_param="value"/>
  </BehaviorTree>
</root>
EOF

ros2 run robot_skill_server bt_runner --tree-file /tmp/test.xml --tick-rate 10 --task-id test1
```
