#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_mock_skill_atoms
{

/**
 * @brief Publishes fake /joint_states with smooth interpolation.
 *
 * Subscribes to /mock/joint_target (arm joints) and /mock/gripper_target (fingers)
 * published by mock skill atoms. Interpolates from current to target positions so
 * rosboard shows smooth joint value changes during simulated skill execution.
 */
class MockJointStatePublisher : public rclcpp::Node
{
public:
  explicit MockJointStatePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("mock_joint_state_publisher", options)
  {
    this->declare_parameter("publish_rate", 50.0);
    this->declare_parameter("interpolation_speed", 1.0);  // rad/s per joint

    const double rate = this->get_parameter("publish_rate").as_double();

    // Joint names + initial home positions are parameterised so the same
    // node can drive a Panda, a Meca500, or any other arm from the
    // dashboard's URDF viewer. Defaults match the original Panda 7-DoF
    // shape so existing callers (lite-native panda variant) keep working.
    this->declare_parameter<std::vector<std::string>>(
      "joint_names",
      {
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
        "panda_finger_joint1", "panda_finger_joint2",
      });
    this->declare_parameter<std::vector<double>>(
      "initial_positions",
      {0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398, 0.035, 0.035});
    // arm_joint_count: how many of `joint_names` are arm joints (the
    // /mock/joint_target subscriber drives these). The remainder are
    // gripper/finger joints (driven by /mock/gripper_target). Defaults
    // to 7 to preserve the Panda layout.
    this->declare_parameter<int>("arm_joint_count", 7);

    joint_names_ = this->get_parameter("joint_names").as_string_array();
    auto initial = this->get_parameter("initial_positions").as_double_array();
    if (initial.size() != joint_names_.size()) {
      RCLCPP_WARN(this->get_logger(),
        "initial_positions has %zu entries, joint_names has %zu — "
        "padding with zeros",
        initial.size(), joint_names_.size());
      initial.resize(joint_names_.size(), 0.0);
    }
    const int64_t arm_joint_param =
      this->get_parameter("arm_joint_count").as_int();
    arm_joint_count_ = arm_joint_param > 0
      ? static_cast<size_t>(arm_joint_param)
      : 0;
    if (arm_joint_count_ > joint_names_.size()) {
      arm_joint_count_ = joint_names_.size();
    }
    current_positions_ = initial;
    target_positions_ = current_positions_;

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    // Subscribe to arm joint targets from mock skills. Writes the first
    // `arm_joint_count_` positions; gripper targets fill the rest.
    arm_target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mock/joint_target", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < msg->data.size() && i < arm_joint_count_; ++i) {
          target_positions_[i] = msg->data[i];
        }
        RCLCPP_DEBUG(this->get_logger(), "New arm joint target received");
      });

    // Subscribe to gripper targets from mock gripper skill. Drives the
    // joints after the arm-joint slice — for Panda that's joints[7..8],
    // for a gripperless config (Meca500 base) this is a no-op.
    gripper_target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mock/gripper_target", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const size_t total = target_positions_.size();
        for (size_t i = 0;
             i < msg->data.size() && (arm_joint_count_ + i) < total;
             ++i) {
          target_positions_[arm_joint_count_ + i] = msg->data[i];
        }
        RCLCPP_DEBUG(this->get_logger(), "New gripper target received");
      });

    // Publish timer
    const auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MockJointStatePublisher::publishJointState, this));

    RCLCPP_INFO(this->get_logger(),
      "Mock joint state publisher started at %.0f Hz", rate);
  }

private:
  void publishJointState()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const double speed = this->get_parameter("interpolation_speed").as_double();
    const double rate = this->get_parameter("publish_rate").as_double();
    const double max_step = speed / rate;

    // Interpolate toward target
    for (size_t i = 0; i < current_positions_.size(); ++i) {
      const double diff = target_positions_[i] - current_positions_[i];
      if (std::abs(diff) < max_step) {
        current_positions_[i] = target_positions_[i];
      } else {
        current_positions_[i] += (diff > 0 ? max_step : -max_step);
      }
    }

    // Publish
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position = current_positions_;
    // Zero velocities and efforts
    msg.velocity.resize(joint_names_.size(), 0.0);
    msg.effort.resize(joint_names_.size(), 0.0);

    joint_state_pub_->publish(msg);
  }

  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  std::vector<double> target_positions_;
  size_t arm_joint_count_ = 0;
  std::mutex mutex_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockJointStatePublisher)
