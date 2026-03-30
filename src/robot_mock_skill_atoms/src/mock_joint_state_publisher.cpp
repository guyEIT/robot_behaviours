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

    // Joint names matching the Panda URDF
    joint_names_ = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7",
      "panda_finger_joint1", "panda_finger_joint2"
    };

    // Start at home configuration
    current_positions_ = {0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398, 0.035, 0.035};
    target_positions_ = current_positions_;

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    // Subscribe to arm joint targets from mock skills
    arm_target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mock/joint_target", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < msg->data.size() && i < 7; ++i) {
          target_positions_[i] = msg->data[i];
        }
        RCLCPP_DEBUG(this->get_logger(), "New arm joint target received");
      });

    // Subscribe to gripper targets from mock gripper skill
    gripper_target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mock/gripper_target", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->data.size() >= 2) {
          target_positions_[7] = msg->data[0];
          target_positions_[8] = msg->data[1];
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
  std::mutex mutex_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockJointStatePublisher)
