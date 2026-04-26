#pragma once

// Helper for publishing a latched SkillManifest from any rclcpp Node.
//
// Used by SkillBase (and the future per-robot proxy nodes) to expose
// skill metadata on the standard `<node_fqn>/skills` topic. QoS matches
// `robot_skill_server.skill_advertiser.make_skills_qos()` on the Python
// side — change both together or DDS will silently refuse to connect.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_skills_msgs/msg/skill_advertisement.hpp"
#include "robot_skills_msgs/msg/skill_manifest.hpp"

namespace robot_skill_atoms
{

inline rclcpp::QoS make_skills_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();
  rmw_qos_liveliness_policy_t liveliness =
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  qos.liveliness(liveliness);
  qos.liveliness_lease_duration(std::chrono::seconds(10));
  return qos;
}

class SkillAdvertiser
{
public:
  using Manifest = robot_skills_msgs::msg::SkillManifest;
  using Advertisement = robot_skills_msgs::msg::SkillAdvertisement;

  SkillAdvertiser(rclcpp::Node * node, std::vector<Advertisement> skills)
  : node_(node), skills_(std::move(skills))
  {
    pub_ = node_->create_publisher<Manifest>("~/skills", make_skills_qos());
    publish();
  }

  void set_skills(std::vector<Advertisement> skills)
  {
    skills_ = std::move(skills);
    publish();
  }

  void add(Advertisement skill)
  {
    skills_.push_back(std::move(skill));
    publish();
  }

  void republish() { publish(); }

private:
  void publish()
  {
    Manifest msg;
    msg.skills = skills_;
    msg.published_at = node_->get_clock()->now();
    msg.source_node = node_->get_fully_qualified_name();
    pub_->publish(msg);
  }

  rclcpp::Node * node_;
  std::vector<Advertisement> skills_;
  rclcpp::Publisher<Manifest>::SharedPtr pub_;
};

}  // namespace robot_skill_atoms
