#include "rm_behavior_tree/plugins/action/robot_control.hpp"

namespace rm_behavior_tree
{

RobotControlAction::RobotControlAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<rm_decision_interfaces::msg::RobotControl>(name, conf, params)
{
}

bool RobotControlAction::setMessage(rm_decision_interfaces::msg::RobotControl & msg)
{
  msg.stop_gimbal_scan = false;
  msg.chassis_spin_vel = 0.0f;
  msg.scan_enabled = false;
  msg.allow_vision_control = false;
  msg.search_when_target_lost = false;
  msg.scan_yaw_rate_deg_s = 0.0f;
  msg.search_pitch_deg = 0.0f;

  getInput("stop_gimbal_scan", msg.stop_gimbal_scan);
  getInput("chassis_spin_vel", msg.chassis_spin_vel);
  getInput("scan_enabled", msg.scan_enabled);
  getInput("allow_vision_control", msg.allow_vision_control);
  getInput("search_when_target_lost", msg.search_when_target_lost);
  getInput("scan_yaw_rate_deg_s", msg.scan_yaw_rate_deg_s);
  getInput("search_pitch_deg", msg.search_pitch_deg);

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RobotControlAction, "RobotControl");
