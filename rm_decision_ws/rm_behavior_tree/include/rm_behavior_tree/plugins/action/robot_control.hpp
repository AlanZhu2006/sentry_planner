#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ROBOT_CONTROL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ROBOT_CONTROL_HPP_

#include <rm_decision_interfaces/msg/detail/robot_control__struct.hpp>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "rm_decision_interfaces/msg/robot_control.hpp"

namespace rm_behavior_tree
{

class RobotControlAction : public BT::RosTopicPubNode<rm_decision_interfaces::msg::RobotControl>
{
public:
  RobotControlAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(rm_decision_interfaces::msg::RobotControl & msg) override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("stop_gimbal_scan"),
      BT::InputPort<float>("chassis_spin_vel"),
      BT::InputPort<bool>("scan_enabled"),
      BT::InputPort<bool>("allow_vision_control"),
      BT::InputPort<bool>("search_when_target_lost"),
      BT::InputPort<float>("scan_yaw_rate_deg_s"),
      BT::InputPort<float>("search_pitch_deg")
    };
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ROBOT_CONTROL_HPP_
