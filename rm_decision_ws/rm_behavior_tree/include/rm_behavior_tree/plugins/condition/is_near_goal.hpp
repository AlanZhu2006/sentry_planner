#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NEAR_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NEAR_GOAL_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{

class IsNearGoalAction : public BT::SimpleConditionNode
{
public:
  IsNearGoalAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkDistanceToGoal();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::TransformStamped>("current_location"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
      BT::InputPort<double>("dist_threshold", 0.3, "distance threshold in meters")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NEAR_GOAL_HPP_
