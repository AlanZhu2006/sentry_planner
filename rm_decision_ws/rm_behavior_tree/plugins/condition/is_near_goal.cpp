#include "rm_behavior_tree/plugins/condition/is_near_goal.hpp"
#include "rm_behavior_tree/bt_conversions.hpp"

#include <cmath>

namespace rm_behavior_tree
{

IsNearGoalAction::IsNearGoalAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsNearGoalAction::checkDistanceToGoal, this), config)
{
}

BT::NodeStatus IsNearGoalAction::checkDistanceToGoal()
{
  auto current_location = getInput<geometry_msgs::msg::TransformStamped>("current_location");
  auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");

  double dist_threshold = 0.3;
  getInput("dist_threshold", dist_threshold);

  if (!current_location || !goal_pose) {
    return BT::NodeStatus::FAILURE;
  }

  const double dx = current_location->transform.translation.x - goal_pose->pose.position.x;
  const double dy = current_location->transform.translation.y - goal_pose->pose.position.y;
  const double distance = std::hypot(dx, dy);

  return distance <= dist_threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsNearGoalAction>("IsNearGoal");
}
