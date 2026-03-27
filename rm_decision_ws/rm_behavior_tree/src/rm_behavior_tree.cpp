#include "rm_behavior_tree/rm_behavior_tree.h"
#include "rm_behavior_tree/bt_conversions.hpp"

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  std::string bt_xml_path;
  std::string start_goal_pose = "0;0;0; 0;0;0;1";
  std::string end_goal_pose = "10.965;-2.807;0; 0;0;0;1";
  bool enable_groot = true;
  int groot_port = 1667;
  auto node = std::make_shared<rclcpp::Node>("rm_behavior_tree");
  node->declare_parameter<std::string>(
    "style", "./rm_decision_ws/rm_behavior_tree/rm_behavior_tree.xml");
  node->declare_parameter<std::string>("start_goal_pose", start_goal_pose);
  node->declare_parameter<std::string>("end_goal_pose", end_goal_pose);
  node->declare_parameter<bool>("enable_groot", true);
  node->declare_parameter<int>("groot_port", 1667);
  node->get_parameter_or<std::string>(
    "style", bt_xml_path, "./rm_decision_ws/rm_behavior_tree/config/attack_left.xml");
  node->get_parameter_or<std::string>("start_goal_pose", start_goal_pose, start_goal_pose);
  node->get_parameter_or<std::string>("end_goal_pose", end_goal_pose, end_goal_pose);
  node->get_parameter_or<bool>("enable_groot", enable_groot, true);
  node->get_parameter_or<int>("groot_port", groot_port, 1667);

  std::cout << "Start RM_Behavior_Tree" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg");

  BT::RosNodeParams params_robot_control;
  params_robot_control.nh = std::make_shared<rclcpp::Node>("robot_control");
  params_robot_control.default_port_value = "robot_control";

  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
  params_send_goal.default_port_value = "goal_pose";

  // clang-format off
  const std::vector<std::string> msg_update_plugin_libs = {
    "sub_all_robot_hp",
    "sub_robot_status",
    "sub_game_status",
    "sub_armors",
  };

  const std::vector<std::string> bt_plugin_libs = {
    "rate_controller",
    "is_game_time",
    "is_status_ok",
    "is_detect_enemy",
    "is_near_goal",
    "is_attacked",
    "is_friend_ok",
    "is_outpost_ok",
    "get_current_location",
    "move_around",
  };
  // clang-format on

  for (const auto & p : msg_update_plugin_libs) {
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
  }

  for (const auto & p : bt_plugin_libs) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("robot_control"), params_robot_control);

  auto tree = factory.createTreeFromFile(bt_xml_path);
  tree.rootBlackboard()->set(
    "start_goal_pose", BT::convertFromString<geometry_msgs::msg::PoseStamped>(start_goal_pose));
  tree.rootBlackboard()->set(
    "end_goal_pose", BT::convertFromString<geometry_msgs::msg::PoseStamped>(end_goal_pose));

  RCLCPP_INFO(node->get_logger(), "Start goal pose: %s", start_goal_pose.c_str());
  RCLCPP_INFO(node->get_logger(), "End goal pose: %s", end_goal_pose.c_str());

  std::unique_ptr<BT::Groot2Publisher> publisher;
  if (enable_groot) {
    try {
      publisher = std::make_unique<BT::Groot2Publisher>(tree, static_cast<unsigned>(groot_port));
      RCLCPP_INFO(node->get_logger(), "Groot2Publisher enabled on port %d", groot_port);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        node->get_logger(),
        "Failed to start Groot2Publisher on port %d: %s. Continue without Groot.",
        groot_port, e.what());
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Groot2Publisher disabled");
  }

  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}
