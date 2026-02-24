#ifndef RM_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define RM_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include <string>
#include "behaviortree_cpp/basic_types.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace BT
{

/** Trim whitespace, then parse as double (handles " 0" from "x;y;z; qx;qy;qz;qw") */
inline double parseDouble(const StringView & sv)
{
  std::string s(sv.begin(), sv.end());
  size_t start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return 0.0;
  size_t end = s.find_last_not_of(" \t\r\n");
  return std::stod(s.substr(start, (end == std::string::npos ? s.size() : end + 1) - start));
}

/**
 * @brief Parse XML string to geometry_msgs::msg::PoseStamped
 * @param key XML string, format: x;y;z; qx;qy;qz;qw (space after z ok)
 * @return geometry_msgs::msg::PoseStamped
 */
template <>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key)
{
  auto parts = splitString(key, ';');
  if (parts.size() != 7) {
    throw RuntimeError("invalid goal_pose format, expected x;y;z; qx;qy;qz;qw");
  }
  geometry_msgs::msg::PoseStamped msg;
  msg.pose.position.x = parseDouble(parts[0]);
  msg.pose.position.y = parseDouble(parts[1]);
  msg.pose.position.z = parseDouble(parts[2]);
  msg.pose.orientation.x = parseDouble(parts[3]);
  msg.pose.orientation.y = parseDouble(parts[4]);
  msg.pose.orientation.z = parseDouble(parts[5]);
  msg.pose.orientation.w = parseDouble(parts[6]);
  return msg;
}

}  // namespace BT

#endif  // RM_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
