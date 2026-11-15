#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace uncertainty_planning_core
{

using ColorRGBA = std_msgs::msg::ColorRGBA;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

using DisplayFunction = std::function<void(const MarkerArray&)>;

}  // namespace uncertainty_planning_core
