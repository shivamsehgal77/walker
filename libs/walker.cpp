/**
 * @file walker.cpp
 * @author Shivam Sehgal (ssehgal7@umd.edu)
 * @brief Walker node for ROS2
 * @version 0.1
 * @date 2023-11-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <walker/walker.hpp>

void Walker::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) const {
  int16_t start_angle = 45;
  int16_t end_angle = 315;
  geometry_msgs::msg::Twist cmd_vel_msg;
  double scan_max = laser_scan->range_max;
  double min_dist_to_obstacle = scan_max;

  for (int16_t i = 0; i < int16_t(laser_scan->ranges.size()); i++) {
    if (i <= start_angle || i >= end_angle) {
      if (!std::isnan(laser_scan->ranges[i])) {
        double scan_dist = laser_scan->ranges[i];
        if (scan_dist < min_dist_to_obstacle) {
          min_dist_to_obstacle = scan_dist;
        }
      }
    }
  }
  if (min_dist_to_obstacle <= collision_distance_) {
    RCLCPP_WARN(this->get_logger(), "Obstacle at distance less than! %f",
                collision_distance_);
    RCLCPP_INFO(this->get_logger(), "Turning turtlebot to avoid obstacle!");
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = -0.5;
  } else {
    RCLCPP_INFO(this->get_logger(), "Moving forward wiht velocity 0.2!");
    cmd_vel_msg.linear.x = 0.2;
    cmd_vel_msg.angular.z = 0.0;
  }
  velocity_publisher_->publish(cmd_vel_msg);
}
