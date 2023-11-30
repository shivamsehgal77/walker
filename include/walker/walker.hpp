/**
 * @file walker.hpp
 * @author
 * @brief
 * @version 0.1
 * @date 2023-11-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
/**
 * @brief Walker class inheriting the rclcpp:Node class
 *
 */
class Walker : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Walker object
   *
   */
  Walker() : Node("walker"), collision_distance_(0.3) {
    RCLCPP_INFO(this->get_logger(), "Setting up publisher and subcriber");
    velocity_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Walker::scan_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Walker Node Initialized!");
  }

 private:
  /**
   * @brief callback for laser scan
   *
   * @param laser_scan_msg
   */
  void scan_callback(
      const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) const;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  double collision_distance_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
};
