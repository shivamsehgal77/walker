/**
 * @file main.cpp
 * @author Shivam Sehgal (ssehgal7@umd.edu)
 * @brief Walker node for ROS2
 * @version 0.1
 * @date 2023-11-24
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <walker/walker.hpp>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}
