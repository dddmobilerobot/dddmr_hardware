/*
 * scout_base_node.cpp
 *
 * Created on: Oct 15, 2021 16:20
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "scout_base/scout_base_ros.hpp"


using namespace westonrobot;

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto robot = std::make_shared<ScoutBaseRos>("scout");
  executor.add_node(robot);
  if (robot->Initialize()) {
    executor.spin();
  }

  rclcpp::shutdown();
}