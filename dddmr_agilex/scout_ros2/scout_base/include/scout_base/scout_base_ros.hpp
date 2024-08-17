/*
 * scout_base_ros.hpp
 *
 * Created on: Oct 15, 2021 14:31
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef SCOUT_BASE_ROS_HPP
#define SCOUT_BASE_ROS_HPP

#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"


#include <cstdio>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include "serial/serial.h"
#include "tf2/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>

#include "scout_base/scout_messenger.h"

using namespace std::chrono_literals;
namespace westonrobot {
class ScoutBaseRos : public rclcpp::Node {

 public:
  ScoutBaseRos(std::string node_name);

  bool Initialize();
  void Run();
  void averageInitialIMUData();
  nav_msgs::msg::Odometry getCalibratedIMU();
 private:
  
  rclcpp::CallbackGroup::SharedPtr imu_polling_group_;
  rclcpp::CallbackGroup::SharedPtr can_polling_group_;

  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool is_scout_mini_ = false;
  bool is_omni_wheel_ = false;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  bool is_omni_ = false;
  std::shared_ptr<ScoutRobot> robot_;
  std::shared_ptr<ScoutMiniOmniRobot> omni_robot_;

  std::atomic<bool> keep_running_;

  void LoadParameters();

  template <typename Map>
    bool map_compare (Map const &lhs, Map const &rhs) {
        // No predicate needed because there is operator== for pairs already.
        return lhs.size() == rhs.size()
            && std::equal(lhs.begin(), lhs.end(),
                        rhs.begin());
    }

  serial::Serial serial_;

  std::string serial_port_;
  std::uint32_t serial_baud_;
  std::uint8_t serial_timeout_;
  
  rclcpp::TimerBase::SharedPtr imu_polling_timer_;
  rclcpp::TimerBase::SharedPtr can_polling_timer_;

  void read_serial();
  std::map<int, uint8_t> end_template_;

  float convert2Float(std::vector<uint8_t>& m_data, int index_start);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom_;
  
  tf2::Transform tf2_imu_odom_initial_avergae_, tf2_imu_odom_initial_avergae_inverse_;
  nav_msgs::msg::Odometry imu_odom_, imu_odom_initial_avergae_;
  bool is_can_initialized_;
  std::shared_ptr<westonrobot::ScoutMessenger> messenger_;
  std::mutex serial_lock_;
  std::vector<nav_msgs::msg::Odometry> initial_imu_odom_vector_;

};
}  // namespace westonrobot

#endif /* SCOUT_BASE_ROS_HPP */
