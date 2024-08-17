/*
 * scout_messenger.hpp
 *
 * Created on: Jun 14, 2019 10:24
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MESSENGER_HPP
#define SCOUT_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "scout_msgs/msg/scout_status.hpp"
#include "scout_msgs/msg/scout_light_cmd.hpp"

#include "ugv_sdk/mobile_robot/scout_robot.hpp"


#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace westonrobot {

class ScoutMessenger {

 public:
  ScoutMessenger(std::shared_ptr<westonrobot::ScoutRobot> scout, rclcpp::Node *node)
      : scout_(scout), node_(node) {}

  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }

  std::shared_ptr<westonrobot::ScoutRobot> scout_;
  rclcpp::Node *node_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<scout_msgs::msg::ScoutStatus>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<scout_msgs::msg::ScoutLightCmd>::SharedPtr
      light_cmd_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // speed variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;

  nav_msgs::msg::Odometry odom_;

  void SetSimulationMode(int loop_rate);
  void SetupSubscription();
  void PublishStateToROS(nav_msgs::msg::Odometry& imu_odom);
  void lightON();
  void lightOFF();
  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  template <typename T,
          std::enable_if_t<!std::is_base_of<ScoutMiniOmniRobot, T>::value,
                            bool> = true>
  void SetScoutMotionCommand(std::shared_ptr<T> base, const geometry_msgs::msg::Twist::SharedPtr &msg);

  template <typename T,
          std::enable_if_t<std::is_base_of<ScoutMiniOmniRobot, T>::value,
                            bool> = true>
  void SetScoutMotionCommand(std::shared_ptr<T> base,
                            const geometry_msgs::msg::Twist::SharedPtr &msg);

  void LightCmdCallback(const scout_msgs::msg::ScoutLightCmd::SharedPtr msg);
  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
  void PublishOdometryToROS3D(const MotionStateMessage &msg, double dt, nav_msgs::msg::Odometry& imu_odom);
  void PublishOdometryToROS(const MotionStateMessage &msg, double dt);

};
}  // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */
