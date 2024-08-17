/*
 * scout_base_ros.cpp
 *
 * Created on: Oct 15, 2021 14:35
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "scout_base/scout_base_ros.hpp"

#include "ugv_sdk/utilities/protocol_detector.hpp"

#include "crc_table.cpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "scout_base/scout_messenger.h"
namespace westonrobot {
ScoutBaseRos::ScoutBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false), is_can_initialized_(false) {
  this->declare_parameter("port_name", rclcpp::ParameterValue("can0"));

  this->declare_parameter("odom_frame", rclcpp::ParameterValue("odom"));
  this->declare_parameter("base_frame", rclcpp::ParameterValue("base_link"));
  this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));

  this->declare_parameter("is_scout_mini", rclcpp::ParameterValue(false));
  this->declare_parameter("is_omni_wheel", rclcpp::ParameterValue(false));

  this->declare_parameter("simulated_robot", rclcpp::ParameterValue(false));
  this->declare_parameter("control_rate", rclcpp::ParameterValue(50));

  LoadParameters();
  
  // declare parametes
  this->declare_parameter("debug", false);
  this->declare_parameter("serial_port", "/dev/ttyACM0");
  this->declare_parameter("serial_baud", 460800);
  this->declare_parameter("serial_timeout", 20);

  // get parameters
  serial_port_ = this->get_parameter("serial_port").as_string();
  serial_baud_ = this->get_parameter("serial_baud").as_int();
  serial_timeout_ = this->get_parameter("serial_timeout").as_int();

  std::vector<rclcpp::Parameter> all_new_parameters{ \
    rclcpp::Parameter("debug", false), \
    rclcpp::Parameter("serial_port", "/dev/ttyACM0"), \
    rclcpp::Parameter("serial_baud", 460800), \
    rclcpp::Parameter("serial_timeout", 20), \
    rclcpp::Parameter("device_type", 1)
    };

  //@initial end template
  end_template_.clear();
  end_template_[1] = {0xff};
  end_template_[2] = {0xff};
  end_template_[3] = {0xfe};
  end_template_[4] = {0xfe};
  end_template_[5] = {0x0a};//"\n"s
  // """ setup serial """
  try
  {
    {
      serial_.setPort(serial_port_.c_str());


      serial_.setBaudrate(serial_baud_);
      serial_.setFlowcontrol(serial::flowcontrol_none);
      serial_.setParity(serial::parity_none);
      serial_.setStopbits(serial::stopbits_one);
      serial_.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
      serial_.setTimeout(time_out);
      serial_.open();
    }
  }
  catch(serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unable to open serial port " << serial_port_.c_str());
    exit(0);
  }
  if (serial_.isOpen())
  {
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Initialized Serial port " << serial_port_.c_str());
    serial_.flush();
  }
  else
  {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unable to initialize serial port " << serial_port_.c_str());
  }
  
  initial_imu_odom_vector_.clear();
  imu_polling_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  can_polling_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("imu_odom", 1);
  imu_polling_timer_ = this->create_wall_timer(5ms, std::bind(&ScoutBaseRos::read_serial, this), imu_polling_group_);
  can_polling_timer_ = this->create_wall_timer(33ms, std::bind(&ScoutBaseRos::Run, this), can_polling_group_);
  
}

float ScoutBaseRos::convert2Float(std::vector<uint8_t>& m_data, int index_start){

  uint8_t indata[4] = { 0 };
  indata[0] = m_data[index_start];
  indata[1] = m_data[index_start+1];
  indata[2] = m_data[index_start+2];
  indata[3] = m_data[index_start+3];

  float g;
  memcpy(&g, &indata, sizeof(g));
  return g;
  
}

void ScoutBaseRos::read_serial()
{
  RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Starting to read Serial\n" );

  if (!serial_.isOpen()){
    RCLCPP_WARN_STREAM_SKIPFIRST(this->get_logger(), "Serial is not open");
  }
  std::unique_lock<std::mutex> lock(serial_lock_);
  // check head start
  bool get_end = false;
  std::map<int, uint8_t> sliding_window;
  std::vector<uint8_t> data;
  sliding_window.clear();
  data.clear();
  while(rclcpp::ok() && !get_end){
    uint8_t one_btye_data[1] = {0xff};
    size_t head_s = serial_.read(one_btye_data, 1);
    sliding_window[1] = sliding_window[2];
    sliding_window[2] = sliding_window[3];
    sliding_window[3] = sliding_window[4];
    sliding_window[4] = sliding_window[5];
    sliding_window[5] = one_btye_data[0];
    data.push_back(one_btye_data[0]);
    if(sliding_window == end_template_){
      //@ check head, head is 255
      uint8_t check_head[1] = {0xff};
      if(data.front() == check_head[0]){
        //RCLCPP_INFO(this->get_logger(), "We get complete string.");
        get_end = true;
        //@ sanity check
        uint8_t ox_head[2] = {0x6f,0x78};
        uint8_t ax_head[2] = {0x61,0x78};
        int trim_index = 0;
        for(int i=0;i<data.size()-1;i++){
          if(data[i]==ox_head[0] && data[i+1]==ox_head[1]){
            trim_index = i;
            break;
          }
        }
        // check ax
        if(data[trim_index+24]==ax_head[0] && data[trim_index+25]==ax_head[1]){
          imu_odom_.header.frame_id = "base_link";
          imu_odom_.pose.pose.orientation.x = convert2Float(data, trim_index+2);
          imu_odom_.pose.pose.orientation.y = convert2Float(data, trim_index+8);
          imu_odom_.pose.pose.orientation.z = convert2Float(data, trim_index+14);
          imu_odom_.pose.pose.orientation.w = convert2Float(data, trim_index+20);
          imu_odom_.twist.twist.angular.x = convert2Float(data, trim_index+26);
          imu_odom_.twist.twist.angular.y = convert2Float(data, trim_index+32);
          imu_odom_.twist.twist.angular.z = convert2Float(data, trim_index+38);
          pub_imu_odom_->publish(imu_odom_);
          if(initial_imu_odom_vector_.size()<100){
            initial_imu_odom_vector_.push_back(imu_odom_);
          }
        }
        else
          return;

        /*
        RCLCPP_INFO(this->get_logger(), "px: %.8f, py: %.8f, pz: %.8f", px_value, py_value, pz_value);
        RCLCPP_INFO(this->get_logger(), "ox: %.8f, oy: %.8f, oz: %.8f, ow: %.8f", ox_value, oy_value, oz_value, ow_value);
        */
      }
      
    }

  }
}



void ScoutBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");

  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
                                      "odom");

  this->get_parameter_or<bool>("is_scout_mini", is_scout_mini_, false);
  this->get_parameter_or<bool>("is_omni_wheel", is_omni_wheel_, false);

  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;

  std::cout << "- is scout mini: " << std::boolalpha << is_scout_mini_
            << std::endl;
  std::cout << "- is omni wheel: " << std::boolalpha << is_omni_wheel_
            << std::endl;

  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool ScoutBaseRos::Initialize() {
  if (is_scout_mini_) {
    std::cout << "Robot base: Scout Mini" << std::endl;
  } else {
    std::cout << "Robot base: Scout" << std::endl;
  }

  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V1) {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
      if (!is_omni_wheel_) {
        is_omni_ = false;
        robot_ = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V1,
                                              is_scout_mini_);
        if (is_scout_mini_) {
          std::cout << "Creating interface for Scout Mini with AGX_V1 Protocol"
                    << std::endl;
        } else {
          std::cout << "Creating interface for Scout with AGX_V1 Protocol"
                    << std::endl;
        }
      } else {
        is_omni_ = true;
        omni_robot_ = std::unique_ptr<ScoutMiniOmniRobot>(
            new ScoutMiniOmniRobot(ProtocolVersion::AGX_V1));
        std::cout
            << "Creating interface for Scout Mini Omni with AGX_V1 Protocol"
            << std::endl;
      }
    } else if (proto == ProtocolVersion::AGX_V2) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      if (!is_omni_wheel_) {
        is_omni_ = false;
        robot_ = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V2,
                                              is_scout_mini_);
        std::cout << "Creating interface for Scout with AGX_V2 Protocol"
                  << std::endl;
      } else {
        is_omni_ = true;
        omni_robot_ = std::unique_ptr<ScoutMiniOmniRobot>(
            new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));
        std::cout
            << "Creating interface for Scout Mini Omni with AGX_V2 Protocol"
            << std::endl;
      }
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void ScoutBaseRos::averageInitialIMUData(){

  double total_r = 0;
  double total_p = 0;
  double total_y = 0;
  for(auto i=initial_imu_odom_vector_.begin();i!=initial_imu_odom_vector_.end();i++){
    tf2::Quaternion tf2_quaternion((*i).pose.pose.orientation.x, (*i).pose.pose.orientation.y, (*i).pose.pose.orientation.z, (*i).pose.pose.orientation.w);
    tf2::Matrix3x3 m(tf2_quaternion);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    total_r+=roll;
    total_p+=pitch;
    total_y+=yaw;
  }
  total_r /=initial_imu_odom_vector_.size();
  total_p /=initial_imu_odom_vector_.size();
  total_y /=initial_imu_odom_vector_.size();
  tf2::Quaternion q;
  q.setRPY(total_r, total_p, total_y);
  tf2_imu_odom_initial_avergae_.setRotation(q);
  tf2_imu_odom_initial_avergae_inverse_ = tf2_imu_odom_initial_avergae_.inverse();
}

nav_msgs::msg::Odometry ScoutBaseRos::getCalibratedIMU(){

  tf2::Transform tf2_imu_odom;
  tf2_imu_odom.setRotation(tf2::Quaternion(
          imu_odom_.pose.pose.orientation.x, imu_odom_.pose.pose.orientation.y, 
          imu_odom_.pose.pose.orientation.z, imu_odom_.pose.pose.orientation.w));

  tf2::Transform tf2_calibrated_imu_odom;
  tf2_calibrated_imu_odom.mult(tf2_imu_odom_initial_avergae_inverse_, tf2_imu_odom);
  nav_msgs::msg::Odometry calibrated_imu_odom;
  calibrated_imu_odom.pose.pose.orientation.x = tf2_calibrated_imu_odom.getRotation().x();
  calibrated_imu_odom.pose.pose.orientation.y = tf2_calibrated_imu_odom.getRotation().y();
  calibrated_imu_odom.pose.pose.orientation.z = tf2_calibrated_imu_odom.getRotation().z();
  calibrated_imu_odom.pose.pose.orientation.w = tf2_calibrated_imu_odom.getRotation().w();
  calibrated_imu_odom.twist = imu_odom_.twist;
  return calibrated_imu_odom;
}

void ScoutBaseRos::Run() {

  if(!is_can_initialized_){
    messenger_ = std::make_shared<westonrobot::ScoutMessenger>(robot_, this);
    messenger_->SetOdometryFrame(odom_frame_);
    messenger_->SetBaseFrame(base_frame_);
    messenger_->SetOdometryTopicName(odom_topic_name_);
    if (simulated_robot_) messenger_->SetSimulationMode(sim_control_rate_);
    // connect to robot and setup ROS subscription
    if (port_name_.find("can") != std::string::npos) {
      if (robot_->Connect(port_name_)) {
        robot_->EnableCommandedMode();
        std::cout << "Using CAN bus to talk with the robot" << std::endl;
      } else {
        std::cout << "Failed to connect to the robot CAN bus" << std::endl;
        return;
      }
    } else {
      std::cout << "Please check the specified port name is a CAN port"
                << std::endl;
      return;
    }
    messenger_->SetupSubscription();
    is_can_initialized_=true;
  }
  std::unique_lock<std::mutex> lock(serial_lock_);
  averageInitialIMUData();
  nav_msgs::msg::Odometry calibrated_imu_odom = getCalibratedIMU();
  messenger_->PublishStateToROS(calibrated_imu_odom);

}
}  // namespace westonrobot
