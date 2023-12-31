#pragma once

#include "ros/subscriber.h"
#include "std_srvs/Trigger.h"
#include <string>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "string"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "robot_info/custom.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

class CVUIROSPublisher {
public:
  CVUIROSPublisher();
  void run();
  
private:
  ros::Publisher pub_;
  ros::Publisher pub_cmd_vel_;
  ros::Timer timer;
  ros::Subscriber sub_robot_info_;
  ros::Subscriber sub_odom_data_;
  ros::ServiceClient distance_client;
  

  void robot_info_callback(const robot_info::custom &msg);
  void timerCallback(const ros::TimerEvent& event);
  void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  
  const std::string WINDOW_NAME = "CVUI ROS BUTTON!";
  robot_info::custom robot_info_msg;
  robot_info::custom robot_info_msg_temp;
  nav_msgs::Odometry odom_data;
  geometry_msgs::Twist speed_msg;
  std::string distance_travelled;
};