#pragma once

#include "ros/subscriber.h"
#include <string>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "string"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "robot_info/custom.h"
#include "geometry_msgs/Twist.h"

class CVUIROSPublisher {
public:
  CVUIROSPublisher();

  void run();
  void robot_info_callback(const robot_info::custom &msg);

private:
  ros::Publisher pub_;
  ros::Publisher pub_cmd_vel_;
  ros::Subscriber sub_;
  const std::string WINDOW_NAME = "CVUI ROS BUTTON!";
  robot_info::custom robot_info_msg;
protected:
  geometry_msgs::Twist speed_msg;
};