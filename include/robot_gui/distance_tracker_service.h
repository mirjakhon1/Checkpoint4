#pragma once

#include <ros/ros.h>
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Trigger.h"
#include "robot_gui/distance_tracker_service.h"

class DistanceTrackerService{
  public:
    DistanceTrackerService(ros::NodeHandle *nh);
  private:
    bool distance_server_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    ros::Subscriber sub_odom_data_;
    nav_msgs::Odometry odom_data;
    ros::ServiceServer distance_service;
    geometry_msgs::Pose prev_pose_;
    double distance_;

    std::string formatFloatToString(float f) {
      std::ostringstream out;
      out << std::fixed << std::setprecision(2) << f;
      return out.str();
    }
};
