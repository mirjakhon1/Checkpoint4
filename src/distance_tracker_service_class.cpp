#include "nav_msgs/Odometry.h"
#include "ros/node_handle.h"
#include "robot_gui/distance_tracker_service.h"

DistanceTrackerService::DistanceTrackerService(ros::NodeHandle *nh){
  ROS_INFO("Distance server is running...");

  distance_service = nh->advertiseService("/get_distance", &DistanceTrackerService::distance_server_callback, this);
  sub_odom_data_ = nh->subscribe("/odom", 1, &DistanceTrackerService::odomCallback, this);
}

bool DistanceTrackerService::distance_server_callback(std_srvs::Trigger::Request &req,
                                                    std_srvs::Trigger::Response &res){
  ROS_INFO("Server callback is called.");
  res.success = true;
    // Respond with distance traveled in meters
  res.message = formatFloatToString(distance_);
//   ROS_INFO(res.message);
  return true;
}

void DistanceTrackerService::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // calculate distance traveled using Euclidean distance formula
    double dx = msg->pose.pose.position.x - prev_pose_.position.x;
    double dy = msg->pose.pose.position.y - prev_pose_.position.y;
    double dz = msg->pose.pose.position.z - prev_pose_.position.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    // add distance traveled to total distance
    distance_ += distance;

    // update previous pose
    prev_pose_ = msg->pose.pose;
  }