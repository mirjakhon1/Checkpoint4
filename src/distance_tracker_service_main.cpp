#include "robot_gui/distance_tracker_service.h"
#include "ros/node_handle.h"
#include <ros/ros.h>
#include "robot_gui/distance_tracker_service.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "distance_tracker_service_node");
  ros::NodeHandle nh;

  DistanceTrackerService distance_tracker(&nh);
  
  ros::spin();
  return 0;
}