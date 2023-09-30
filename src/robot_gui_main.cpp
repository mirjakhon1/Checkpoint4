#include "robot_gui/robot_gui.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "main_gui_node");
  
  CVUIROSPublisher button_clicks_publisher;
  button_clicks_publisher.run();

  ros::spin();
  return 0;
}