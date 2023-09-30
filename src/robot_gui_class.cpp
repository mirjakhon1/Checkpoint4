
#include "robot_gui/robot_gui.h"
#include "ros/rate.h"
#include <iostream>

CVUIROSPublisher::CVUIROSPublisher() {
  // Initialize ROS node
  ros::NodeHandle nh;
  pub_ = nh.advertise<std_msgs::String>("cvui_button_clicks", 10);
  pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  sub_robot_info_ = nh.subscribe("/robot_info", 100,
                      &CVUIROSPublisher::robot_info_callback, this);
  timer = nh.createTimer(ros::Duration(5.0),  &CVUIROSPublisher::timerCallback, this);
  timer.start();
  
}

// robot info topic subscriber callback, updating temp info var 
void CVUIROSPublisher::robot_info_callback(const robot_info::custom &msg) {
  this->robot_info_msg_temp = msg;
}

// updating gui info var at a specific rate
void CVUIROSPublisher::timerCallback(const ros::TimerEvent& event) {
  this->robot_info_msg = this->robot_info_msg_temp;
}

void CVUIROSPublisher::run() {
  cv::Mat frame = cv::Mat(800, 400, CV_8UC3); // 800 - y; 400 - x
  int count = 0;
  ros::Rate speed_update_rate(1.0);


  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Show a button at position x = 40, y = 80
    if (cvui::button(frame, 150, 190, 90, 70, "Forward")) {
      speed_msg.linear.x += 0.5;
    }

    if (cvui::button(frame, 150, 270, 90, 70, "Stop")) {
      speed_msg.linear.x = 0;
      speed_msg.linear.y = 0;
      speed_msg.angular.x = 0;
      speed_msg.angular.y = 0;
    }

    if (cvui::button(frame, 40, 270, 90, 70, "Left")) {
      speed_msg.angular.z += 0.5;
    }

    if (cvui::button(frame, 260, 270, 90, 70, "Right")) {
      speed_msg.angular.z -= 0.5;
    }

    if (cvui::button(frame, 150, 350, 90, 70, "Backward")) {
      speed_msg.linear.x -= 0.5;
    }

    pub_cmd_vel_.publish(speed_msg);

    // velocity text bar
    cvui::window(frame, 40, 440, 140, 40, "Linear velocity:");
    cvui::window(frame, 200, 440, 140, 40, "Angular velocity");
    cvui::printf(frame, 40, 465, 0.4, 0x220000, "%.2f m/sec", speed_msg.linear.x);
    cvui::printf(frame, 200, 465, 0.4, 0x220000, "%.2f rad/sec", speed_msg.angular.z);

    cvui::printf(frame, 40, 490, 0.4, 0x820000,
                 "Estimated robot position based off odometry");

    // odometry text bar
    cvui::window(frame, 40, 510, 90, 70, "X");
    cvui::window(frame, 150, 510, 90, 70, "Y");
    cvui::window(frame, 260, 510, 90, 70, "Z");

    // Distance travelled
    cvui::printf(frame, 40, 590, 0.4, 0x820000, "Distance travelled");

    if (cvui::button(frame, 40, 610, 90, 70, "Call")) {
      std_msgs::String msg;
      msg.data = "Button clicked " + std::to_string(count) + " times.";
      pub_.publish(msg);
    }

    cvui::window(frame, 150, 610, 200, 70, "Distance in meters: ");

    // Create window at (220, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 40, 20, 300, 160, "Info");

    // Info tab text
    cvui::printf(frame, 40, 45, 0.4, 0x220000, "- %s", robot_info_msg.robot_desc.c_str());
    cvui::printf(frame, 40, 65, 0.4, 0x220000, "- %s", robot_info_msg.serial_num.c_str());
    cvui::printf(frame, 40, 85, 0.4, 0x220000, "- %s", robot_info_msg.ip_address.c_str());
    cvui::printf(frame, 40, 105, 0.4, 0x220000, "- %s", robot_info_msg.firmware_ver.c_str());
    cvui::printf(frame, 40, 125, 0.4, 0x220000, "- %s", robot_info_msg.max_payload.c_str());
    cvui::printf(frame, 40, 145, 0.4, 0x220000, "- %s", robot_info_msg.hydraulic_oil_temperature.c_str());

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    ros::spinOnce();
    speed_update_rate.sleep();
  }
}
