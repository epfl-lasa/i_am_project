#include "ros/ros.h"
#include <ros/console.h>

#include <sstream>
#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <time.h>

using namespace std;

// Now we can be in different modes of the control system and we need to identify which state we are in.

// We are talking about the

int main (int argc, char** argv){


	//ROS Initialization
  ros::init(argc, argv, "drop_object");
  ros::NodeHandle nh;


  ros::Publisher pubCommand; // publishes to robot command topic
  std_msgs::Float64MultiArray pubPose; // pose data to be published

  pubCommand = nh.advertise<std_msgs::Float64MultiArray>("/iiwa/PositionController/command", 1);

  ros::Rate rate(200);

  pubPose.data.clear();
  pubPose.data.push_back(1.0);
  pubPose.data.push_back(1.0);
  pubPose.data.push_back(1.0);
  pubPose.data.push_back(1.0);
  pubPose.data.push_back(0.0);
  pubPose.data.push_back(0.0);
  pubPose.data.push_back(0.0);

  while(ros::ok()){
	pubCommand.publish(pubPose);
	ros::spinOnce();
	rate.sleep();
  }
  return 0;


}
