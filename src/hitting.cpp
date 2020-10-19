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


// Let us say that I am already in the position controller mode and then i try to just move the system to a specific point



int main (int argc, char** argv){


	// Get initial and the final positions of where we want the object to be
	// Assume that there is no friction in the world for now and we can calculate at what velocity we are hitting the object

	std::vector<float> initialPosition;
	std::vector<float> finalPosition;
	std::vector<float> desiredInitialVelocity;
	std::vector<float> hittingVelocity;

	float coeffRestitution;


	//ROS Initialization
    ros::init(argc, argv, "hitting");
    ros::NodeHandle nh;

    nh.getParam("/box/intial_position/", initialPosition);
    nh.getParam("/box/final_position/", finalPosition);



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
