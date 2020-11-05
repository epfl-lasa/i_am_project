#include "ros/ros.h"
#include <ros/console.h>

#include "math.h"
#include <sstream>
#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>

#include <sstream>
#include <time.h>

using namespace std;


// Let us say that I am already in the position controller mode and then i try to just move the system to a specific point

geometry_msgs::Pose box_pose, iiwa_pose;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void objectPositionCallback(const gazebo_msgs::ModelStates model_states){
  int box_index = getIndex(model_states.name, "box_model");

  box_pose = model_states.pose[box_index];
}


void iiwaPositionCallback(const gazebo_msgs::LinkStates link_states){
  int iiwa_index = getIndex(link_states.name, "iiwa::iiwa_link_7");

  iiwa_pose = link_states.pose[iiwa_index];
}


int main (int argc, char** argv){

	std::vector<float> desiredVelocity;

	//ROS Initialization
  ros::init(argc, argv, "hitting");
  ros::NodeHandle nh;

  ros::Subscriber object_position = nh.subscribe("/gazebo/model_states", 100 , objectPositionCallback);
  ros::Subscriber iiwa_position = nh.subscribe("/gazebo/link_states", 100, iiwaPositionCallback);

  ros::Publisher pubCommand; // publishes to robot command topic
  std_msgs::Float64MultiArray pubPose; // pose data to be published

  pubCommand = nh.advertise<std_msgs::Float64MultiArray>("/iiwa/CustomControllers/command", 1);

  // // first reach the attractor

  float vel_x, vel_y, vel_z;


  while(ros::ok()){

    //calculation of desired velocities now


    if(box_pose.position.x - iiwa_pose.position.x > 0.4){
      vel_x = 1.0f*(box_pose.position.x - iiwa_pose.position.x - 0.4);
      vel_y = 1.0f*(box_pose.position.y - iiwa_pose.position.y);
      vel_z = 1.0f*(box_pose.position.z - iiwa_pose.position.z);
    }
    else{
      vel_x = 3.0f;
      vel_y = 0.0f;
      vel_z = 0.0f;
    }
    
    
    // calculate the desired velocity depending on where the object is

    ros::Rate rate(200);

    pubPose.data.clear();
    pubPose.data.push_back(0.0);
    pubPose.data.push_back(0.0);
    pubPose.data.push_back(0.0);
    pubPose.data.push_back(vel_x);
    pubPose.data.push_back(vel_y);
    pubPose.data.push_back(vel_z);

    //std::cout << "x velocity: " << box_pose.position.x << std::endl;

    pubCommand.publish(pubPose);
    ros::spinOnce();
    rate.sleep();
  
  }

  return 0;


}
