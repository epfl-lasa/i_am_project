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

#include "../include/tracking_velocity.h"

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
  int iiwa_index = getIndex(link_states.name, "iiwa::iiwa_link_7"); // End effector is the 7th link in KUKA IIWA

  iiwa_pose = link_states.pose[iiwa_index];
}


int main (int argc, char** argv){

	std::vector<float> desired_hitting_velocity {10.0f, 0.0f, 0.0f};
  std::vector<float> tracking_velocity {5.0f, 0.0f, 0.0f};
  std::vector<float> end_effector_position;
  std::vector<float> box_position;
  std::vector<float> position_in_line;

  end_effector_position.reserve(3);
  box_position.reserve(3);
  position_in_line.reserve(3);

  float velocityGain = 25.0f;

	//ROS Initialization
  ros::init(argc, argv, "hitting");
  ros::NodeHandle nh;

  ros::Subscriber object_position = nh.subscribe("/gazebo/model_states", 100 , objectPositionCallback);
  ros::Subscriber iiwa_position = nh.subscribe("/gazebo/link_states", 100, iiwaPositionCallback);

  ros::Publisher pubCommand; // publishes to robot command topic
  std_msgs::Float64MultiArray pubPose; // pose data to be published

  pubCommand = nh.advertise<std_msgs::Float64MultiArray>("/iiwa/CustomControllers/command", 1);

  while(ros::ok()){


    ros::Rate rate(100);

    end_effector_position[0] = iiwa_pose.position.x;
    end_effector_position[1] = iiwa_pose.position.y;
    end_effector_position[2] = iiwa_pose.position.z;


    box_position[0] = box_pose.position.x - 0.2f;
    box_position[1] = box_pose.position.y;
    box_position[2] = box_pose.position.z;


    float d = calculate_distance(end_effector_position, box_position);

    std::cout << "distance is" << d << std::endl;

    float hitting_speed = sqrt(std::inner_product(desired_hitting_velocity.begin(), desired_hitting_velocity.end(), desired_hitting_velocity.begin(), 0));
    
    std::cout << "hitting speed" << hitting_speed << std::endl;

    for (int i = 0; i < position_in_line.capacity(); i++){
        position_in_line[i] = box_position[i] - (d/hitting_speed)*desired_hitting_velocity[i];
    }


    // get_position_in_line(position_in_line, box_position, end_effector_position, desired_hitting_velocity);

    std::cout << "position in line after: " << position_in_line[0] << ", " << position_in_line[1] << ", " << position_in_line[2] << std::endl;
  
    //calculation of desired velocities now
    for(unsigned int i = 0; i < tracking_velocity.capacity(); i++){
      tracking_velocity[i] = velocityGain*(position_in_line[i] - end_effector_position[i]) + desired_hitting_velocity[i];
    }
    float tracking_speed = sqrt(std::inner_product(tracking_velocity.begin(), tracking_velocity.end(), tracking_velocity.begin(), 0));
    
    for(unsigned int i = 0; i < tracking_velocity.capacity(); i++){
      tracking_velocity[i] *= hitting_speed/tracking_speed;
    }

    std::cout << "Tracking velocity: " << tracking_velocity[0] << ", " << tracking_velocity[1] << ", " << tracking_velocity[2] << std::endl;

    

    // Sending the tracking velocity commands in the node publisher

    pubPose.data.clear();
    pubPose.data.push_back(0.0);
    pubPose.data.push_back(0.0);
    pubPose.data.push_back(0.0);
    pubPose.data.push_back(tracking_velocity[0]);
    pubPose.data.push_back(tracking_velocity[1]);
    pubPose.data.push_back(tracking_velocity[2]);

    pubCommand.publish(pubPose);
    ros::spinOnce();
    rate.sleep();
  
  }

  return 0;


}
