//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "../include/momentum_hit.h"
#include <experimental/filesystem>

geometry_msgs::Pose box_pose, iiwa_pose;
geometry_msgs::Twist iiwa_vel;

Eigen::Vector3f object_position_from_source;
Eigen::Vector4d object_orientation_from_source;
Eigen::Vector3f iiwa_position_from_source;
Eigen::Vector3f iiwa_vel_from_source;
Eigen::Vector4d iiwa_orientation_from_source;

using namespace std;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void objectPositionCallback_gazebo(const gazebo_msgs::ModelStates model_states){
  int box_index = getIndex(model_states.name, "box_model");

  box_pose = model_states.pose[box_index];
  object_position_from_source << box_pose.position.x, box_pose.position.y, box_pose.position.z;
  object_orientation_from_source << box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z, box_pose.orientation.w;
}


void iiwaPositionCallback_gazebo(const gazebo_msgs::LinkStates link_states){
  int iiwa_index = getIndex(link_states.name, "iiwa::iiwa_link_7"); // End effector is the 7th link in KUKA IIWA

  iiwa_pose = link_states.pose[iiwa_index];
  iiwa_position_from_source << iiwa_pose.position.x, iiwa_pose.position.y, iiwa_pose.position.z;
  iiwa_vel = link_states.twist[iiwa_index];
}


int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "momentum_hit");
  ros::NodeHandle nh;

  ros::Subscriber object_position = nh.subscribe("/gazebo/model_states", 100 , objectPositionCallback_gazebo);
  
  ros::Publisher pub_vel_quat; // publishes to robot command topic
  geometry_msgs::Pose vel_quat; // we are publishing the desired velocity and the desired orientation

  pub_vel_quat = nh.advertise<geometry_msgs::Pose>("/passive_control/vel_quat", 1);

  ros::Publisher pub_pos_quat; // publishes to robot command topic
  geometry_msgs::Pose pos_quat; // we are publishing the desired velocity and the desired orientation

  pub_pos_quat = nh.advertise<geometry_msgs::Pose>("/passive_control/pos_quat", 1);

  Eigen::Vector3f des_vel = {0.3, 0.0, 0.0};
  
  // double des_speed;
  // std::cout << "Enter the desired speed of hitting (between 0 and 1)" << std::endl;
  // std::cin >> des_speed;

  // while(des_speed < 0 || des_speed > 1){
  //   std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
  //   std::cin >> des_speed;
  // }

  // double theta;
  // std::cout << "Enter the desired direction of hitting (between -pi/2 and pi/2)" << std::endl;
  // std::cin >> theta;

  // while(theta < -M_PI_2 || theta > M_PI_2){
  //   std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
  //   std::cin >> theta;
  // }
  
  Eigen::Quaterniond rot_quat;
  Eigen::Quaterniond rot_quat_ee;
  Eigen::Quaterniond initial_ee_quat;
  Eigen::Vector4d des_quat = Eigen::Vector4d::Zero();
  
  Eigen::Vector3f end_effector_position;
  Eigen::Vector3f object_position_current;

  Eigen::Matrix3d gain_main;
  Eigen::Matrix3d gain_aux;

  Eigen::Vector3f attractor_main;

  gain_main << -0.9, 0.0, 0.0,
                0.0, -0.1, 0.0,
                0.0, 0.0, -0.9;

  
  while(ros::ok()){

    // Initialise all the position in the hitting of objects

    object_position_current = object_position_from_source; 
    end_effector_position = iiwa_position_from_source; 

    vel_quat.position.x = des_vel(0);
    vel_quat.position.y = des_vel(1);
    vel_quat.position.z = des_vel(2);
    vel_quat.orientation.x = des_quat(0);
    vel_quat.orientation.y = des_quat(1);
    vel_quat.orientation.z = des_quat(2);
    vel_quat.orientation.w = des_quat(3);

    pub_vel_quat.publish(vel_quat);

      

    ros::Rate rate(100);

    ros::spinOnce();
    rate.sleep();
  
  }

  

  return 0;


}
