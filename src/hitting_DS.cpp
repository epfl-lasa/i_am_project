//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "../include/hitting_DS.h"
#include "../include/calculate_alpha.h"
#include "../include/nominal_DS_aux.h"
#include "../include/nominal_DS_main.h"
#include "../include/modulated_DS.h"

geometry_msgs::Pose box_pose, iiwa_pose;
geometry_msgs::Twist iiwa_vel;

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
}


void iiwaPositionCallback_gazebo(const gazebo_msgs::LinkStates link_states){
  int iiwa_index = getIndex(link_states.name, "iiwa::iiwa_link_7"); // End effector is the 7th link in KUKA IIWA

  iiwa_pose = link_states.pose[iiwa_index];
  iiwa_vel = link_states.twist[iiwa_index];
}


int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "hitting_DS");
  ros::NodeHandle nh;

  // ros::Subscriber object_position = nh.subscribe("/gazebo/model_states", 100 , objectPositionCallback);
  // ros::Subscriber iiwa_position = nh.subscribe("/gazebo/link_states", 100, iiwaPositionCallback);

  ros::Subscriber object_position = nh.subscribe("/gazebo/model_states", 100 , objectPositionCallback_gazebo);
  ros::Subscriber iiwa_position = nh.subscribe("/gazebo/link_states", 100, iiwaPositionCallback_gazebo);  

  ros::Publisher pub_vel_quat; // publishes to robot command topic
  geometry_msgs::Pose vel_quat; // we are publishing the desired velocity and the desired orientation

  pub_vel_quat = nh.advertise<geometry_msgs::Pose>("/passive_control/vel_quat", 1);

  Eigen::Vector3d des_vel = {0.0, 0.0, 0.0};
  double des_speed = 0.99;
  Eigen::Vector4d des_quat = Eigen::Vector4d::Zero();
  
  Eigen::Vector3d end_effector_position;
  Eigen::Vector3d object_position_current;

  Eigen::Matrix3d gain_main;
  Eigen::Matrix3d gain_aux;
  Eigen::Matrix3d rotation;

  Eigen::Vector3d object_position_init;
  Eigen::Vector3d end_effector_position_init;

  Eigen::Vector3d attractor_main;
  Eigen::Vector3d attractor_aux;

  gain_main << -0.1, 0.0, 0.0,
                0.0, -0.9, 0.0,
                0.0, 0.0, -0.9;


  gain_aux << -3.0, 0.0, 0.0,
                0.0, -3.0, 0.0,
                0.0, 0.0, -3.0;
                
  Eigen::Vector3d desired_final_position;
  Eigen::Vector3d relative_displacement = {2.0, 0.0, 0.0};


  double modulated_sigma = 2.0;
  bool init_flag = 0;
  
  while(ros::ok()){


    // Initialise all the position in the hitting of objects


    if (init_flag == 0){
      object_position_init = {(double)box_pose.position.x, (double)box_pose.position.y, (double)box_pose.position.z};
      end_effector_position_init = {(double)iiwa_pose.position.x, (double)iiwa_pose.position.y, (double)iiwa_pose.position.z};
      
      //std::cout << "iiwa is at: " << end_effector_position_init(0) << ", " << end_effector_position_init(1) << ", " << end_effector_position_init(2) << std::endl;

      desired_final_position = object_position_init + relative_displacement;
      attractor_main = desired_final_position;
      
      attractor_aux = (5/4)*object_position_init - (1/4)*attractor_main;
      if (object_position_init(0)!=0 && object_position_init(1)!=0 && object_position_init(2)!=0 && end_effector_position_init(0) != 0 && end_effector_position_init(1) != 0 && end_effector_position_init(2) != 0){
        init_flag = 1;
      }

    }

    ros::Rate rate(100);
    
    // Create the hitting motion here and init values do not change


    if (init_flag == 1){


      object_position_current << (double)box_pose.position.x ,(double)box_pose.position.y , (double)box_pose.position.z; 
      end_effector_position << (double)iiwa_pose.position.x , (double)iiwa_pose.position.y , (double)iiwa_pose.position.z; 

      //std::cout << "iiwa is now at: " << end_effector_position(0) << ", " << end_effector_position(1) << ", " << end_effector_position(2) << std::endl;

      double theta = atan2(desired_final_position(2) - object_position_init(2), desired_final_position(1) - object_position_init(1));

      rotation << cos(theta), -sin(theta), 0, 
                  sin(theta), cos(theta), 0,
                  0, 0, 1;


      double alpha = calculate_alpha(end_effector_position, end_effector_position_init, object_position_init, attractor_main);

      if ((object_position_current - object_position_init).norm() < 0.1){
        des_vel = alpha*nominal_aux(rotation, gain_aux, end_effector_position, attractor_aux) + (1 - alpha)*nominal_main(rotation, gain_main, end_effector_position, attractor_main)
                  + modulated_DS(attractor_main, object_position_init, end_effector_position, modulated_sigma);

        des_vel = des_vel*des_speed / des_vel.norm();
      }
      else{
        des_vel << 0.0, 0.0, 0.0;
      }
      
      
      vel_quat.position.x = des_vel(0);
      vel_quat.position.y = des_vel(1);
      vel_quat.position.z = des_vel(2);
      vel_quat.orientation.x = des_quat(0);
      vel_quat.orientation.y = des_quat(1);
      vel_quat.orientation.z = des_quat(2);
      vel_quat.orientation.w = des_quat(3);

      pub_vel_quat.publish(vel_quat);

    }
    ros::spinOnce();
    rate.sleep();
  
  }

  return 0;


}
