//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "../include/hitting_DS.h" 
#include "../include/calculate_alpha.h"
#include "../include/nominal_DS_aux.h"
#include "../include/nominal_DS_main.h"
#include "../include/modulated_DS.h"

#include <experimental/filesystem>

geometry_msgs::Pose box_pose, ee_pose, iiwa_base_pose;
geometry_msgs::Twist box_vel, ee_vel;
Eigen::Vector3d object_pos, object_vel, ee_pos;

double des_speed;
double theta;

Eigen::Vector3d rest_pos = {0.0, -1.0, 0.4};
Eigen::Vector4d rest_quat = {-0.707, 0.0, 0.0, 0.707};

/*
Eigen::Vector3d object_position_from_source;
Eigen::Vector4d object_orientation_from_source;
Eigen::Vector3d object_position_mocap;
Eigen::Vector4d object_orientation_mocap;
Eigen::Vector3d iiwa_base_position_from_source;
Eigen::Vector4d iiwa_base_orientation_from_source;
Eigen::Vector3d iiwa_position_from_source;
Eigen::Vector3d iiwa_vel_from_source;
Eigen::Vector4d iiwa_orientation_from_source;*/

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

void objectCallback(const gazebo_msgs::ModelStates model_states){
  int box_index = getIndex(model_states.name, "my_box");

  box_pose = model_states.pose[box_index];
  box_vel = model_states.twist[box_index];
  object_pos << box_pose.position.x, box_pose.position.y, box_pose.position.z;
  object_vel << box_vel.linear.x, box_vel.linear.y, box_vel.linear.z;
}


void iiwaCallback(const gazebo_msgs::LinkStates link_states){
  int ee_index = getIndex(link_states.name, "iiwa1::iiwa1_link_7"); // End effector is the 7th link in KUKA IIWA
  int iiwa_base_index = getIndex(link_states.name, "iiwa1::iiwa1_link_0");

  ee_pose = link_states.pose[ee_index];
  ee_pos << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z;
  ee_vel = link_states.twist[ee_index];

  iiwa_base_pose = link_states.pose[iiwa_base_index];
}





int modeCheck(const int prev_mode){
  int mode = prev_mode;
  switch (prev_mode){
    case 1:   //track
      if (object_pos[1] < 0 && object_vel.norm() < 0.01) {mode = 2;}    //if object is on your side and not moving, go to hit
      break;
    case 2:   //hit
      if (object_vel.norm() > 0.01) {mode = 3;}                          //if object starts moving because it is hit, go to post hit
      break;
    case 3:   //post hit
      if (object_pos[1] > 0) {mode = 4;}                                //if object has left the range of arm, go to rest
      break;
    case 4:   //rest
      if (object_vel[1] < 0) {mode = 1;}                                //if object moves toward arm again, go to track
      break;
  }
  return mode;
}


geometry_msgs::Pose trackDS(){
  geometry_msgs::Pose vel_quat;

  return vel_quat;
}


geometry_msgs::Pose hitDS(Eigen::Vector3d ee_pos_init){
  geometry_msgs::Pose vel_quat;


  Eigen::Vector3d des_vel = {0.0, 0.0, 0.0};
  Eigen::Vector4d des_quat = Eigen::Vector4d::Zero();

  Eigen::Vector3d object_offset = {0.0, 0.0, 0.0};

  Eigen::Matrix3d rot_mat;
  rot_mat << cos(theta), -sin(theta), 0, 
              sin(theta), cos(theta), 0,
              0, 0, 1;

  Eigen::Vector3d ee_direction_x;
  Eigen::Vector3d ee_direction_y;
  Eigen::Vector3d ee_direction_z;
  Eigen::Matrix3d rot_ee;
  Eigen::Quaterniond rot_quat;

  Eigen::Vector3d unit_x = {0.0, 1.0, 0.0};

  Eigen::Vector3d attractor_main;
  Eigen::Vector3d attractor_aux;
  Eigen::Matrix3d gain_main;
  Eigen::Matrix3d gain_aux;

  gain_main << -0.9,  0.0, 0.0,
                0.0, -0.1, 0.0,
                0.0, 0.0, -0.9;

  gain_aux << -0.1,  0.0, 0.0,
               0.0, -0.1, 0.0,
               0.0, 0.0, -0.1;

  double modulated_sigma = 3.0;
  

  
  attractor_main = object_pos + 1*rot_mat*unit_x;
  attractor_aux = (4.0/3.0)*object_pos - (1.0/3.0)*attractor_main;

  ee_direction_z = attractor_main - object_pos;
  ee_direction_x = {0.0, 0.0, 1.0};
  ee_direction_y = ee_direction_z.cross(ee_direction_x);

  ee_direction_z = ee_direction_z/ee_direction_z.norm();
  ee_direction_y = ee_direction_y/ee_direction_y.norm();
  ee_direction_x = ee_direction_x/ee_direction_x.norm();

  rot_ee.col(0) = ee_direction_x;
  rot_ee.col(1) = ee_direction_y;
  rot_ee.col(2) = ee_direction_z;

  rot_quat =  rot_ee;
  des_quat << rot_quat.x(), rot_quat.y(), rot_quat.z(), rot_quat.w();

  
  double alpha = calculate_alpha(ee_pos, ee_pos_init, object_pos, attractor_main);

  des_vel = alpha*nominal_aux(rot_mat, gain_aux, ee_pos, attractor_aux) + (1 - alpha)*nominal_main(rot_mat, gain_main, ee_pos, attractor_main)
            + modulated_DS(attractor_main, object_pos, ee_pos, modulated_sigma);

  des_vel = des_vel*des_speed / des_vel.norm();

  vel_quat.position.x = des_vel(0);
  vel_quat.position.y = des_vel(1);
  vel_quat.position.z = des_vel(2);
  vel_quat.orientation.x = des_quat(0);
  vel_quat.orientation.y = des_quat(1);
  vel_quat.orientation.z = des_quat(2);
  vel_quat.orientation.w = des_quat(3);
  
  return vel_quat;
}


geometry_msgs::Pose postHit(const Eigen::Vector3d object_pos_init){
  geometry_msgs::Pose pos_quat;
  pos_quat.position.x = object_pos_init[0] - iiwa_base_pose.position.x;
  pos_quat.position.y = object_pos_init[1] - iiwa_base_pose.position.y;
  pos_quat.position.z = object_pos_init[2] - iiwa_base_pose.position.z;
  pos_quat.orientation.x = rest_quat[0];
  pos_quat.orientation.y = rest_quat[1];
  pos_quat.orientation.z = rest_quat[2];
  pos_quat.orientation.w = rest_quat[3];
  return pos_quat;
}


geometry_msgs::Pose rest(){
  geometry_msgs::Pose pos_quat;
  pos_quat.position.x = rest_pos[0] - iiwa_base_pose.position.x;
  pos_quat.position.y = rest_pos[1] - iiwa_base_pose.position.y;
  pos_quat.position.z = rest_pos[2] - iiwa_base_pose.position.z;
  pos_quat.orientation.x = rest_quat[0];
  pos_quat.orientation.y = rest_quat[1];
  pos_quat.orientation.z = rest_quat[2];
  pos_quat.orientation.w = rest_quat[3];
  return pos_quat;
}







int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "hitting_DS");
  ros::NodeHandle nh;
  ros::Rate rate(100);

  ros::Subscriber object_subs = nh.subscribe("/gazebo/model_states", 100 , objectCallback);
  ros::Subscriber iiwa_subs = nh.subscribe("/gazebo/link_states", 100, iiwaCallback);
  
  
  ros::Publisher pub_vel_quat; // publishes to robot command topic
  pub_vel_quat = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/vel_quat", 1);

  ros::Publisher pub_pos_quat; // publishes to robot command topic
  pub_pos_quat = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/pos_quat", 1);

  
  
  
  std::cout << "Enter the desired speed of hitting (between 0 and 1)" << std::endl;
  std::cin >> des_speed;

  while(des_speed < 0 || des_speed > 1){
    std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
    std::cin >> des_speed;
  }

  std::cout << "Enter the desired direction of hitting (between -pi/2 and pi/2)" << std::endl;
  std::cin >> theta;

  while(theta < -M_PI_2 || theta > M_PI_2){
    std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
    std::cin >> theta;
  }





  int mode = 1;
  int prev_mode = mode;

  Eigen::Vector3d object_pos_init = object_pos;
  Eigen::Vector3d ee_pos_init = ee_pos;



  std::ofstream object_data;
  bool store_data = 0;

  while(ros::ok()){
    //object_position_from_source = world_to_base * (object_position_mocap - iiwa_base_position_from_source);
    std::cout << "object is at: " << object_pos <<std::endl;



    mode = modeCheck(prev_mode);
    switch (mode) {
      case 1: //track
        pub_vel_quat.publish(trackDS());
        ee_pos_init = ee_pos;
        break;
      case 2: //hit
        pub_vel_quat.publish(hitDS(ee_pos_init));
        object_pos_init = object_pos;
        break;
      case 3: //post hit
        pub_pos_quat.publish(postHit(object_pos_init));
        break;
      case 4: //rest
        pub_pos_quat.publish(rest());
        break;
    }
    prev_mode = mode;
    
      

    object_data.open("/home/ros/ros_overlay_ws/src/i_am_project/data/object_data.csv", std::ofstream::out | std::ofstream::app);
    if(!object_data.is_open())
    {
      std::cerr << "Error opening output file.\n";
      std::cout << "Current path is " << std::experimental::filesystem::current_path() << '\n';
    }
    object_data << des_speed << ", " << theta << ", " << object_pos_init(0) << ", " << object_pos_init(1) << ", " << object_pos_init(2) << ", " << object_pos(0) << ", " << object_pos(1) << ", " << object_pos(2) << "\n";
    object_data.close();


    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
