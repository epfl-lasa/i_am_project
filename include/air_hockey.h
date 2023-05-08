#pragma once

#include "ros/package.h"
#include "ros/ros.h"
#include "thirdparty/Utils.h"
#include <ros/console.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <math.h>
#include <numeric>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>

#include "../include/block.h"
#include "../include/hit_DS.h"
#include "../include/post_hit.h"
#include "../include/rest.h"
#include "../include/track.h"

#include "../include/hittable.h"
#include "../include/modeselektor.h"
#include "../include/quattools.h"

#include <experimental/filesystem>
#define _USE_MATH_DEFINES

class AirHockey {
private:
  bool hollow_, iiwa_real_, manual_mode_, object_real_, hitta1_, hitta2_, farra1_, farra2_;

  int key_ctrl_ = 0;
  int mode1_ = 5;
  int mode2_ = 5;
  int prev_mode1_, prev_mode2_;

  double ETA_;
  double ee_offset_h_, ee_offset_v_, des_speed_, x_reach_, y_reach_, x_offset_, y_offset_, min_y_, max_y_, object_th_,
      object_th_mod_;

  std_msgs::Int16 msg_mode1_, msg_mode2_;

  Eigen::Vector2d ee_offset_;
  Eigen::Vector3d object_pos_, iiwa1_base_pos_, iiwa2_base_pos_, ee1_pos_, ee2_pos_, object_pos_init1_,
      object_pos_init2_, ee1_pos_init_, ee2_pos_init_, object_vel_, predict_pos_, center1_, center2_;
  Eigen::Vector4d hittable_params_;
  Eigen::Matrix3d R_Opti_, R_EE_;

  std::vector<double> center1vec_, center2vec_;

  ros::Rate rate_;
  ros::NodeHandle nh_;
  ros::Publisher pub_mode1_, pub_mode2_, pub_vel_quat1_, pub_pos_quat1_, pub_vel_quat2_, pub_pos_quat2_;
  ros::Subscriber estimate_object_subs_, iiwa_subs_, iiwa1_base_subs_, iiwa2_base_subs_, iiwa1_ee_subs_, iiwa2_ee_subs_,
      mode_sub_, object_subs_;
  ros::ServiceClient set_state_client_;

public:
  explicit AirHockey(ros::NodeHandle& nh, float frequency) : nh_(nh), rate_(frequency){};

  bool init();
  void run();

  void switch_both_mode();
  void move_robot(int mode, int mode_id);
  void reset_object_position();

  //Gazebo
  void objectSimCallback(const gazebo_msgs::ModelStates model_states);
  void iiwaSimCallback(const gazebo_msgs::LinkStates link_states);

  //Optitrack
  void objectCallback(const geometry_msgs::Pose object_pose);
  void iiwa1BaseCallback(const geometry_msgs::Pose base_pose);
  void iiwa2BaseCallback(const geometry_msgs::Pose base_pose);
  void iiwa1EEPoseCallback(const geometry_msgs::Pose ee_pose);
  void iiwa2EEPoseCallback(const geometry_msgs::Pose ee_pose);

  //i_am_predict or estimate_sim
  void estimateObjectCallback(std_msgs::Float64MultiArray estimation);

  //manual control
  void modeCallback(std_msgs::Int16 msg);
  int getIndex(std::vector<std::string> v, std::string value);
};
