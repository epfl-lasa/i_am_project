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
  bool hollow_;
  bool iiwa_real_;
  bool manual_mode_;
  bool object_real_;

  int mode1_ = 5;
  int mode2_ = 5;
  int prev_mode1_;
  int prev_mode2_;
  int key_ctrl_ = 0;

  double ee_offset_h_;
  double ee_offset_v_;
  double x_reach_;
  double y_reach_;
  double x_offset_;
  double y_offset_;
  double des_speed_;
  double theta_;

  double min_y_, max_y_;

  bool hitta1_, hitta2_, farra1_, farra2_;

  std_msgs::Int16 msg_mode1_;
  std_msgs::Int16 msg_mode2_;

  Eigen::Vector3d object_pos_;
  Eigen::Vector3d iiwa1_base_pos_;
  Eigen::Vector3d iiwa2_base_pos_;
  Eigen::Vector3d ee1_pos_;
  Eigen::Vector3d ee2_pos_;
  Eigen::Vector3d object_pos_init1_;
  Eigen::Vector3d object_pos_init2_;
  Eigen::Vector3d ee1_pos_init_;
  Eigen::Vector3d ee2_pos_init_;

  // Stuff to estimate
  Eigen::Vector3d object_vel_;
  Eigen::Vector3d predict_pos_;
  double ETA_;
  double stdev_;

  // Stuff to measure
  geometry_msgs::Pose object_pose_, iiwa1_base_pose_, iiwa2_base_pose_, ee1_pose_, ee2_pose_;
  geometry_msgs::Twist object_twist_;

  Eigen::Vector3d object_rpy_;
  double object_th_, object_th_mod_;

  std::vector<double> center1vec_;
  std::vector<double> center2vec_;
  Eigen::Vector3d center1_;
  Eigen::Vector3d center2_;
  Eigen::Vector2d ee_offset_;
  Eigen::Vector4d hittable_params_;
  Eigen::Matrix3d R_Opti;
  Eigen::Matrix3d R_EE;

  ros::Rate rate_;
  ros::NodeHandle nh_;
  ros::Publisher pub_mode1_;
  ros::Publisher pub_mode2_;
  ros::Publisher pub_vel_quat1_;
  ros::Publisher pub_pos_quat1_;
  ros::Publisher pub_vel_quat2_;
  ros::Publisher pub_pos_quat2_;
  ros::Subscriber estimate_object_subs_;
  ros::Subscriber iiwa_subs_;
  ros::Subscriber iiwa1_base_subs_;
  ros::Subscriber iiwa2_base_subs_;
  ros::Subscriber iiwa1_ee_subs_;
  ros::Subscriber iiwa2_ee_subs_;
  ros::Subscriber mode_sub_;
  ros::Subscriber object_subs_;
  ros::ServiceClient set_state_client_;

public:
  explicit AirHockey(ros::NodeHandle& nh, float frequency) : nh_(nh), rate_(frequency){};

  bool init();
  void run();

  void switch_both_mode();
  void move_robot(int mode, int mode_id);
  void reset_object_position();

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
