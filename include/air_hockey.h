#pragma once

#include "move_robot.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "thirdparty/Utils.h"
#include "tools/hittable.h"
#include "tools/quattools.h"

#include "dynamic_reconfigure/server.h"
#include "i_am_project/workspace_paramsConfig.h"

#include <ros/console.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <numeric>
#include <queue>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <string>
#include <vector>

#include <experimental/filesystem>
#define _USE_MATH_DEFINES

class AirHockey {
private:
  bool hollow_, iiwa_real_, manual_mode_, object_real_, hitta1_, hitta2_, farra1_, farra2_, debug_, inertia_;

  int key_ctrl_ = 0;
  int mode1_ = 5;
  int mode2_ = 5;
  int prev_mode1_, prev_mode2_;

  double ETA_, ee_offset_h_, ee_offset_v_, des_speed_, x_reach_, y_reach_, x_offset_, y_offset_, min_y_, max_y_,
      object_th_, object_th_mod_, threshold_ee_ready_;

  std_msgs::Int16 msg_mode1_, msg_mode2_;

  Eigen::Vector2d ee_offset_;
  Eigen::Vector3d object_pos_, iiwa1_base_pos_, iiwa2_base_pos_, ee1_pos_, ee2_pos_, object_pos_init1_,
      object_pos_init2_, ee1_pos_init_, ee2_pos_init_, object_vel_, predict_pos_, center1_, center2_;
  Eigen::Matrix3d iiwa1_task_inertia_pos_, iiwa2_task_inertia_pos_, R_Opti_, R_EE_;
  Eigen::Vector4d hittable_params_;

  std::vector<double> center1vec_, center2vec_;

  ros::Rate rate_;
  ros::NodeHandle nh_;
  ros::Publisher pub_mode1_, pub_mode2_, pub_vel_quat1_, pub_pos_quat1_, pub_vel_quat2_, pub_pos_quat2_;
  ros::Subscriber estimate_object_subs_, iiwa_subs_, iiwa1_base_subs_, iiwa2_base_subs_, iiwa1_ee_subs_, iiwa2_ee_subs_,
      mode_sub_, object_subs_, iiwa1_inertia_, iiwa2_inertia_;
  ros::ServiceClient set_state_client_;

  dynamic_reconfigure::Server<i_am_project::workspace_paramsConfig> dynRecServer_;
  dynamic_reconfigure::Server<i_am_project::workspace_paramsConfig>::CallbackType dynRecCallback_;

public:
  explicit AirHockey(ros::NodeHandle& nh, float frequency) : nh_(nh), rate_(frequency){};

  bool init();
  void run();

  void get_private_param();
  void param_cfg_callback(i_am_project::workspace_paramsConfig& config, uint32_t level);

  void switch_both_mode();
  void move_robot(int mode, int mode_id);
  void reset_object_position();

  //Inertia
  void iiwa1InertiaCallback(const geometry_msgs::Inertia& inertia_msg);
  void iiwa2InertiaCallback(const geometry_msgs::Inertia& inertia_msg);

  //Gazebo
  void objectSimCallback(const gazebo_msgs::ModelStates model_states);
  void iiwaSimCallback(const gazebo_msgs::LinkStates link_states);

  //Optitrack
  void objectCallback(const geometry_msgs::PoseStamped object_pose);
  void iiwa1BaseCallback(const geometry_msgs::PoseStamped base_pose);
  void iiwa2BaseCallback(const geometry_msgs::PoseStamped base_pose);
  void iiwa1EEPoseCallback(const geometry_msgs::Pose ee_pose);
  void iiwa2EEPoseCallback(const geometry_msgs::Pose ee_pose);

  //i_am_predict or estimate_sim
  void estimateObjectCallback(std_msgs::Float64MultiArray estimation);

  //manual control
  void modeCallback(std_msgs::Int16 msg);
  int getIndex(std::vector<std::string> v, std::string value);

  //Select mode
  int modeSelektor(Eigen::Vector3d object_pos,
                   Eigen::Vector3d object_pos_init,
                   Eigen::Vector3d object_vel,
                   Eigen::Vector3d predict_pos,
                   double ETA,
                   Eigen::Vector3d ee_pos,
                   Eigen::Vector3d center1,
                   Eigen::Vector3d center2,
                   Eigen::Vector2d ee_offset,
                   Eigen::Vector4d hittable_params,
                   const int prev_mode);
  int maniModeSelektor(Eigen::Vector3d object_pos,
                       Eigen::Vector3d object_pos_init,
                       Eigen::Vector3d object_vel,
                       Eigen::Vector3d ee_pos,
                       Eigen::Vector3d center1,
                       Eigen::Vector3d center2,
                       Eigen::Vector2d ee_offset,
                       Eigen::Vector4d hittable_params,
                       const int prev_mode,
                       const int key_ctrl,
                       const int iiwa_no);
};
