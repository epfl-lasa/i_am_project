//|
//|    Copyright (C) 2021-2023 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors: Harshit Khurana (maintainer)
//|
//|    email:   harshit.khurana@epfl.ch
//|
//|    website: lasa.epfl.ch
//|
//|    This file is part of iam_dual_arm_control.
//|    This work was supported by the European Community's Horizon 2020 Research and Innovation
//|    programme (call: H2020-ICT-09-2019-2020, RIA), grant agreement 871899 Impact-Aware Manipulation.
//|
//|    iam_dual_arm_control is free software: you can redistribute it and/or modify  it under the terms
//|    of the GNU General Public License as published by  the Free Software Foundation,
//|    either version 3 of the License, or  (at your option) any later version.
//|
//|    iam_dual_arm_control is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sys/stat.h>
#include <string>
#include <iomanip>
#include <limits>

#include "dynamical_system.h"
#include "keyboard_interaction.hpp"

#define NB_ROBOTS 2// Number of robots

class AirHockey {
private:
  enum Robot { IIWA_7 = 0, IIWA_14 = 1 };
  enum robotState{REST, HIT};
  enum objectState{STOPPED_IN_1 = 0, MOVING_TO_2 = 1, STOPPED_IN_2 = 2, MOVING_TO_1 = 3};

  struct StatesVar{
    robotState state_robot7_ = REST;
    robotState state_robot14_ = REST;
    objectState state_object_ = STOPPED_IN_1;
    objectState prev_object_state_ = STOPPED_IN_1;
    bool isHit_ = 0;
  };

  struct RecordedRobotState {
    std::string robot_name;
    ros::Time time;
    Eigen::VectorXd joint_pos = Eigen::VectorXd(7);
    Eigen::VectorXd joint_vel = Eigen::VectorXd(7);
    Eigen::Vector3f eef_pos;
    Eigen::Vector4f eef_orientation;
    Eigen::Vector3f eef_vel;
    Eigen::Matrix<float, 9, 1> inertia;
    float hitting_flux;
  };

  struct RecordedObjectState {
    ros::Time time;
    Eigen::Vector3f position;
  };

  bool isHit_ = 0;
  bool isSim_;
  bool isRecording_;

  std::string recordingFolderPath_;
  float recordingTimeObject_;

  StatesVar statesvar;

  Eigen::Vector3f hitDirection_[NB_ROBOTS];
  Eigen::Vector3f refVelocity_[NB_ROBOTS];
  Eigen::Vector4f refQuat_[NB_ROBOTS];
  Eigen::Vector3f returnPos_[NB_ROBOTS];
  float hittingFlux_[NB_ROBOTS];
  float objectMass_;

  std::string pubVelQuatTopic_[NB_ROBOTS];
  std::string iiwaInertiaTopic_[NB_ROBOTS];
  std::string iiwaPositionTopicSim_;
  std::string objectPositionTopic_;
  std::string iiwaPositionTopicReal_[NB_ROBOTS];
  std::string iiwaVelocityTopicReal_[NB_ROBOTS];
  std::string iiwaBasePositionTopic_[NB_ROBOTS];
  std::string iiwaJointStateTopicReal_[NB_ROBOTS];

  ros::Rate rate_;
  ros::NodeHandle nh_;
  ros::Publisher pubVelQuat_[NB_ROBOTS];
  ros::Subscriber objectPosition_;
  ros::Subscriber iiwaPosition_;
  ros::Subscriber iiwaInertia_[NB_ROBOTS];
  ros::Subscriber iiwaPositionReal_[NB_ROBOTS];
  ros::Subscriber iiwaVelocityReal_[NB_ROBOTS];
  ros::Subscriber iiwaBasePosition_[NB_ROBOTS];
  ros::Subscriber iiwaJointStateReal_[NB_ROBOTS];

  geometry_msgs::Pose boxPose_;
  geometry_msgs::Pose iiwaPose_[NB_ROBOTS];
  geometry_msgs::Twist iiwaVel_[NB_ROBOTS];
  sensor_msgs::JointState iiwaJointState_[NB_ROBOTS];

  Eigen::Vector3f objectPositionFromSource_;
  Eigen::Vector3f previousObjectPositionFromSource_;
  Eigen::Vector4f objectOrientationFromSource_;
  Eigen::Vector3f objectPositionForIiwa_[NB_ROBOTS];
  Eigen::Matrix3f rotationMat_;
  Eigen::Vector3f iiwaPositionFromSource_[NB_ROBOTS];
  Eigen::Vector4f iiwaOrientationFromSource_[NB_ROBOTS];
  Eigen::Vector3f iiwaVelocityFromSource_[NB_ROBOTS];
  Eigen::Vector3f iiwaBasePositionFromSource_[NB_ROBOTS];
  Eigen::Matrix3f iiwaTaskInertiaPos_[NB_ROBOTS];
  Eigen::Vector3f objectOffset_[NB_ROBOTS];
  bool isObjectMoving_;
  int moved_manually_count_;


  std::unique_ptr<hitting_DS> generateHitting7_ =
      std::make_unique<hitting_DS>(iiwaPositionFromSource_[IIWA_7], objectPositionFromSource_);
  std::unique_ptr<hitting_DS> generateHitting14_ =
      std::make_unique<hitting_DS>(iiwaPositionFromSource_[IIWA_14], objectPositionFromSource_);


  std::vector<RecordedRobotState> robotStatesVector_[NB_ROBOTS];
  std::vector<RecordedObjectState> objectStatesVector_;


  //   std::unique_ptr<hitting_DS> generateHitting_[NB_ROBOTS];
  //   generateHitting_[IIWA_1] = std::make_unique<hitting_DS>(iiwaPositionFromSource_[IIWA_1], objectPositionFromSource_);
  //   generateHitting_[IIWA_2] = std::make_unique<hitting_DS>(iiwaPositionFromSource_[IIWA_2], objectPositionFromSource_);

public:

  explicit AirHockey(ros::NodeHandle& nh, float frequency) : nh_(nh), rate_(frequency){};

  bool init();

  void run();
  void updateCurrentEEPosition(Eigen::Vector3f new_position[]);
  void publishVelQuat(Eigen::Vector3f DS_vel[], Eigen::Vector4f DS_quat[]);

  int getIndex(std::vector<std::string> v, std::string value);
  void updateDSAttractor();
  void iiwaInertiaCallback(const geometry_msgs::Inertia::ConstPtr& msg, int k);
  void iiwaPositionCallbackGazebo(const gazebo_msgs::LinkStates& linkStates);
  void objectPositionCallbackGazebo(const gazebo_msgs::ModelStates& modelStates);
  void iiwaJointStateCallbackReal(const sensor_msgs::JointState::ConstPtr& msg, int k);

  void iiwaPoseCallbackReal(const geometry_msgs::Pose::ConstPtr& msg, int k);
  void iiwaVelocityCallbackReal(const geometry_msgs::Twist::ConstPtr& msg, int k);
  void iiwaBasePositionCallbackReal(const geometry_msgs::PoseStamped::ConstPtr& msg, int k);
  void objectPositionCallbackReal(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void objectPositionIiwaFrames();

  void recordRobot(Robot robot_name);
  void recordObject();
  void recordObjectMovedByHand(int hit_count); 
  void writeRobotStatesToFile(Robot robot_name, int hit_count);
  void writeObjectStatesToFile(int hit_count);
  void copyYamlFile(std::string inFilePath, std::string outFilePath);
  void setUpRecordingDir();
  std::string robotToString(Robot robot_name);
  float calculateDirFlux(Robot robot_name);


  StatesVar getKeyboard(StatesVar statesvar );
};