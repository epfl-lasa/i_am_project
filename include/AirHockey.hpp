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

#include "ros/ros.h"
#include <ros/console.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>
#include <iostream>

#include "dynamical_system.h"
#include "keyboard_interaction.hpp"

#define NB_ROBOTS 2// Number of robots

class AirHockey {
private:
  enum Robot { IIWA_7 = 0, IIWA_14 = 1 };

  bool isHit_ = 0;

  Eigen::Vector3f hitDirection_[NB_ROBOTS];
  Eigen::Vector3f refVelocity_[NB_ROBOTS];
  Eigen::Vector4f refQuat_[NB_ROBOTS];
  Eigen::Vector3f returnPos_[NB_ROBOTS];

  std::string pubVelQuatTopic_[NB_ROBOTS];
  std::string iiwaInertiaTopic_[NB_ROBOTS];
  std::string iiwaPositionTopic_;
  std::string objectPositionTopic_;

  ros::Rate rate_;
  ros::NodeHandle nh_;
  ros::Publisher pubVelQuat_[NB_ROBOTS];
  ros::Subscriber objectPosition_;
  ros::Subscriber iiwaPosition_;
  ros::Subscriber iiwaInertia_[NB_ROBOTS];

  geometry_msgs::Pose boxPose_;
  geometry_msgs::Pose iiwaPose_[NB_ROBOTS];
  geometry_msgs::Twist iiwaVel_[NB_ROBOTS];

  Eigen::Vector3f objectPositionFromSource_;
  Eigen::Vector4d objectOrientationFromSource_;
  Eigen::Vector3f iiwaPositionFromSource_[NB_ROBOTS];
  Eigen::Matrix3f iiwaTaskInertiaPos_[NB_ROBOTS];


  std::unique_ptr<hitting_DS> generateHitting7_ =
      std::make_unique<hitting_DS>(iiwaPositionFromSource_[IIWA_7], objectPositionFromSource_);
  std::unique_ptr<hitting_DS> generateHitting14_ =
      std::make_unique<hitting_DS>(iiwaPositionFromSource_[IIWA_14], objectPositionFromSource_);

  //   std::unique_ptr<hitting_DS> generateHitting_[NB_ROBOTS];
  //   generateHitting_[IIWA_1] = std::make_unique<hitting_DS>(iiwaPositionFromSource_[IIWA_1], objectPositionFromSource_);
  //   generateHitting_[IIWA_2] = std::make_unique<hitting_DS>(iiwaPositionFromSource_[IIWA_2], objectPositionFromSource_);

public:
  enum robotState{REST, HIT};
  enum objectState{STOPPED_IN_1, MOVING_TO_2, STOPPED_IN_2, MOVING_TO_1};

  robotState state_robot7_ = REST;
  robotState state_robot14_ = REST;
  objectState state_object_ = STOPPED_IN_1;
  objectState prev_object_state_;

  struct StatesVar{
    robotState state_robot7_;
    robotState state_robot14_;
    objectState state_object_;
    objectState prev_object_state_;
    bool isHit_;
  };

  explicit AirHockey(ros::NodeHandle& nh, float frequency) : nh_(nh), rate_(frequency){};

  bool init();

  void run();
  void updateCurrentEEPosition(Eigen::Vector3f new_position[]);
  void publishVelQuat(Eigen::Vector3f DS_vel[], Eigen::Vector4f DS_quat[]);

  int getIndex(std::vector<std::string> v, std::string value);
  void updateCurrentObjectPosition(Eigen::Vector3f& new_position);
  void iiwaInertiaCallbackGazebo(const geometry_msgs::Inertia::ConstPtr& msg, int k);
  void iiwaPositionCallbackGazebo(const gazebo_msgs::LinkStates& linkStates);
  void objectPositionCallbackGazebo(const gazebo_msgs::ModelStates& modelStates);


  StatesVar getKeyboard(StatesVar statesvar ) {

    nonBlock(1);

    if (khBit() != 0) {
      char keyboardCommand = fgetc(stdin);
      fflush(stdin);

      switch (keyboardCommand) {
        case 'q': {
          statesvar.state_robot7_ = HIT;
          std::cout << "q is pressed " << std::endl;
          
        } break;
        case 'p': {
          statesvar.state_robot14_ = HIT;
        } break;
        case 'r': {
          statesvar.state_robot7_ = REST;
          statesvar.state_robot14_ = REST;
        } break;
        case 'h': { // toggle isHit_ 
          if(statesvar.isHit_){ statesvar.isHit_ = 0;}
          else if(!statesvar.isHit_){ statesvar.isHit_= 1;}
        } break;

      }
    }
    nonBlock(0);

    return statesvar;
  }
};