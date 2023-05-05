#pragma once


#include "ros/ros.h"
#include "ros/package.h"

#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/SpawnModel.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

//Container for all necessary properties of a model
struct modelProperties {
  const char* name;
  double size_x;
  double size_y;
  double size_z;
  double com_x;
  double com_y;
  double com_z;
  double mass;
  double mu;
  double mu2;
  double ixx;
  double iyy;
  double izz;
  geometry_msgs::Pose pose;
};