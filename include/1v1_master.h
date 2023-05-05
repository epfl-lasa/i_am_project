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
