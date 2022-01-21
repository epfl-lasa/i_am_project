#pragma once

#include "ros/ros.h"
#include "ros/package.h"
#include <ros/console.h>
#include "thirdparty/Utils.h"

#include <cstdio>
#include <iostream>
#include <numeric>
#include <math.h>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "../include/track.h"
#include "../include/block.h"
#include "../include/hit_DS.h"
#include "../include/post_hit.h"
#include "../include/rest.h"

#include "../include/quattools.h"
#include "../include/hittable.h"
#include "../include/modeselektor.h"


#include <iiwa_tools/iiwa_tools.h>
#include <experimental/filesystem>
#define _USE_MATH_DEFINES
