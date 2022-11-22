#pragma once

#include "ros/ros.h"
#include <ros/console.h>

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"
#include <vector>
#include <string>
#include <fstream>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Inertia.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dynamical_system.h"