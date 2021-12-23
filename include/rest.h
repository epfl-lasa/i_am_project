#pragma once

#include "../include/quattools.h"

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose rest(Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector3d iiwa_base_pos);