#pragma once

#include "../include/quattools.h"

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose track(Eigen::Vector3d predict_pos, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector3d iiwa_base_pos);