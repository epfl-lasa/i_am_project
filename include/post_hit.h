#pragma once

#include "../include/quattools.h"

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose postHit(Eigen::Vector3d object_pos_init, Eigen::Vector3d center2, Eigen::Vector3d iiwa_base_pos);