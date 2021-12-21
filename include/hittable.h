#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

bool hittable(Eigen::Vector3d object_pos, Eigen::Vector3d iiwa_base_pos, Eigen::Vector3d iiwa_flip, int iiwa_no);