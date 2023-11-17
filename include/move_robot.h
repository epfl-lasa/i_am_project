#pragma once

#include "ros/ros.h"
#include <ros/console.h>

#include "tools/quattools.h"

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <iostream>

geometry_msgs::Pose postHit(Eigen::Vector3d object_pos_init,
                            Eigen::Vector3d center,
                            Eigen::Vector3d iiwa_base_pos,
                            Eigen::Vector2d ee_offset);

geometry_msgs::Pose postHitDS(Eigen::Vector3d object_pos_init,
                            Eigen::Vector3d center,
                            Eigen::Vector3d iiwa_base_pos,
                            Eigen::Vector2d ee_offset,
                            Eigen::Vector3d return_position);

geometry_msgs::Pose hitDSInertia(double des_speed,
                                 Eigen::Vector3d object_pos,
                                 Eigen::Vector3d center,
                                 Eigen::Vector3d ee_pos,
                                 Eigen::Vector2d ee_offset,
                                 Eigen::Matrix3d& current_inertia);