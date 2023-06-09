#pragma once

#include "ros/ros.h"
#include <ros/console.h>

#include "tools/quattools.h"

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <iostream>

geometry_msgs::Pose
track(Eigen::Vector3d predict_pos, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector3d iiwa_base_pos);

geometry_msgs::Pose
rest(Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector3d iiwa_base_pos);

geometry_msgs::Pose postHit(Eigen::Vector3d object_pos_init,
                            Eigen::Vector3d center2,
                            Eigen::Vector3d iiwa_base_pos,
                            Eigen::Vector2d ee_offset);

geometry_msgs::Pose hitDSInertia(double des_speed,
                                 Eigen::Vector3d object_pos,
                                 Eigen::Vector3d center2,
                                 Eigen::Vector3d ee_pos,
                                 Eigen::Vector2d ee_offset,
                                 Eigen::Matrix3d& current_inertia);

geometry_msgs::Pose hitDS(double des_speed,
                          Eigen::Vector3d object_pos,
                          Eigen::Vector3d center2,
                          Eigen::Vector3d ee_pos,
                          Eigen::Vector3d ee_pos_init,
                          Eigen::Vector2d ee_offset);

// IIWA stops the object if goes too far
geometry_msgs::Pose block(Eigen::Vector3d object_pos,
                          Eigen::Vector3d predict_pos,
                          Eigen::Vector3d center1,
                          Eigen::Vector3d center2,
                          double object_th_mod,
                          Eigen::Vector3d iiwa_base_pos);

//   ------------------------------------------------ UTILS ------------------------------------------------
double calculate_alpha(Eigen::Vector3d& end_effector_position,
                       const Eigen::Vector3d& end_effector_init,
                       const Eigen::Vector3d& object_position,
                       Eigen::Vector3d& attractor_main);

Eigen::Vector3d modulated_DS(Eigen::Vector3d& attractor_main,
                             const Eigen::Vector3d& object_position_init,
                             Eigen::Vector3d& current_end_effector,
                             double sigma);

Eigen::Vector3d nominal_aux(Eigen::Matrix3d& rotation,
                            Eigen::Matrix3d& gain,
                            Eigen::Vector3d& current_end_effector,
                            Eigen::Vector3d& attractor_aux);

Eigen::Vector3d nominal_main(Eigen::Matrix3d& rotation,
                             Eigen::Matrix3d& gain,
                             Eigen::Vector3d& current_end_effector,
                             Eigen::Vector3d& attractor_main);
