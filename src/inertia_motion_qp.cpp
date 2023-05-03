//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "inertia_motion_qp.h"

bool InertiaMotionQP::init() {
  // Get topics names
  nh_.getParam("/passive_control/vel_quat", pub_vel_quat_topic_);
  nh_.getParam("/iiwa/ee_info/Pose", iiwa_position_topic_);
  nh_.getParam("/iiwa/Inertia/taskPos", iiwa_inertia_topic_);

  pub_vel_quat_ = nh_.advertise<geometry_msgs::Pose>(pub_vel_quat_topic_, 1);

  iiwa_position_ = nh_.subscribe(iiwa_position_topic_,
                                 1,
                                 &InertiaMotionQP::iiwaPositionCallback,
                                 this,
                                 ros::TransportHints().reliable().tcpNoDelay());
  iiwa_inertia_ = nh_.subscribe(iiwa_inertia_topic_,
                                1,
                                &InertiaMotionQP::iiwaInertiaCallback,
                                this,
                                ros::TransportHints().reliable().tcpNoDelay());
  generate_inertia_motion_->set_current_position(iiwa_position_from_source_);
  return true;

  // TODO NEVER USED DELETE
  // nh_.getParam("/iiwa/PositionController/command", pub_ref_position_topic_);
  // pub_ref_position_ = nh_.advertise<std_msgs::Float64MultiArray>(pub_ref_position_topic_, 1);
}

void InertiaMotionQP::run() {

  while (ros::ok()) {
    ref_velocity_ = generate_inertia_motion_->linear_DS();
    updateCurrentEEPosition(iiwa_position_from_source_);
    publishVelQuat(ref_velocity_, ref_quat_);
    ros::spinOnce();
    rate_.sleep();
  }

  publishVelQuat(ref_velocity_, ref_quat_);
  ros::spinOnce();
  rate_.sleep();
  ros::shutdown();
}

void InertiaMotionQP::iiwaInertiaCallback(const geometry_msgs::Inertia& inertia_msg) {
  iiwa_task_inertia_pos_(0, 0) = inertia_msg.ixx;
  iiwa_task_inertia_pos_(1, 1) = inertia_msg.iyy;
  iiwa_task_inertia_pos_(2, 2) = inertia_msg.izz;
  iiwa_task_inertia_pos_(0, 1) = inertia_msg.ixy;
  iiwa_task_inertia_pos_(1, 0) = inertia_msg.ixy;
  iiwa_task_inertia_pos_(0, 2) = inertia_msg.ixz;
  iiwa_task_inertia_pos_(2, 0) = inertia_msg.ixz;
  iiwa_task_inertia_pos_(1, 2) = inertia_msg.iyz;
  iiwa_task_inertia_pos_(2, 1) = inertia_msg.iyz;
}

void InertiaMotionQP::iiwaPositionCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  iiwa_position_from_source_ << msg->position.x, msg->position.y, msg->position.z;
  iiwa_orientation_from_source_ << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
}

void InertiaMotionQP::publishVelQuat(Eigen::Vector3f& DS_vel, Eigen::Vector4f& DS_quat) {
  geometry_msgs::Pose ref_vel_publish;
  ref_vel_publish.position.x = DS_vel(0);
  ref_vel_publish.position.y = DS_vel(1);
  ref_vel_publish.position.z = DS_vel(2);
  ref_vel_publish.orientation.x = DS_quat(0);
  ref_vel_publish.orientation.y = DS_quat(1);
  ref_vel_publish.orientation.z = DS_quat(2);
  ref_vel_publish.orientation.w = DS_quat(3);

  pub_vel_quat_.publish(ref_vel_publish);
}

void InertiaMotionQP::setGains(Eigen::Matrix3f& gain) { generate_inertia_motion_->set_gain(gain); }

void InertiaMotionQP::updateCurrentEEPosition(Eigen::Vector3f& new_position) {
  generate_inertia_motion_->set_current_position(new_position);
}

int main(int argc, char** argv) {

  //ROS Initialization
  ros::init(argc, argv, "inertia_QP");
  float frequency = 200.0f;
  ros::NodeHandle nh;

  std::unique_ptr<InertiaMotionQP> generate_motion = std::make_unique<InertiaMotionQP>(nh, frequency);

  if (!generate_motion->init()) {
    return -1;
  } else {
    generate_motion->run();
  }

  return 0;
}

// TODO NEVER USED DELETE
// void InertiaMotionQP::publishJointPosition(const Eigen::VectorXf& joint_pos) {
//   std_msgs::Float64MultiArray ref_joint_position;

//   for (int i = 0; i < joint_pos.size(); ++i) { ref_joint_position.data.push_back(joint_pos[i]); }

//   pub_ref_position_.publish(ref_joint_position);
// }