//|    Copyright (C) 2022 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "hit_motion.h"

bool HitMotion::init() {
  // Get topics names
  nh_.getParam("/passive_control/vel_quat", pub_vel_quat_topic_);
  nh_.getParam("/iiwa/dir_flux", pub_dir_flux_topic);
  nh_.getParam("/vrpn_client_node/object_1/pose", object_position_topic_);
  nh_.getParam("/iiwa/ee_info/Pose", iiwa_position_topic_);
  nh_.getParam("/iiwa/ee_info/Vel", iiwa_vel_topic_);
  nh_.getParam("/iiwa/Inertia/taskPos", iiwa_inertia_topic_);
  nh_.getParam("/vrpn_client_node/iiwa_7_base/pose", iiwa_base_position_topic_);

  pub_vel_quat_ = nh_.advertise<geometry_msgs::Pose>(pub_vel_quat_topic_, 1);
  pub_dir_flux_ = nh_.advertise<std_msgs::Float32>(pub_dir_flux_topic, 1);

  object_position_ = nh_.subscribe(object_position_topic_,
                                   1,
                                   &HitMotion::objectPositionCallback,
                                   this,
                                   ros::TransportHints().reliable().tcpNoDelay());
  iiwa_position_ = nh_.subscribe(iiwa_position_topic_,
                                 1,
                                 &HitMotion::iiwaPositionCallback,
                                 this,
                                 ros::TransportHints().reliable().tcpNoDelay());
  iiwa_vel_ = nh_.subscribe(iiwa_vel_topic_,
                            1,
                            &HitMotion::iiwaVelCallback,
                            this,
                            ros::TransportHints().reliable().tcpNoDelay());
  iiwa_inertia_ = nh_.subscribe(iiwa_inertia_topic_,
                                1,
                                &HitMotion::iiwaInertiaCallback,
                                this,
                                ros::TransportHints().reliable().tcpNoDelay());
  iiwa_base_position_ = nh_.subscribe(iiwa_base_position_topic_,
                                      1,
                                      &HitMotion::iiwaBasePositionCallback,
                                      this,
                                      ros::TransportHints().reliable().tcpNoDelay());

  while (object_position_from_source_.norm() == 0) {
    updateCurrentObjectPosition(object_position_from_source_);
    ros::spinOnce();
    rate_.sleep();
  }

  objectPositionWorldFrame();
  std::cout << "object at: " << object_position_world_.transpose() << std::endl;

  Eigen::Vector3f object_offset = {0.0, -0.2, -0.00};//normal
  // Eigen::Vector3f object_offset = {0.05, -0.15, -0.05}; //small object

  _generate_hitting->set_current_position(iiwa_position_from_source_);
  _generate_hitting->set_DS_attractor(object_position_world_ + object_offset);

  nh_.getParam("ref_velocity/x", ref_velocity_[0]);
  nh_.getParam("ref_velocity/y", ref_velocity_[1]);
  nh_.getParam("ref_velocity/z", ref_velocity_[2]);
  nh_.getParam("ref_quat/w", ref_quat_[0]);
  nh_.getParam("ref_quat/x", ref_quat_[1]);
  nh_.getParam("ref_quat/y", ref_quat_[2]);
  nh_.getParam("ref_quat/z", ref_quat_[3]);
  nh_.getParam("hit_direction/x", hit_direction_[0]);
  nh_.getParam("hit_direction/y", hit_direction_[1]);
  nh_.getParam("hit_direction/z", hit_direction_[2]);

  _generate_hitting->set_des_direction(hit_direction_);

  return true;

  // TODO DELETE NEVER USED
  // pub_pos_quat_ = nh_.advertise<geometry_msgs::Pose>("/passive_control/pos_quat", 1);
}

void HitMotion::run() {
  Eigen::Vector3f iiwa_return_position = {0.5, -0.25, 0.3};
  Eigen::Vector3f final_position = {0.6, 0.1, 0.2};
  while (ros::ok()) {

    if (!is_hit_) {
      ref_velocity_ = _generate_hitting->flux_DS(0.5, iiwa_task_inertia_pos_);
      publishFlux(iiwa_task_inertia_pos_, iiwa_vel_from_source_);

      // ref_velocity_ = _generate_hitting->vel_max_DS();
      // objectPositionWorldFrame();

      // ROS_INFO_STREAM("object at: " << object_position_world_.transpose());

    } else {
      ref_velocity_ = _generate_hitting->linear_DS(iiwa_return_position);
    }

    if (!is_hit_
        && _generate_hitting->get_des_direction().dot(_generate_hitting->get_DS_attractor()
                                                      - _generate_hitting->get_current_position())
            < 0) {
      is_hit_ = 1;
    }
    // ref_velocity_ = _generate_hitting->linear_DS(final_position);
    // objectPositionWorldFrame();

    // ROS_INFO_STREAM("object at: " << object_position_world_.transpose());

    updateCurrentEEPosition(iiwa_position_from_source_);
    publishVelQuat(ref_velocity_, ref_quat_);
    ros::spinOnce();
    rate_.sleep();
  }

  publishVelQuat(ref_velocity_, ref_quat_);
  // publishFlux(iiwa_task_inertia_pos_, iiwa_vel_from_source_);
  ros::spinOnce();
  rate_.sleep();
  ros::shutdown();
}

void HitMotion::iiwaBasePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  iiwa_base_position_from_source_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  iiwa_base_orientation_from_source_ << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w;
}

void HitMotion::iiwaPositionCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  iiwa_position_from_source_ << msg->position.x, msg->position.y, msg->position.z;
  iiwa_orientation_from_source_ << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
}

void HitMotion::iiwaVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  iiwa_vel_from_source_ << msg->linear.x, msg->linear.y, msg->linear.z;
}

void HitMotion::objectPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  object_position_from_source_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  object_orientation_from_source_ << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w;
}

void HitMotion::objectPositionWorldFrame() {
  rotation_ << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  object_position_world_ = rotation_ * (object_position_from_source_ - iiwa_base_position_from_source_);
}

void HitMotion::iiwaInertiaCallback(const geometry_msgs::Inertia& inertia_msg) {
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

void HitMotion::updateCurrentEEPosition(Eigen::Vector3f& new_position) {
  _generate_hitting->set_current_position(new_position);
}

void HitMotion::updateCurrentObjectPosition(Eigen::Vector3f& new_position) {
  _generate_hitting->set_DS_attractor(new_position);
}

void HitMotion::publishVelQuat(Eigen::Vector3f& DS_vel, Eigen::Vector4f& DS_quat) {
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

void HitMotion::publishFlux(Eigen::Matrix3f& iiwa_task_inertia_pos, Eigen::Vector3f& iiwa_vel_from_source) {
  std_msgs::Float32 dir_flux_publish;
  float m_obj = 0.4;
  dir_flux_publish.data =
      (iiwa_task_inertia_pos(1, 1) / (iiwa_task_inertia_pos(1, 1) + m_obj)) * iiwa_vel_from_source(1);
  pub_dir_flux_.publish(dir_flux_publish);
}

int main(int argc, char** argv) {

  //ROS Initialization
  ros::init(argc, argv, "momentum_hit_real");
  float frequency = 200.0f;
  ros::NodeHandle nh;

  std::unique_ptr<HitMotion> generate_motion = std::make_unique<HitMotion>(nh, frequency);

  if (!generate_motion->init()) {
    return -1;
  } else {
    generate_motion->run();
  }

  return 0;
}

// TODO DELETE NEVER USED
// void HitMotion::publishPosQuat(const Eigen::Vector3f& DS_pos, const Eigen::Vector4f& DS_quat) {
//   geometry_msgs::Pose ref_pos_publish;
//   ref_pos_publish.position.x = DS_pos(0);
//   ref_pos_publish.position.y = DS_pos(1);
//   ref_pos_publish.position.z = DS_pos(2);
//   ref_pos_publish.orientation.x = DS_quat(0);
//   ref_pos_publish.orientation.y = DS_quat(1);
//   ref_pos_publish.orientation.z = DS_quat(2);
//   ref_pos_publish.orientation.w = DS_quat(3);

//   pub_pos_quat_.publish(ref_pos_publish);
// }

// void HitMotion::setGains(Eigen::Matrix3f& gain) { _generate_hitting->set_gain(gain); }
