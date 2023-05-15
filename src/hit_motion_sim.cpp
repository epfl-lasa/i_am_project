//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "hit_motion_sim.h"

bool HitMotionSim::init() {
  // Get topics names
  if (!nh_.getParam("/passive_control/vel_quat", pub_vel_quat_topic_)) {ROS_ERROR("Topic /passive_control not found");}
  if (!nh_.getParam("/gazebo/model_states", object_position_topic_)) {ROS_ERROR("Topic /gazebo not found");}
  if (!nh_.getParam("/gazebo/link_states", iiwa_position_topic_)) {ROS_ERROR("Topic /gazebo not found");}
  if (!nh_.getParam("/iiwa/inertia/taskPos", iiwa_inertia_topic_)) {ROS_ERROR("Topic /iiwa/inertia/taskPos not found");}

  // Init publishers
  pub_vel_quat_ = nh_.advertise<geometry_msgs::Pose>(pub_vel_quat_topic_, 1);

  // Init subscribers
  object_position_ = nh_.subscribe(object_position_topic_,
                                   1,
                                   &HitMotionSim::objectPositionCallback_gazebo,
                                   this,
                                   ros::TransportHints().reliable().tcpNoDelay());
  iiwa_position_ = nh_.subscribe(iiwa_position_topic_,
                                 1,
                                 &HitMotionSim::iiwaPositionCallback_gazebo,
                                 this,
                                 ros::TransportHints().reliable().tcpNoDelay());
  iiwa_inertia_ = nh_.subscribe(iiwa_inertia_topic_,
                                1,
                                &HitMotionSim::iiwaInertiaCallback_gazebo,
                                this,
                                ros::TransportHints().reliable().tcpNoDelay());

  while (object_position_from_source_.norm() == 0) {
    this->updateCurrentObjectPosition(object_position_from_source_);
    ros::spinOnce();
    rate_.sleep();
  }

  generate_hitting_->set_current_position(iiwa_position_from_source_);
  generate_hitting_->set_DS_attractor(object_position_from_source_);

  if (!nh_.getParam("ref_velocity/x", ref_velocity_[0])) {ROS_ERROR("Topic ref_velocity/x not found");}
  if (!nh_.getParam("ref_velocity/y", ref_velocity_[1])) {ROS_ERROR("Topic ref_velocity/y not found");}
  if (!nh_.getParam("ref_velocity/z", ref_velocity_[2])) {ROS_ERROR("Topic ref_velocity/z not found");}
  if (!nh_.getParam("ref_quat/w", ref_quat_[0])) {ROS_ERROR("Topic ref_quat/w not found");}
  if (!nh_.getParam("ref_quat/x", ref_quat_[1])) {ROS_ERROR("Topic ref_quat/x not found");}
  if (!nh_.getParam("ref_quat/y", ref_quat_[2])) {ROS_ERROR("Topic ref_quat/y not found");}
  if (!nh_.getParam("ref_quat/z", ref_quat_[3])) {ROS_ERROR("Topic ref_quat/z not found");}
  if (!nh_.getParam("hit_direction/x", hit_direction_[0])) {ROS_ERROR("Topic hit_direction/x not found");}
  if (!nh_.getParam("hit_direction/y", hit_direction_[1])) {ROS_ERROR("Topic hit_direction/y not found");}
  if (!nh_.getParam("hit_direction/z", hit_direction_[2])) {ROS_ERROR("Topic hit_direction/z not found");}

  generate_hitting_->set_des_direction(hit_direction_);

  return true;
}

void HitMotionSim::run() {
  Eigen::Vector3f iiwa_return_position = {0.3, 0.0, 0.5};

  while (ros::ok()) {
    if (!is_hit_) {
      ref_velocity_ = generate_hitting_->flux_DS(0.5, iiwa_task_inertia_pos_);
    } else {
      ref_velocity_ = generate_hitting_->linear_DS(iiwa_return_position);
    }

    // To add the condition for the return of the iiwa after hitting
    ROS_INFO_STREAM("Dot product: " << generate_hitting_->get_des_direction().dot(
                        generate_hitting_->get_DS_attractor() - generate_hitting_->get_current_position()));

    if (!is_hit_
        && generate_hitting_->get_des_direction().dot(generate_hitting_->get_DS_attractor()
                                                      - generate_hitting_->get_current_position())
            < 0) {
      is_hit_ = 1;
    }

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

int HitMotionSim::getIndex(std::vector<std::string> v, std::string value) {
  for (int i = 0; i < v.size(); i++) {
    if (v[i].compare(value) == 0) return i;
  }
  return -1;
}

void HitMotionSim::iiwaInertiaCallback_gazebo(const geometry_msgs::Inertia& inertia_msg) {
  iiwa_task_inertia_pos_(0, 0) = inertia_msg.ixx;
  iiwa_task_inertia_pos_(2, 2) = inertia_msg.izz;
  iiwa_task_inertia_pos_(1, 1) = inertia_msg.iyy;
  iiwa_task_inertia_pos_(0, 1) = inertia_msg.ixy;
  iiwa_task_inertia_pos_(1, 0) = inertia_msg.ixy;
  iiwa_task_inertia_pos_(0, 2) = inertia_msg.ixz;
  iiwa_task_inertia_pos_(2, 0) = inertia_msg.ixz;
  iiwa_task_inertia_pos_(1, 2) = inertia_msg.iyz;
  iiwa_task_inertia_pos_(2, 1) = inertia_msg.iyz;
}

void HitMotionSim::iiwaPositionCallback_gazebo(const gazebo_msgs::LinkStates& link_states) {
  int iiwa_index = getIndex(link_states.name, "iiwa::iiwa_link_7");// End effector is the 7th link in KUKA IIWA

  iiwa_pose_ = link_states.pose[iiwa_index];
  iiwa_position_from_source_ << iiwa_pose_.position.x, iiwa_pose_.position.y, iiwa_pose_.position.z;
  iiwa_vel_ = link_states.twist[iiwa_index];
}

void HitMotionSim::objectPositionCallback_gazebo(const gazebo_msgs::ModelStates& model_states) {
  int box_index = getIndex(model_states.name, "box_model");
  box_pose_ = model_states.pose[box_index];
  object_position_from_source_ << box_pose_.position.x, box_pose_.position.y, box_pose_.position.z;
  object_orientation_from_source_ << box_pose_.orientation.x, box_pose_.orientation.y, box_pose_.orientation.z,
      box_pose_.orientation.w;
}

void HitMotionSim::publishVelQuat(Eigen::Vector3f& DS_vel, Eigen::Vector4f& DS_quat) {
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

void HitMotionSim::updateCurrentEEPosition(Eigen::Vector3f& new_position) {
  generate_hitting_->set_current_position(new_position);
}

void HitMotionSim::updateCurrentObjectPosition(Eigen::Vector3f& new_position) {
  generate_hitting_->set_DS_attractor(new_position);
}

int main(int argc, char** argv) {

  //ROS Initialization
  ros::init(argc, argv, "momentum_hit");
  ros::NodeHandle nh;
  float frequency = 200.0f;

  std::unique_ptr<HitMotionSim> generate_motion = std::make_unique<HitMotionSim>(nh, frequency);

  if (!generate_motion->init()) {
    return -1;
  } else {
    generate_motion->run();
  }

  return 0;
}
