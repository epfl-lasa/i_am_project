//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    website: lasa.epfl.ch

#include "collect_data.h"

int CollectData::getIndex(std::vector<std::string> v, std::string value) {
  for (int i = 0; i < v.size(); i++) {
    if (v[i].compare(value) == 0) return i;
  }
  return -1;
}

//Gazebo callbacks
void CollectData::objectSimCallback(const gazebo_msgs::ModelStates model_states) {
  int box_index = getIndex(model_states.name, "my_box");

  object_pose_ = model_states.pose[box_index];
  object_twist_ = model_states.twist[box_index];
  object_pos_ << object_pose_.position.x, object_pose_.position.y, object_pose_.position.z;
  object_vel_ << object_twist_.linear.x, object_twist_.linear.y, object_twist_.linear.z;

  object_rpy_ = quatToRPY({object_pose_.orientation.w,
                           object_pose_.orientation.x,
                           object_pose_.orientation.y,
                           object_pose_.orientation.z});//get orientation in rpy
  object_th_ = object_rpy_[2];
  object_th_dot_ = object_twist_.angular.z;//only the z-axis
}

void CollectData::iiwaSimCallback(const gazebo_msgs::LinkStates link_states) {
  int iiwa1_ee_index = getIndex(link_states.name, "iiwa1::iiwa1_link_7");
  int iiwa2_ee_index = getIndex(link_states.name, "iiwa2::iiwa2_link_7");

  ee1_pose_ = link_states.pose[iiwa1_ee_index];
  ee1_rpy_ = quatToRPY({ee1_pose_.orientation.w,
                        ee1_pose_.orientation.x,
                        ee1_pose_.orientation.y,
                        ee1_pose_.orientation.z});//get orientation in rpy
  ee2_pose_ = link_states.pose[iiwa2_ee_index];
  ee2_rpy_ = quatToRPY({ee2_pose_.orientation.w,
                        ee2_pose_.orientation.x,
                        ee2_pose_.orientation.y,
                        ee2_pose_.orientation.z});//get orientation in rpy
}

//Optitrack callbacks
void CollectData::objectCallback(const geometry_msgs::Pose object_pose) {
  object_pos_ << object_pose.position.x, object_pose.position.y, object_pose.position.z;
  object_pos_ = R_Opti_ * object_pos_;

  object_rpy_ = quatToRPY({object_pose.orientation.w,
                           object_pose.orientation.x,
                           object_pose.orientation.y,
                           object_pose.orientation.z});//get orientation in rpy
  object_th_ = object_rpy_[2];                         //only the z-axis
}

void CollectData::iiwa1EEPoseCallback(const geometry_msgs::Pose ee_pose) {
  ee1_pose_.position.x = ee_pose.position.x;
  ee1_pose_.position.y = ee_pose.position.y;
  ee1_pose_.position.z = ee_pose.position.z;
  ee1_pose_.orientation.w = ee_pose.orientation.w;
  ee1_pose_.orientation.x = ee_pose.orientation.x;
  ee1_pose_.orientation.y = ee_pose.orientation.y;
  ee1_pose_.orientation.z = ee_pose.orientation.z;
  ee1_rpy_ = quatToRPY({ee1_pose_.orientation.w,
                        ee1_pose_.orientation.x,
                        ee1_pose_.orientation.y,
                        ee1_pose_.orientation.z});//get orientation in rpy
}

void CollectData::iiwa2EEPoseCallback(const geometry_msgs::Pose ee_pose) {
  ee2_pose_.position.x = ee_pose.position.x;
  ee2_pose_.position.y = ee_pose.position.y;
  ee2_pose_.position.z = ee_pose.position.z;
  ee2_pose_.orientation.w = ee_pose.orientation.w;
  ee2_pose_.orientation.x = ee_pose.orientation.x;
  ee2_pose_.orientation.y = ee_pose.orientation.y;
  ee2_pose_.orientation.z = ee_pose.orientation.z;
  ee2_rpy_ = quatToRPY({ee2_pose_.orientation.w,
                        ee2_pose_.orientation.x,
                        ee2_pose_.orientation.y,
                        ee2_pose_.orientation.z});//get orientation in rpy
}

//Passive_track (iiwa_toolkit) callbacks
void CollectData::iiwa1EETwistCallback(const geometry_msgs::Twist ee_twist) {
  ee1_twist_.linear.x = ee_twist.linear.x;
  ee1_twist_.linear.y = ee_twist.linear.y;
  ee1_twist_.linear.z = ee_twist.linear.z;
  ee1_twist_.angular.x = ee_twist.angular.x;
  ee1_twist_.angular.y = ee_twist.angular.y;
  ee1_twist_.angular.z = ee_twist.angular.z;
}

void CollectData::iiwa2EETwistCallback(const geometry_msgs::Twist ee_twist) {
  ee2_twist_.linear.x = ee_twist.linear.x;
  ee2_twist_.linear.y = ee_twist.linear.y;
  ee2_twist_.linear.z = ee_twist.linear.z;
  ee2_twist_.angular.x = ee_twist.angular.x;
  ee2_twist_.angular.y = ee_twist.angular.y;
  ee2_twist_.angular.z = ee_twist.angular.z;
}

void CollectData::iiwa1JointCallback(sensor_msgs::JointState joint_states) {
  iiwa1_joint_angles_ = joint_states.position;
}

void CollectData::iiwa2JointCallback(sensor_msgs::JointState joint_states) {
  iiwa2_joint_angles_ = joint_states.position;
}

//i_am_predict callbacks
void CollectData::estimateObjectCallback(std_msgs::Float64MultiArray estimation) {
  if (object_real_) { object_vel_ << estimation.data[5], estimation.data[6], estimation.data[7]; }
}

void CollectData::modeIwaa1Callback(std_msgs::Int16 msg) { mode1_ = msg.data; }

void CollectData::modeIwaa2Callback(std_msgs::Int16 msg) { mode2_ = msg.data; }

void CollectData::init() {

  //Get environment setting from param
  if (!nh_.getParam("object_real", object_real_)) { ROS_ERROR("Param object_real not found"); }
  if (!nh_.getParam("iiwa_real", iiwa_real_)) { ROS_ERROR("Param iiwa_real not found"); }
  if (!nh_.getParam("manual_mode", manual_mode_)) { ROS_ERROR("Param manual_mode not found"); }
  if (!nh_.getParam("hollow", hollow_)) { ROS_ERROR("Param hollow not found"); }

  // Get ros topics name
  std::string track_object, track_left_eepose, track_right_eepose, gazebo_model_states, gazebo_set_model_states,
      gazebo_link_states, joint_states_iiwa1, joint_states_iiwa2, ee_twist_iiwa1, ee_twist_iiwa2, estimate_object,
      mode_iiwa1, mode_iiwa2;
  if (!nh_.getParam("/simo_track/object_pose", track_object)) {
    ROS_ERROR("Param ros topic /simo_track/object_pose not found");
  }
  if (!nh_.getParam("/simo_track/robot_left/ee_pose", track_left_eepose)) {
    ROS_ERROR("Param ros topic /simo_track/robot_left/ee_pose not found");
  }
  if (!nh_.getParam("/simo_track/robot_right/ee_pose", track_right_eepose)) {
    ROS_ERROR("Param ros topic /simo_track/robot_right/ee_pose not found");
  }
  if (!nh_.getParam("/gazebo/model_states", gazebo_model_states)) {
    ROS_ERROR("Param ros topic /gazebo/model_states not found");
  }
  if (!nh_.getParam("/gazebo/set_model_state", gazebo_set_model_states)) {
    ROS_ERROR("Param ros topic /gazebo/set_model_state not found");
  }
  if (!nh_.getParam("/gazebo/link_states", gazebo_link_states)) {
    ROS_ERROR("Param ros topic /gazebo/link_states not found");
  }
  if (!nh_.getParam("/joint_states/iiwa1", joint_states_iiwa1)) {
    ROS_ERROR("Param ros topic /joint_states/iiwa1 not found");
  }
  if (!nh_.getParam("/joint_states/iiwa2", joint_states_iiwa2)) {
    ROS_ERROR("Param ros topic /joint_states/iiwa2 not found");
  }
  if (!nh_.getParam("/ee_twist/iiwa1", ee_twist_iiwa1)) { ROS_ERROR("Param ros topic /ee_twist/iiwa1 not found"); }
  if (!nh_.getParam("/ee_twist/iiwa2", ee_twist_iiwa2)) { ROS_ERROR("Param ros topic /ee_twist/iiwa2 not found"); }
  if (!nh_.getParam("estimate/object", estimate_object)) { ROS_ERROR("Param ros topic estimate/object not found"); }
  if (!nh_.getParam("/mode/iiwa1", mode_iiwa1)) { ROS_ERROR("Param ros topic mode/iiwa1 not found"); }
  if (!nh_.getParam("/mode/iiwa2", mode_iiwa2)) { ROS_ERROR("Param ros topic mode/iiwa2 not found"); }

  //Subscribers to object and IIWA states
  if (object_real_) {
    object_subs_ = nh_.subscribe(track_object, 10, &CollectData::objectCallback, this);
  } else {
    object_subs_ = nh_.subscribe(gazebo_model_states, 10, &CollectData::objectSimCallback, this);
    //Client to reset object pose
    ros::service::waitForService(gazebo_set_model_states);
    set_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(gazebo_set_model_states);
  }

  if (iiwa_real_) {
    iiwa1_ee_subs_ = nh_.subscribe(track_left_eepose, 10, &CollectData::iiwa1EEPoseCallback, this);
    iiwa2_ee_subs_ = nh_.subscribe(track_right_eepose, 10, &CollectData::iiwa2EEPoseCallback, this);
  } else {
    iiwa_subs_ = nh_.subscribe(gazebo_link_states, 10, &CollectData::iiwaSimCallback, this);
  }

  // from passive_control and iiwa_ros
  iiwa1_ee_twist_subs_ = nh_.subscribe(ee_twist_iiwa1, 100, &CollectData::iiwa1EETwistCallback, this);
  iiwa2_ee_twist_subs_ = nh_.subscribe(ee_twist_iiwa2, 100, &CollectData::iiwa2EETwistCallback, this);
  iiwa1_joint_subs_ = nh_.subscribe(joint_states_iiwa1, 100, &CollectData::iiwa1JointCallback, this);
  iiwa2_joint_subs_ = nh_.subscribe(joint_states_iiwa2, 100, &CollectData::iiwa2JointCallback, this);

  //Subcriber to position estimator
  estimate_object_subs_ = nh_.subscribe(estimate_object, 10, &CollectData::estimateObjectCallback, this);

  //Subscriber to Robot modes from master node
  mode1_sub_ = nh_.subscribe(mode_iiwa1, 10, &CollectData::modeIwaa1Callback, this);
  mode2_sub_ = nh_.subscribe(mode_iiwa2, 10, &CollectData::modeIwaa2Callback, this);

  //Center points of desired workspaces (used to aim for and tell what orientation)
  std::vector<double> center1vec;
  std::vector<double> center2vec;
  if (!nh_.getParam("center1", center1vec)) { ROS_ERROR("Param center1 not found"); }
  if (!nh_.getParam("center2", center2vec)) { ROS_ERROR("Param center2 not found"); }
  center1_ << center1vec[0], center1vec[1], center1vec[2];
  center2_ << center2vec[0], center2vec[1], center2vec[2];

  double x_reach, y_reach, x_offset, y_offset;
  if (!nh_.getParam("hittable/reach/x", x_reach)) { ROS_ERROR("Param hittable/reach/x not found"); }
  if (!nh_.getParam("hittable/reach/y", y_reach)) { ROS_ERROR("Param hittable/reach/y not found"); }
  if (!nh_.getParam("hittable/offset/x", x_offset)) { ROS_ERROR("Param hittable/offset/x not found"); }
  if (!nh_.getParam("hittable/offset/y", y_offset)) { ROS_ERROR("Param hittable/offset/y not found"); }
  hittable_params_ = {x_reach, y_reach, x_offset, y_offset};

  nh_.getParam("box/properties/size/x", box_.size_x);
  nh_.getParam("box/properties/size/y", box_.size_y);
  nh_.getParam("box/properties/size/z", box_.size_z);
  nh_.getParam("box/properties/COM/x", box_.com_x);
  nh_.getParam("box/properties/COM/y", box_.com_y);
  nh_.getParam("box/properties/COM/z", box_.com_z);
  nh_.getParam("box/properties/mass", box_.mass);
  nh_.getParam("box/properties/mu", box_.mu);
  nh_.getParam("box/properties/mu2", box_.mu2);
  box_.izz = 1.0 / 12 * box_.mass * (pow(box_.size_x, 2) + pow(box_.size_y, 2));

  R_Opti_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
}

void CollectData::run() {

  //Storing data to get the right values in time. The data we want is actually from right before events
  // that we can recognize (e.g. speed right before hitting)
  std::queue<ros::Time> time_q, pred_time_q;
  std::queue<double> object_theta_q, object_theta_dot_q;
  std::queue<std::vector<double>> iiwa1_joint_angles_q, iiwa2_joint_angles_q;
  std::queue<Eigen::Vector3d> object_pos_q, object_vel_q, predict_pos_q, ee1_rpy_q, ee2_rpy_q;
  std::queue<geometry_msgs::Pose> ee1_pose_q, ee2_pose_q;
  std::queue<geometry_msgs::Twist> ee1_twist_q, ee2_twist_q;

  int oneinten;
  bool once11 = true;
  bool once12 = true;
  bool once21 = false;
  bool once22 = false;
  bool tracking = false;
  bool hitta1, hitta2, farra1, farra2;
  std::stringstream data_path;
  data_path << ros::package::getPath("i_am_project") << "/data/collect/object_data.csv";
  std::ofstream object_data;
  ros::Time current_time;

  while (ros::ok()) {

    object_data.open(data_path.str(), std::ofstream::out | std::ofstream::app);
    if (!object_data.is_open()) {
      std::cerr << "Error opening output file.\n";
      std::cout << "Current path is " << std::experimental::filesystem::current_path() << '\n';
    }

    std::tie(hitta1, farra1) = hittable(object_pos_, center1_, center2_, hittable_params_);
    std::tie(hitta2, farra2) = hittable(object_pos_, center2_, center1_, hittable_params_);

    // Reset object position once out of reach if it is in gazebo (otherwise, there is little our code can do for us)
    if (object_real_ == false && object_vel_.norm() < 0.01 && (!hitta1 && !hitta2)) {

      tracking = false;
      if (once11 == false) {
        once12 = false;
        tracking = false;
        object_data << "end, reset"
                    << "\n\n\n";
        once21 = true;
      }
      if (once21 == false) {
        once22 = false;
        tracking = false;
        object_data << "end, reset"
                    << "\n\n\n";
        once11 = true;
      }

      ros::Duration(0.5).sleep();
    }

    current_time = ros::Time::now();

    //Store the actual data for 3 time steps. This turns out to be the right time. In other words, post-hit
    // is initiated 3 time steps after the step right before impact, which is the step we want data from.
    time_q.push(current_time);
    pred_time_q.push(current_time);
    object_pos_q.push(object_pos_);
    object_vel_q.push(object_vel_);
    object_theta_q.push(object_th_);
    object_theta_dot_q.push(object_th_dot_);
    ee1_pose_q.push(ee1_pose_);
    ee2_pose_q.push(ee2_pose_);
    ee1_rpy_q.push(ee1_rpy_);
    ee2_rpy_q.push(ee2_rpy_);
    ee1_twist_q.push(ee1_twist_);
    ee2_twist_q.push(ee2_twist_);
    iiwa1_joint_angles_q.push(iiwa1_joint_angles_);
    iiwa2_joint_angles_q.push(iiwa2_joint_angles_);

    while (time_q.size() > 4) { time_q.pop(); }
    while (object_pos_q.size() > 4) { object_pos_q.pop(); }
    while (object_vel_q.size() > 4) { object_vel_q.pop(); }
    while (pred_time_q.size() > 104) { pred_time_q.pop(); }
    while (predict_pos_q.size() > 104) { predict_pos_q.pop(); }
    while (object_theta_q.size() > 4) { object_theta_q.pop(); }
    while (object_theta_dot_q.size() > 4) { object_theta_dot_q.pop(); }
    while (ee1_pose_q.size() > 4) { ee1_pose_q.pop(); }
    while (ee2_pose_q.size() > 4) { ee2_pose_q.pop(); }
    while (ee1_rpy_q.size() > 4) { ee1_rpy_q.pop(); }
    while (ee2_rpy_q.size() > 4) { ee2_rpy_q.pop(); }
    while (ee1_twist_q.size() > 4) { ee1_twist_q.pop(); }
    while (ee2_twist_q.size() > 4) { ee2_twist_q.pop(); }
    while (iiwa1_joint_angles_q.size() > 4) { iiwa1_joint_angles_q.pop(); }
    while (iiwa2_joint_angles_q.size() > 4) { iiwa2_joint_angles_q.pop(); }

    if (mode1_ == 4
        && once11 == true) {//store data right before hit (this is once, directly after hit ; mode 4 --> post hit)
      once11 = false;
      once12 = true;
      object_data << "pre_post_time,    " << time_q.front() << ", " << current_time.toSec() << "\n";
      object_data << "box_properties,   " << box_.size_x << ", " << box_.size_y << ", " << box_.size_z << ", "
                  << box_.com_x << ", " << box_.com_y << ", " << box_.com_z << ", " << box_.mass << ", " << box_.mu
                  << ", " << box_.mu2 << ", " << box_.izz << "\n";
      object_data << "pre_hit_joints,   " << iiwa1_joint_angles_q.front()[0] << ", " << iiwa1_joint_angles_q.front()[1]
                  << ", " << iiwa1_joint_angles_q.front()[2] << ", " << iiwa1_joint_angles_q.front()[3] << ", "
                  << iiwa1_joint_angles_q.front()[4] << ", " << iiwa1_joint_angles_q.front()[5] << ", "
                  << iiwa1_joint_angles_q.front()[6] << "\n";
      object_data << "post_hit_joints,  " << iiwa1_joint_angles_[0] << ", " << iiwa1_joint_angles_[1] << ", "
                  << iiwa1_joint_angles_[2] << ", " << iiwa1_joint_angles_[3] << ", " << iiwa1_joint_angles_[4] << ", "
                  << iiwa1_joint_angles_[5] << ", " << iiwa1_joint_angles_[6] << "\n";
      object_data << "pre_hit_ee,       " << ee1_pose_q.front().position.x << ", " << ee1_pose_q.front().position.y
                  << ", " << ee1_pose_q.front().position.z << ", " << ee1_pose_q.front().orientation.x << ", "
                  << ee1_pose_q.front().orientation.y << ", " << ee1_pose_q.front().orientation.z << ", "
                  << ee1_pose_q.front().orientation.w << ", " << ee1_rpy_q.front()[0] << ", " << ee1_rpy_q.front()[1]
                  << ", " << ee1_rpy_q.front()[2] << ", " << ee1_twist_q.front().linear.x << ", "
                  << ee1_twist_q.front().linear.y << ", " << ee1_twist_q.front().linear.z << ", "
                  << ee1_twist_q.front().angular.x << ", " << ee1_twist_q.front().angular.y << ", "
                  << ee1_twist_q.front().angular.z << "\n";
      object_data << "post_hit_ee,      " << ee1_pose_.position.x << ", " << ee1_pose_.position.y << ", "
                  << ee1_pose_.position.z << ", " << ee1_pose_.orientation.x << ", " << ee1_pose_.orientation.y << ", "
                  << ee1_pose_.orientation.z << ", " << ee1_pose_.orientation.w << ", " << ee1_rpy_[0] << ", "
                  << ee1_rpy_[1] << ", " << ee1_rpy_[2] << ", " << ee1_twist_.linear.x << ", " << ee1_twist_.linear.y
                  << ", " << ee1_twist_.linear.z << ", " << ee1_twist_.angular.x << ", " << ee1_twist_.angular.y << ", "
                  << ee1_twist_.angular.z << "\n";
      object_data << "pred_stop_pos,    " << pred_time_q.front() << ", " << predict_pos_q.front()[0] << ", "
                  << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n";
      object_data << "pre_hit_object,   " << time_q.front() << ", " << object_pos_q.front()[0] << ", "
                  << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << ", " << object_theta_q.front()
                  << ", " << object_vel_q.front()[0] << ", " << object_vel_q.front()[1] << ", "
                  << object_vel_q.front()[2] << ", " << object_theta_dot_q.front() << "\n";
      object_data << "post_hit_object,  " << current_time.toSec() << ", " << object_pos_[0] << ", " << object_pos_[1]
                  << ", " << object_pos_[2] << ", " << object_th_ << ", " << object_vel_[0] << ", " << object_vel_[1]
                  << ", " << object_vel_[2] << ", " << object_th_dot_ << "\n";
      object_data << "post_hit_trajectory:\n";
      tracking = true;
      oneinten = 10;
    }

    if (mode2_ == 2 && object_pos_[1] > 0.1 && once12 == true) {// it will be stopped before reaching the end position
      once12 = false;
      tracking = false;
      object_data << "end, stopped"
                  << "\n\n\n";
      //object_data << "pred_stop_pos,   " << pred_time_q.front()<< ", " << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n";
      once21 = true;
    }
    if (mode2_ == 3 && once12 == true) {//or if it will be hit back just before it came to a halt
      once12 = false;
      tracking = false;
      object_data << current_time.toSec() << ", " << object_pos_[0] << ", " << object_pos_[1] << ", " << object_pos_[2]
                  << ", " << object_th_ << ", " << object_vel_[0] << ", " << object_vel_[1] << ", " << object_vel_[2]
                  << ", " << object_th_dot_ << "\n";
      object_data << "end, free m3, "
                  << "\n\n\n";
      once21 = true;
    }

    if (mode2_ == 4 && once21 == true) {
      once21 = false;
      once22 = true;
      object_data << "pre_post_time,    " << time_q.front() << ", " << current_time.toSec() << "\n";
      object_data << "box_properties,   " << box_.size_x << ", " << box_.size_y << ", " << box_.size_z << ", "
                  << box_.com_x << ", " << box_.com_y << ", " << box_.com_z << ", " << box_.mass << ", " << box_.mu
                  << ", " << box_.mu2 << ", " << box_.izz << "\n";
      object_data << "pre_hit_joints,   " << iiwa2_joint_angles_q.front()[0] << ", " << iiwa2_joint_angles_q.front()[1]
                  << ", " << iiwa2_joint_angles_q.front()[2] << ", " << iiwa2_joint_angles_q.front()[3] << ", "
                  << iiwa2_joint_angles_q.front()[4] << ", " << iiwa2_joint_angles_q.front()[5] << ", "
                  << iiwa2_joint_angles_q.front()[6] << "\n";
      object_data << "post_hit_joints,  " << iiwa2_joint_angles_[0] << ", " << iiwa2_joint_angles_[1] << ", "
                  << iiwa2_joint_angles_[2] << ", " << iiwa2_joint_angles_[3] << ", " << iiwa2_joint_angles_[4] << ", "
                  << iiwa2_joint_angles_[5] << ", " << iiwa2_joint_angles_[6] << "\n";
      object_data << "pre_hit_ee,       " << ee2_pose_q.front().position.x << ", " << ee2_pose_q.front().position.y
                  << ", " << ee2_pose_q.front().position.z << ", " << ee2_pose_q.front().orientation.x << ", "
                  << ee2_pose_q.front().orientation.y << ", " << ee2_pose_q.front().orientation.z << ", "
                  << ee2_pose_q.front().orientation.w << ", " << ee2_rpy_q.front()[0] << ", " << ee2_rpy_q.front()[1]
                  << ", " << ee2_rpy_q.front()[2] << ", " << ee2_twist_q.front().linear.x << ", "
                  << ee2_twist_q.front().linear.y << ", " << ee2_twist_q.front().linear.z << ", "
                  << ee2_twist_q.front().angular.x << ", " << ee2_twist_q.front().angular.y << ", "
                  << ee2_twist_q.front().angular.z << "\n";
      object_data << "post_hit_ee,      " << ee2_pose_.position.x << ", " << ee2_pose_.position.y << ", "
                  << ee2_pose_.position.z << ", " << ee2_pose_.orientation.x << ", " << ee2_pose_.orientation.y << ", "
                  << ee2_pose_.orientation.z << ", " << ee2_pose_.orientation.w << ", " << ee2_rpy_[0] << ", "
                  << ee2_rpy_[1] << ", " << ee2_rpy_[2] << ", " << ee2_twist_.linear.x << ", " << ee2_twist_.linear.y
                  << ", " << ee2_twist_.linear.z << ", " << ee2_twist_.angular.x << ", " << ee2_twist_.angular.y << ", "
                  << ee2_twist_.angular.z << "\n";
      object_data << "pred_stop_pos,    " << pred_time_q.front() << ", " << predict_pos_q.front()[0] << ", "
                  << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n";
      object_data << "pre_hit_object,   " << time_q.front() << ", " << object_pos_q.front()[0] << ", "
                  << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << ", " << object_theta_q.front()
                  << ", " << object_vel_q.front()[0] << ", " << object_vel_q.front()[1] << ", "
                  << object_vel_q.front()[2] << ", " << object_theta_dot_q.front() << "\n";
      object_data << "post_hit_object,  " << current_time.toSec() << ", " << object_pos_[0] << ", " << object_pos_[1]
                  << ", " << object_pos_[2] << ", " << object_th_ << ", " << object_vel_[0] << ", " << object_vel_[1]
                  << ", " << object_vel_[2] << ", " << object_th_dot_ << "\n";
      object_data << "post_hit_trajectory:\n";
      tracking = true;
      oneinten = 10;
    }
    if (mode1_ == 2 && object_pos_[1] < -0.1 && once22 == true) {
      once22 = false;
      tracking = false;
      object_data << "end, stopped"
                  << "\n\n\n";
      //object_data << "pred_stop_pos,   " << pred_time_q.front()<< ", " << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n";
      once11 = true;
    }
    if (mode1_ == 3 && once22 == true) {
      once22 = false;
      tracking = false;
      object_data << current_time.toSec() << ", " << object_pos_[0] << ", " << object_pos_[1] << ", " << object_pos_[2]
                  << ", " << object_th_ << ", " << object_vel_[0] << ", " << object_vel_[1] << ", " << object_vel_[2]
                  << ", " << object_th_dot_ << "\n";
      object_data << "end, free m3, "
                  << "\n\n\n";
      once11 = true;
    }
    if (tracking == true && oneinten >= 10) {
      oneinten = 0;
      object_data << current_time.toSec() << ", " << object_pos_[0] << ", " << object_pos_[1] << ", " << object_pos_[2]
                  << ", " << object_th_ << ", " << object_vel_[0] << ", " << object_vel_[1] << ", " << object_vel_[2]
                  << ", " << object_th_dot_ << "\n";
      if (object_vel_.norm() < 0.05) {
        tracking = false;
        if (once11 == false) {
          once12 = false;
          tracking = false;
          object_data << "end, free v=0, " << current_time.toSec() << "\n\n\n";
          once21 = true;
          once22 = true;
        }
        if (once21 == false) {
          once22 = false;
          tracking = false;
          object_data << "end, free v=0, " << current_time.toSec() << "\n\n\n";
          once11 = true;
          once12 = true;
        }
      }
    }
    oneinten++;

    object_data.close();

    ros::spinOnce();
    rate_.sleep();
  }
}

int main(int argc, char** argv) {
  //ROS Initialization
  ros::init(argc, argv, "collect_data");
  ros::NodeHandle nh_;
  float frequency = 100.0f;
  std::unique_ptr<CollectData> collect_data = std::make_unique<CollectData>(nh_, frequency);

  collect_data->init();
  collect_data->run();

  return 0;
}
