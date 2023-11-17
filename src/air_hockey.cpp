//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    website: lasa.epfl.ch

#include "air_hockey.h"

bool AirHockey::init() {

  get_private_param();

  // Get ros topics name
  std::string mode_iiwa1, mode_iiwa2, passive_ctrl_iiwa1_vel_quat, passive_ctrl_iiwa2_vel_quat,
      passive_ctrl_iiwa1_pos_quat, passive_ctrl_iiwa2_pos_quat, estimate_object, track_object, track_left_pose,
      track_left_eepose, track_right_pose, track_right_eepose, gazebo_model_states, gazebo_set_model_states,
      gazebo_link_states, iiwa1_inertia_topic, iiwa2_inertia_topic;
  if (!nh_.getParam("/mode/iiwa1", mode_iiwa1)) { ROS_ERROR("Param ros topic mode/iiwa1 not found"); }
  if (!nh_.getParam("/mode/iiwa2", mode_iiwa2)) { ROS_ERROR("Param ros topic mode/iiwa2 not found"); }
  if (!nh_.getParam("/passive_control/vel_quat/iiwa1", passive_ctrl_iiwa1_vel_quat)) {
    ROS_ERROR("Param ros topic /passive_control/vel_quat/iiwa1 not found");
  }
  if (!nh_.getParam("/passive_control/vel_quat/iiwa2", passive_ctrl_iiwa2_vel_quat)) {
    ROS_ERROR("Param ros topic /passive_control/vel_quat/iiwa2 not found");
  }
  if (!nh_.getParam("/passive_control/pos_quat/iiwa1", passive_ctrl_iiwa1_pos_quat)) {
    ROS_ERROR("Param ros topic /passive_control/pos_quat/iiwa1 not found");
  }
  if (!nh_.getParam("/passive_control/pos_quat/iiwa2", passive_ctrl_iiwa2_pos_quat)) {
    ROS_ERROR("Param ros topic /passive_control/pos_quat/iiwa2 not found");
  }
  if (!nh_.getParam("/estimate/object", estimate_object)) { ROS_ERROR("Param ros topic estimate/object not found"); }
  if (!nh_.getParam("/simo_track/object_pose", track_object)) {
    ROS_ERROR("Param ros topic /simo_track/object_pose not found");
  }
  if (!nh_.getParam("/simo_track/robot_left/pose", track_left_pose)) {
    ROS_ERROR("Param ros topic /simo_track/robot_left/pose not found");
  }
  if (!nh_.getParam("/simo_track/robot_left/ee_pose", track_left_eepose)) {
    ROS_ERROR("Param ros topic /simo_track/robot_left/ee_pose not found");
  }
  if (!nh_.getParam("/simo_track/robot_right/pose", track_right_pose)) {
    ROS_ERROR("Param ros topic /simo_track/robot_right/pose not found");
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
  if (!nh_.getParam("/Inertia/taskPos/iiwa1", iiwa1_inertia_topic)) {
    ROS_ERROR("Topic /iiwa1/inertia/taskPos not found");
  }
  if (!nh_.getParam("/Inertia/taskPos/iiwa2", iiwa2_inertia_topic)) {
    ROS_ERROR("Topic /iiwa2/inertia/taskPos not found");
  }

  center1_ << center1vec_[0], center1vec_[1], center1vec_[2];
  center2_ << center2vec_[0], center2vec_[1], center2vec_[2];
  ee_offset_ = {ee_offset_h_, ee_offset_v_};
  hittable_params_ = {x_reach_, y_reach_, x_offset_, y_offset_};

  //Init publishers
  pub_mode1_ = nh_.advertise<std_msgs::Int16>(mode_iiwa1, 1);
  pub_mode2_ = nh_.advertise<std_msgs::Int16>(mode_iiwa2, 1);
  pub_vel_quat1_ = nh_.advertise<geometry_msgs::Pose>(passive_ctrl_iiwa1_vel_quat, 1);
  pub_pos_quat1_ = nh_.advertise<geometry_msgs::Pose>(passive_ctrl_iiwa1_pos_quat, 1);
  pub_vel_quat2_ = nh_.advertise<geometry_msgs::Pose>(passive_ctrl_iiwa2_vel_quat, 1);
  pub_pos_quat2_ = nh_.advertise<geometry_msgs::Pose>(passive_ctrl_iiwa2_pos_quat, 1);

  //Init subscribers
  estimate_object_subs_ = nh_.subscribe(estimate_object, 10, &AirHockey::estimateObjectCallback, this);
  mode_sub_ = nh_.subscribe("/key_ctrl_mode", 10, &AirHockey::modeCallback, this);
  iiwa1_inertia_ = nh_.subscribe(iiwa1_inertia_topic,
                                 1,
                                 &AirHockey::iiwa1InertiaCallback,
                                 this,
                                 ros::TransportHints().reliable().tcpNoDelay());
  iiwa2_inertia_ = nh_.subscribe(iiwa2_inertia_topic,
                                 1,
                                 &AirHockey::iiwa2InertiaCallback,
                                 this,
                                 ros::TransportHints().reliable().tcpNoDelay());

  if (object_real_) {
    object_subs_ = nh_.subscribe(track_object, 10, &AirHockey::objectCallback, this);
  } else {
    object_subs_ = nh_.subscribe(gazebo_model_states, 10, &AirHockey::objectSimCallback, this);
    //Client to reset object pose
    ros::service::waitForService(gazebo_set_model_states);
    set_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(gazebo_set_model_states);
  }

  if (iiwa_real_) {
    iiwa1_base_subs_ = nh_.subscribe(track_left_pose, 10, &AirHockey::iiwa1BaseCallback, this);
    iiwa2_base_subs_ = nh_.subscribe(track_right_pose, 10, &AirHockey::iiwa2BaseCallback, this);
    iiwa1_ee_subs_ = nh_.subscribe(track_left_eepose, 10, &AirHockey::iiwa1EEPoseCallback, this);
    iiwa2_ee_subs_ = nh_.subscribe(track_right_eepose, 10, &AirHockey::iiwa2EEPoseCallback, this);

    R_Opti_ << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    threshold_ee_ready_ = 0.18;
  } else {
    iiwa_subs_ = nh_.subscribe(gazebo_link_states, 10, &AirHockey::iiwaSimCallback, this);

    R_Opti_ << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    threshold_ee_ready_ = 0.05;
  }

  // dynamic configure:
  dynRecCallback_ = boost::bind(&AirHockey::param_cfg_callback, this, _1, _2);
  dynRecServer_.setCallback(dynRecCallback_);

  // EE rotation
  R_EE_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  return true;
}

void AirHockey::run() {

  msg_mode1_.data = mode1_;
  msg_mode2_.data = mode2_;
  prev_mode1_ = mode1_;
  prev_mode2_ = mode2_;

  while (ros::ok()) {
    switch_both_mode();

    msg_mode1_.data = mode1_;
    pub_mode1_.publish(msg_mode1_);
    msg_mode2_.data = mode2_;
    pub_mode2_.publish(msg_mode2_);

    move_robot(mode1_, 1);
    move_robot(mode2_, 2);

    prev_mode1_ = mode1_;
    prev_mode2_ = mode2_;

    std::tie(hitta1_, farra1_) = hittable(object_pos_, center1_, center2_, hittable_params_);
    std::tie(hitta2_, farra2_) = hittable(object_pos_, center2_, center1_, hittable_params_);

    if (object_real_ == false && object_vel_.norm() < 0.01 && (!hitta1_ && !hitta2_)) { reset_object_position(); }

    if (debug_) {
      if (mode1_ == 1) { ROS_INFO_STREAM("1 HIT" << ee1_pos_init_); }
      if (mode1_ == 2) { ROS_INFO_STREAM("1 AFTERHIT" << object_pos_init1_); }
      if (mode2_ == 1) { ROS_INFO_STREAM("2 HIT  " << ee2_pos_init_); }
      if (mode2_ == 2) { ROS_INFO("2 AFTERHIT"); }
      ROS_INFO_STREAM("hittable, too_far: " << hitta1_ << farra1_ << hitta2_ << farra2_);
    }

    ros::spinOnce();
    rate_.sleep();
  }
  ros::spinOnce();
  rate_.sleep();
  ros::shutdown();
}

void AirHockey::get_private_param() {
  // Get environment settings
  if (!nh_.getParam("hollow", hollow_)) { ROS_ERROR("Param hollow not found"); }
  if (!nh_.getParam("iiwa_real", iiwa_real_)) { ROS_ERROR("Param iiwa_real not found"); }
  if (!nh_.getParam("manual_mode", manual_mode_)) { ROS_ERROR("Param manual_mode not found"); }
  if (!nh_.getParam("object_real", object_real_)) { ROS_ERROR("Param object_real not found"); }
  if (!nh_.getParam("debug", debug_)) { debug_ = false; }
  if (!nh_.getParam("inertia", inertia_)) { inertia_ = false; }
  if (!nh_.getParam("is_hit_1", is_hit_1)) { is_hit_1 = false; }
  if (!nh_.getParam("is_hit_2", is_hit_2)) { is_hit_2 = false; }
  

  //Get center points of desired workspaces (used to aim for and tell what orientation)
  if (!nh_.getParam("center1", center1vec_)) { ROS_ERROR("Param center1 not found"); }
  if (!nh_.getParam("center2", center2vec_)) { ROS_ERROR("Param center2 not found"); }
  if (!nh_.getParam("ee_offset/h", ee_offset_h_)) { ROS_ERROR("Param ee_offset/h not found"); }
  if (!nh_.getParam("ee_offset/v", ee_offset_v_)) { ROS_ERROR("Param ee_offset/v not found"); }
  if (!nh_.getParam("hittable/reach/x", x_reach_)) { ROS_ERROR("Param hittable/reach/x not found"); }
  if (!nh_.getParam("hittable/reach/y", y_reach_)) { ROS_ERROR("Param hittable/reach/y not found"); }
  if (!nh_.getParam("hittable/offset/x", x_offset_)) { ROS_ERROR("Param hittable/offset/x not found"); }
  if (!nh_.getParam("hittable/offset/y", y_offset_)) { ROS_ERROR("Param hittable/offset/y not found"); }
  if (!nh_.getParam("hit/speed", des_speed_)) { ROS_ERROR("Param hit/speed not found"); }
  if (!nh_.getParam("hit/flux", des_flux_)) { ROS_ERROR("Param hit/speed not found"); }
}

void AirHockey::param_cfg_callback(i_am_project::workspace_paramsConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure request.. Updating the parameters ... ");
  double center1vec_X = config.center1_x;
  double center1vec_Y = config.center1_y;
  double center1vec_Z = config.center1_z;
  double center2vec_X = config.center2_x;
  double center2vec_Y = config.center2_y;
  double center2vec_Z = config.center2_z;

  double ee_offset_H = config.ee_offset_h;
  double ee_offset_V = config.ee_offset_v;
  ROS_INFO_STREAM("center1vec_X  " << center1vec_X);
  ROS_INFO_STREAM("center1vec_Y  " << center1vec_Y);
  ROS_INFO_STREAM("center1vec_Z  " << center1vec_Z);
  ROS_INFO_STREAM("center2vec_X  " << center2vec_X);
  ROS_INFO_STREAM("center2vec_Y  " << center2vec_Y);
  ROS_INFO_STREAM("center2vec_Z  " << center2vec_Z);

  center1_ << center1vec_X, center1vec_Y, center1vec_Z;
  center2_ << center2vec_X, center2vec_Y, center2vec_Z;
  ee_offset_ = {ee_offset_H, ee_offset_V};
}

void AirHockey::reset_object_position() {
  gazebo_msgs::SetModelState setmodelstate;
  geometry_msgs::Pose new_box_pose;

  nh_.getParam("box/initial_pos/x", new_box_pose.position.x);
  nh_.getParam("box/initial_pos/y", new_box_pose.position.y);
  nh_.getParam("box/initial_pos/z", new_box_pose.position.z);
  new_box_pose.orientation.x = 0.0;
  new_box_pose.orientation.y = 0.0;
  new_box_pose.orientation.z = 0.0;
  new_box_pose.orientation.w = 0.0;

  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = (std::string) "my_box";
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = new_box_pose;
  setmodelstate.request.model_state = modelstate;
  set_state_client_.call(setmodelstate);

  if (hollow_ == true) {
    //Set new pose of minibox
    geometry_msgs::Pose new_mini_pose;
    nh_.getParam("mini/initial_pos/x", new_mini_pose.position.x);
    nh_.getParam("mini/initial_pos/y", new_mini_pose.position.y);
    nh_.getParam("mini/initial_pos/z", new_mini_pose.position.z);
    new_mini_pose.position.x += new_box_pose.position.x;
    new_mini_pose.position.y += new_box_pose.position.y;
    new_mini_pose.position.z += new_box_pose.position.z;
    new_mini_pose.orientation.x = 0.0;
    new_mini_pose.orientation.y = 0.0;
    new_mini_pose.orientation.z = 0.0;
    new_mini_pose.orientation.w = 0.0;

    modelstate.model_name = (std::string) "my_mini";
    modelstate.pose = new_mini_pose;
    setmodelstate.request.model_state = modelstate;
    set_state_client_.call(setmodelstate);
  }

  ROS_INFO("Resetting object pose");
  ros::Duration(0.5).sleep();
}

void AirHockey::move_robot(int mode, int robot_id) {
  switch (mode) {
  
    case 1://hit
      if (robot_id == 1) {
          pub_vel_quat1_.publish(
              hitDSInertia(des_flux_, object_pos_, center2_, ee1_pos_, ee_offset_, iiwa1_task_inertia_pos_));
          object_pos_init1_ = object_pos_;//need this for post-hit to guide the arm right after hit
      } 
      else if (robot_id == 2) {
          pub_vel_quat2_.publish(
              hitDSInertia(des_flux_, object_pos_, center1_, ee2_pos_, ee_offset_, iiwa2_task_inertia_pos_));
          object_pos_init2_ = object_pos_;
      }
      break;
    case 2://post hit
      if (robot_id == 1) {
        std::cout << "here" <<std::endl;
        // pub_pos_quat1_.publish(postHit(object_pos_init1_, center2_, iiwa1_base_pos_, ee_offset_));
        pub_pos_quat1_.publish(postHitDS(object_pos_init1_, center2_, iiwa1_base_pos_, ee_offset_, center1_));
        std::cout<< center1_ << std::endl;
      } else if (robot_id == 2) {
        // pub_pos_quat1_.publish(postHit(object_pos_init1_, center1_, iiwa1_base_pos_, ee_offset_));
        pub_pos_quat2_.publish(postHitDS(object_pos_init2_, center1_, iiwa2_base_pos_, ee_offset_, center2_));
      }
      break;
  }
}

void AirHockey::switch_both_mode() {

  mode1_ = maniModeSelektor_2(object_pos_,
                              object_pos_init1_,
                              ee1_pos_,
                              center1_,
                              center2_,
                              ee_offset_,
                              hittable_params_,
                              prev_mode1_, 1, 
                              is_hit_1, is_hit_2);

  mode2_ = maniModeSelektor_2(object_pos_,
                              object_pos_init2_,
                              ee2_pos_,
                              center2_,
                              center1_,
                              ee_offset_,
                              hittable_params_,
                              prev_mode2_, 2,
                              is_hit_1, is_hit_2);
}

int AirHockey::getIndex(std::vector<std::string> v, std::string value) {
  for (int i = 0; i < v.size(); i++) {
    if (v[i].compare(value) == 0) return i;
  }
  return -1;
}

//Gazebo
void AirHockey::objectSimCallback(const gazebo_msgs::ModelStates model_states) {
  int box_index = getIndex(model_states.name, "my_box");

  geometry_msgs::Pose object_pose_msg = model_states.pose[box_index];
  geometry_msgs::Twist object_twist = model_states.twist[box_index];
  Eigen::Vector3d object_rpy = quatToRPY({object_pose_msg.orientation.w,
                                          object_pose_msg.orientation.x,
                                          object_pose_msg.orientation.y,
                                          object_pose_msg.orientation.z});

  object_pos_ << object_pose_msg.position.x, object_pose_msg.position.y, object_pose_msg.position.z;
  object_th_mod_ =
      std::fmod(object_rpy[2] + M_PI + M_PI / 4, M_PI / 2) - M_PI / 4;//get relative angle of box face facing the arm
}

void AirHockey::iiwaSimCallback(const gazebo_msgs::LinkStates link_states) {
  int iiwa1_ee_index = getIndex(link_states.name, "iiwa1::iiwa1_link_7");
  int iiwa2_ee_index = getIndex(link_states.name, "iiwa2::iiwa2_link_7");
  int iiwa1_base_index = getIndex(link_states.name, "iiwa1::iiwa1_link_0");
  int iiwa2_base_index = getIndex(link_states.name, "iiwa2::iiwa2_link_0");

  geometry_msgs::Pose ee1_pose, ee2_pose, iiwa1_base_pose, iiwa2_base_pose;
  ee1_pose = link_states.pose[iiwa1_ee_index];
  ee2_pose = link_states.pose[iiwa2_ee_index];
  ee1_pos_ << ee1_pose.position.x, ee1_pose.position.y, ee1_pose.position.z;
  ee2_pos_ << ee2_pose.position.x, ee2_pose.position.y, ee2_pose.position.z;

  iiwa1_base_pose = link_states.pose[iiwa1_base_index];
  iiwa2_base_pose = link_states.pose[iiwa2_base_index];
  iiwa1_base_pos_ << iiwa1_base_pose.position.x, iiwa1_base_pose.position.y, iiwa1_base_pose.position.z;
  iiwa2_base_pos_ << iiwa2_base_pose.position.x, iiwa2_base_pose.position.y, iiwa2_base_pose.position.z;
}

//Optitrack
void AirHockey::objectCallback(const geometry_msgs::PoseStamped object_pose) {
  object_pos_ << object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z;
  object_pos_ = R_Opti_ * object_pos_;
  Eigen::Vector3d object_rpy = quatToRPY({object_pose.pose.orientation.w,
                                          object_pose.pose.orientation.x,
                                          object_pose.pose.orientation.y,
                                          object_pose.pose.orientation.z});

  object_th_mod_ =
      std::fmod(object_rpy[2] + M_PI + M_PI / 4, M_PI / 2) - M_PI / 4;//get relative angle of box face facing the arm
}

void AirHockey::iiwa1BaseCallback(const geometry_msgs::PoseStamped base_pose) {
  iiwa1_base_pos_ << base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z;
  iiwa1_base_pos_ = R_Opti_ * iiwa1_base_pos_;
}

void AirHockey::iiwa2BaseCallback(const geometry_msgs::PoseStamped base_pose) {
  iiwa2_base_pos_ << base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z;
  iiwa2_base_pos_ = R_Opti_ * iiwa2_base_pos_;
}

void AirHockey::iiwa1EEPoseCallback(const geometry_msgs::Pose ee_pose) {
  ee1_pos_ << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z;
  ee1_pos_ = R_EE_ * ee1_pos_;
  ee1_pos_ = ee1_pos_ + iiwa1_base_pos_;
}

void AirHockey::iiwa2EEPoseCallback(const geometry_msgs::Pose ee_pose) {
  // Get ee pose in iiwa ref (origin at iiwa) - need to transform it
  ee2_pos_ << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z;
  ee2_pos_ = R_EE_ * ee2_pos_;
  ee2_pos_ = ee2_pos_ + iiwa2_base_pos_;
}

void AirHockey::iiwa1InertiaCallback(const geometry_msgs::Inertia& inertia_msg) {
  iiwa1_task_inertia_pos_(0, 0) = inertia_msg.ixx;
  iiwa1_task_inertia_pos_(1, 1) = inertia_msg.iyy;
  iiwa1_task_inertia_pos_(2, 2) = inertia_msg.izz;
  iiwa1_task_inertia_pos_(0, 1) = inertia_msg.ixy;
  iiwa1_task_inertia_pos_(1, 0) = inertia_msg.ixy;
  iiwa1_task_inertia_pos_(0, 2) = inertia_msg.ixz;
  iiwa1_task_inertia_pos_(2, 0) = inertia_msg.ixz;
  iiwa1_task_inertia_pos_(1, 2) = inertia_msg.iyz;
  iiwa1_task_inertia_pos_(2, 1) = inertia_msg.iyz;
}

void AirHockey::iiwa2InertiaCallback(const geometry_msgs::Inertia& inertia_msg) {
  iiwa2_task_inertia_pos_(0, 0) = inertia_msg.ixx;
  iiwa2_task_inertia_pos_(1, 1) = inertia_msg.iyy;
  iiwa2_task_inertia_pos_(2, 2) = inertia_msg.izz;
  iiwa2_task_inertia_pos_(0, 1) = inertia_msg.ixy;
  iiwa2_task_inertia_pos_(1, 0) = inertia_msg.ixy;
  iiwa2_task_inertia_pos_(0, 2) = inertia_msg.ixz;
  iiwa2_task_inertia_pos_(2, 0) = inertia_msg.ixz;
  iiwa2_task_inertia_pos_(1, 2) = inertia_msg.iyz;
  iiwa2_task_inertia_pos_(2, 1) = inertia_msg.iyz;
}

//i_am_predict or estimate_sim
void AirHockey::estimateObjectCallback(std_msgs::Float64MultiArray estimation) {
  ETA_ = estimation.data[1];
  predict_pos_ << estimation.data[2], estimation.data[3], estimation.data[4];
  object_vel_ << estimation.data[5], estimation.data[6], estimation.data[7];
  if (iiwa_real_) {
    predict_pos_ = R_Opti_ * predict_pos_;
    object_vel_ = R_Opti_ * object_vel_;
  }
}

//manual control
void AirHockey::modeCallback(std_msgs::Int16 msg) { key_ctrl_ = msg.data; }

int AirHockey::maniModeSelektor_2(Eigen::Vector3d object_pos,
                                Eigen::Vector3d object_pos_init,
                                Eigen::Vector3d ee_pos,
                                Eigen::Vector3d center1,
                                Eigen::Vector3d center2,
                                Eigen::Vector2d ee_offset,
                                Eigen::Vector4d hittable_params,
                                const int prev_mode,
                                const int iiwa_no,
                                bool &is_hit_1,
                                bool &is_hit_2) {
  int mode = prev_mode;// if none of the conditions are met, mode remains the same

  Eigen::Vector3d d_center = center2 - center1;
  Eigen::Vector3d d_points = center2 - object_pos;
  Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};

  bool cur_hittable, cur_too_far;
  std::tie(cur_hittable, cur_too_far) = hittable(object_pos, center1, center2, hittable_params);

  bool ee_hit;
  if (ee_pos.dot(d_center) > object_pos_init.dot(d_center) - 0.13) {
    ee_hit = true;
  } else {
    ee_hit = false;
  }

  if (iiwa_no == 1){is_hit_1 = ee_hit;}
  if (iiwa_no == 2){is_hit_2 = ee_hit;}

  // switch (prev_mode) {
  //   case 1://hit
  //     // sum of count is even for iiwa 1 and odd for iiwa 2 to hit
  //     if (iiwa_no == 1 && is_hit_1 == true && is_hit_2 == false){
  //       mode = 2;
  //       // sleep(2);
  //       } 
  //     if (iiwa_no == 2 && is_hit_2 == true && is_hit_1 == false){
  //       mode = 2; 
  //       // sleep(2);
  //     }
  //     break;

  //   case 2://post hit
  //     if (!cur_hittable) { 
  //       mode = 2; 
  //       // sleep(2);
  //     } //if object has left the range of arm, go to rest
  //     if (cur_hittable) {
  //       mode = 1;
  //       // sleep(2);
  //     }
  //     break;

    switch (prev_mode) {
    case 1://hit
      // sum of count is even for iiwa 1 and odd for iiwa 2 to hit
      if (iiwa_no == 1 && is_hit_1 == true && is_hit_2 == false){
        mode = 2;
        // sleep(2);
        } 
      if (iiwa_no == 2){
        mode = 2; 
        // sleep(2);
      }
      break;

    case 2://post hit
      if (iiwa_no ==1 && is_hit_1 == false && is_hit_2 == false){
      mode = 1;}
      else {
        mode = 2;
      }
      break;

  }
  return mode;
}

void AirHockey::updateRobotState(){

  // Determine phase based on robots state + object position and movement

  // TODO : add key control

  if(state_robot1_ == REST && state_robot2_ == REST && state_object_ == STOPPED_IN_1){
    state_robot1_ = HIT;}


  if(state_robot1_ == HIT && state_robot2_ == REST && state_object_ == MOVING_TO_1){
    state_robot1_ = REST;}


  if(state_robot1_ == REST && state_robot2_ == REST && state_object_ == STOPPED_IN_2){
    state_robot2_ = HIT;}

  if(state_robot1_ == REST && state_robot2_ == HIT && state_object_ == MOVING_TO_2){
    state_robot2_ = REST;}

}

void AirHockey::move_robot_updated(robotState state, int robot_id) {
  
  switch (state) {
    case HIT://hit
      if (robot_id == 1) {
          pub_vel_quat1_.publish(
              hitDSInertia(des_flux_, object_pos_, center2_, ee1_pos_, ee_offset_, iiwa1_task_inertia_pos_));
          object_pos_init1_ = object_pos_;//need this for post-hit to guide the arm right after hit
      } 
      else if (robot_id == 2) {
          pub_vel_quat2_.publish(
              hitDSInertia(des_flux_, object_pos_, center1_, ee2_pos_, ee_offset_, iiwa2_task_inertia_pos_));
          object_pos_init2_ = object_pos_;
      }
      break;
    case REST://post hit
      if (robot_id == 1) {
        pub_pos_quat1_.publish(postHit(center1_, center1_, iiwa1_base_pos_, ee_offset_));
      } else if (robot_id == 2) {
        pub_pos_quat2_.publish(postHit(center2_, center2_, iiwa2_base_pos_, ee_offset_));
      }
      break;
  }
}

void AirHockey::run_updated() {

  msg_mode1_.data = state_robot1_;
  msg_mode2_.data = state_robot2_;
  prev_mode1_ = state_robot1_;
  prev_mode2_ = state_robot1_;

  prev_object_state_ = state_object_;

  while (ros::ok()) {

    // Update object status (based on position and velocity) 
    get_object_state();

    // Update state machine
    if(state_object_ != prev_object_state_) { 
      ROS_INFO_STREAM("updating robot state");
      updateRobotState();
      prev_object_state_ = state_object_;}

    // Publish robot state (WHY?)
    msg_mode1_.data = state_robot1_;
    pub_mode1_.publish(msg_mode1_);
    msg_mode2_.data = state_robot2_;
    pub_mode2_.publish(msg_mode2_);

    // Send commands to robots
    move_robot(state_robot1_, 1);
    move_robot(state_robot1_, 2);

    prev_mode1_ = state_robot1_;
    prev_mode2_ = state_robot1_;

    if (object_real_ == false && object_vel_.norm() < 0.01 && (!hitta1_ && !hitta2_)) { reset_object_position(); }

    if (debug_) {
      ROS_INFO_STREAM("STATES [robot1, robot2, object]: " << state_robot1_ << state_robot2_ << state_object_);
      ROS_INFO_STREAM("count_for_testing: " << count_for_testing);
    }

    ros::spinOnce();
    rate_.sleep();
  }
  ros::spinOnce();
  rate_.sleep();
  ros::shutdown();
}

void AirHockey::get_object_state(){

  // object should change state when hit and when it stops moving (add some delay OR key control )

  // USe counter for test here;
  if(count_for_testing%3 == 1){
    state_object_ = static_cast<objectState>((state_object_+ 1) % (MOVING_TO_1 + 1));;
  }
  count_for_testing+=1;

}

int main(int argc, char** argv) {

  //ROS Initialization
  ros::init(argc, argv, "air_hockey");
  ros::NodeHandle nh_;
  float frequency = 100.0f;

  std::unique_ptr<AirHockey> play_air_hockey = std::make_unique<AirHockey>(nh_, frequency);

  if (!play_air_hockey->init()) {
    return -1;
  } else {
    ros::Duration(3).sleep();
    play_air_hockey->run();
  }
}