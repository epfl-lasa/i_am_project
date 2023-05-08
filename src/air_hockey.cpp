//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    website: lasa.epfl.ch

#include "air_hockey.h"

bool AirHockey::init() {
  // Get environment settings
  if (!nh_.getParam("hollow", hollow_)) { ROS_ERROR("Param hollow not found"); }
  if (!nh_.getParam("iiwa_real", iiwa_real_)) { ROS_ERROR("Param iiwa_real not found"); }
  if (!nh_.getParam("manual_mode", manual_mode_)) { ROS_ERROR("Param manual_mode not found"); }
  if (!nh_.getParam("object_real", object_real_)) { ROS_ERROR("Param object_real not found"); }

  //Get center points of desired workspaces (used to aim for and tell what orientation)
  if (!nh_.getParam("center1", center1vec_)) { ROS_ERROR("Param center1 not found"); }
  if (!nh_.getParam("center2", center2vec_)) { ROS_ERROR("Param center2 not found"); }
  if (!nh_.getParam("ee_offset/h", ee_offset_h_)) { ROS_ERROR("Param ee_offset/h not found"); }
  if (!nh_.getParam("ee_offset/v", ee_offset_v_)) { ROS_ERROR("Param ee_offset/v not found"); }
  if (!nh_.getParam("hittable/reach/x", x_reach_)) { ROS_ERROR("Param hittable/reach/x not found"); }
  if (!nh_.getParam("hittable/reach/y", y_reach_)) { ROS_ERROR("Param hittable/reach/y not found"); }
  if (!nh_.getParam("hittable/offset/x", x_offset_)) { ROS_ERROR("Param hittable/offset/x not found"); }
  if (!nh_.getParam("hittable/offset/y", y_offset_)) { ROS_ERROR("Param hittable/offset/y not found"); }
  nh_.getParam("hit/speed", des_speed_);
  //   ros::Duration(3).sleep(); // TODO WHY?? (LINE 240)

  center1_ << center1vec_[0], center1vec_[1], center1vec_[2];
  center2_ << center2vec_[0], center2vec_[1], center2vec_[2];
  ee_offset_ = {ee_offset_h_, ee_offset_v_};
  hittable_params_ = {x_reach_, y_reach_, x_offset_, y_offset_};

  //Init publishers
  pub_mode1_ = nh_.advertise<std_msgs::Int16>("/mode/iiwa1", 1);
  pub_mode2_ = nh_.advertise<std_msgs::Int16>("/mode/iiwa2", 1);
  pub_vel_quat1_ = nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/vel_quat", 1);
  pub_pos_quat1_ = nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/pos_quat", 1);
  pub_vel_quat2_ = nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/vel_quat", 1);
  pub_pos_quat2_ = nh_.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/pos_quat", 1);

  //Init subscribers
  estimate_object_subs_ = nh_.subscribe("estimate/object", 10, &AirHockey::estimateObjectCallback, this);
  mode_sub_ = nh_.subscribe("mode", 10, &AirHockey::modeCallback, this);//Subscriber to key control
  if (object_real_) {
    object_subs_ = nh_.subscribe("/simo_track/object_pose", 10, &AirHockey::objectCallback, this);
  } else {
    object_subs_ = nh_.subscribe("/gazebo/model_states", 10, &AirHockey::objectSimCallback, this);
    //Client to reset object pose
    ros::service::waitForService("gazebo/set_model_state");
    set_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  }

  if (iiwa_real_) {
    iiwa1_base_subs_ = nh_.subscribe("/simo_track/robot_left/pose", 10, &AirHockey::iiwa1BaseCallback, this);
    iiwa2_base_subs_ = nh_.subscribe("/simo_track/robot_right/pose", 10, &AirHockey::iiwa2BaseCallback, this);
    iiwa1_ee_subs_ = nh_.subscribe("/simo_track/robot_left/ee_pose", 10, &AirHockey::iiwa1EEPoseCallback, this);
    iiwa2_ee_subs_ = nh_.subscribe("/simo_track/robot_right/ee_pose", 10, &AirHockey::iiwa2EEPoseCallback, this);
  } else {
    iiwa_subs_ = nh_.subscribe("/gazebo/link_states", 10, &AirHockey::iiwaSimCallback, this);
  }

  // TODO IS THIS NEEDED??
  //   Set hitting speed and direction. Once we change attractor to something more related to the game, we no longer need this
  //   std::cout << "Enter the desired speed of hitting (between 0 and 1)" << std::endl;
  //   std::cin >> des_speed;
  //     double theta;
  //     nh_.getParam("hit/direction", theta);
  //   while(des_speed < 0 || des_speed > 3){
  //     std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
  //     std::cin >> des_speed;
  //   }
  //   std::cout << "Enter the desired direction of hitting (between -pi/2 and pi/2)" << std::endl;
  //   std::cin >> theta;
  //   while(theta < -M_PI_2 || theta > M_PI_2){
  //     std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
  //     std::cin >> theta;
  //   }

  return true;
}

void AirHockey::run() {

  msg_mode1_.data = mode1_;
  msg_mode2_.data = mode2_;
  prev_mode1_ = mode1_;
  prev_mode2_ = mode2_;

  R_EE_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  R_Opti_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

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
    // Some infos
    std::stringstream ss1;
    std::stringstream ss2;

    ss1 << "mode1: " << mode1_;
    ss2 << "mode2: " << mode2_;

    ROS_INFO("%s", ss1.str().c_str());
    ROS_INFO("%s", ss2.str().c_str());

    ros::spinOnce();
    rate_.sleep();
  }
  ros::spinOnce();
  rate_.sleep();
  ros::shutdown();
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

void AirHockey::move_robot(int mode, int mode_id) {
  switch (mode) {
    case 1://track
      if (mode_id == 1) {
        pub_pos_quat1_.publish(track(predict_pos_, center2_, ee_offset_, iiwa1_base_pos_));
        ee1_pos_init_ = ee1_pos_;//we can go from track to hit. For hit, we need initial
      } else if (mode_id == 2) {
        pub_pos_quat2_.publish(track(predict_pos_, center1_, ee_offset_, iiwa2_base_pos_));
        ee2_pos_init_ = ee2_pos_;
      }
      break;
    case 2://stop
      if (mode_id == 1) {
        ROS_INFO_STREAM("______ STOP MODE CASE 2");
        pub_pos_quat1_.publish(block(object_pos_, predict_pos_, center1_, center2_, object_th_mod_, iiwa1_base_pos_));
      } else if (mode_id == 2) {
        pub_pos_quat2_.publish(block(object_pos_, predict_pos_, center2_, center1_, object_th_mod_, iiwa2_base_pos_));
      }
      break;
    case 3://hit
      if (mode_id == 1) {
        pub_vel_quat1_.publish(hitDS(des_speed_, object_pos_, center2_, ee1_pos_, ee1_pos_init_));
        object_pos_init1_ = object_pos_;//need this for post-hit to guide the arm right after hit
      } else if (mode_id == 2) {
        pub_vel_quat2_.publish(hitDS(des_speed_, object_pos_, center1_, ee2_pos_, ee2_pos_init_));
        object_pos_init2_ = object_pos_;
      }
      break;
    case 4://post hit
      if (mode_id == 1) {
        pub_pos_quat1_.publish(postHit(object_pos_init1_, center2_, iiwa1_base_pos_));
      } else if (mode_id == 2) {
        pub_pos_quat2_.publish(postHit(object_pos_init2_, center1_, iiwa2_base_pos_));
      }
      break;
    case 5://rest
      if (mode_id == 1) {
        pub_pos_quat1_.publish(rest(center1_, center2_, ee_offset_, iiwa1_base_pos_));
        ee1_pos_init_ = ee1_pos_;//we can also go from rest to hit..
      } else if (mode_id == 2) {
        pub_pos_quat2_.publish(rest(center2_, center1_, ee_offset_, iiwa2_base_pos_));
        ee2_pos_init_ = ee2_pos_;
      }
      break;
  }
}

void AirHockey::switch_both_mode() {
  if (manual_mode_) {
    mode1_ = maniModeSelektor(object_pos_,
                              object_pos_init1_,
                              object_vel_,
                              ee1_pos_,
                              center1_,
                              center2_,
                              ee_offset_,
                              hittable_params_,
                              prev_mode1_,
                              key_ctrl_,
                              1);

    mode2_ = maniModeSelektor(object_pos_,
                              object_pos_init2_,
                              object_vel_,
                              ee2_pos_,
                              center2_,
                              center1_,
                              ee_offset_,
                              hittable_params_,
                              prev_mode2_,
                              key_ctrl_,
                              2);
  } else {
    mode1_ = modeSelektor(object_pos_,
                          object_pos_init1_,
                          object_vel_,
                          predict_pos_,
                          ETA_,
                          ee1_pos_,
                          center1_,
                          center2_,
                          ee_offset_,
                          hittable_params_,
                          prev_mode1_);

    mode2_ = modeSelektor(object_pos_,
                          object_pos_init2_,
                          object_vel_,
                          predict_pos_,
                          ETA_,
                          ee2_pos_,
                          center2_,
                          center1_,
                          ee_offset_,
                          hittable_params_,
                          prev_mode2_);
  }
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

  // TODO NEEDED OR DELETE?
  // min_y_ = iiwa2_base_pos_[1] - 0.5;
  // max_y_ = iiwa2_base_pos_[1] - 0.1;
}

//Optitrack
void AirHockey::objectCallback(const geometry_msgs::Pose object_pose) {
  object_pos_ << object_pose.position.x, object_pose.position.y, object_pose.position.z;
  object_pos_ = R_Opti_ * object_pos_;
  Eigen::Vector3d object_rpy = quatToRPY(
      {object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z});

  object_th_mod_ =
      std::fmod(object_rpy[2] + M_PI + M_PI / 4, M_PI / 2) - M_PI / 4;//get relative angle of box face facing the arm
}

void AirHockey::iiwa1BaseCallback(const geometry_msgs::Pose base_pose) {
  iiwa1_base_pos_ << base_pose.position.x, base_pose.position.y, base_pose.position.z;
  iiwa1_base_pos_ = R_Opti_ * iiwa1_base_pos_;
}

void AirHockey::iiwa2BaseCallback(const geometry_msgs::Pose base_pose) {
  iiwa2_base_pos_ << base_pose.position.x, base_pose.position.y, base_pose.position.z;
  iiwa2_base_pos_ = R_Opti_ * iiwa2_base_pos_;

  // TODO NEEDED OR DELETE?
  // min_y_ = iiwa2_base_pos_[1] - 0.5;
  // max_y_ = iiwa2_base_pos_[1] - 0.1;
}

void AirHockey::iiwa1EEPoseCallback(const geometry_msgs::Pose ee_pose) {
  // TODO NEEDED OR DELETE?
  // geometry_msgs::Pose ee1_pose;
  // ee1_pose.position.x = ee_pose.position.x;
  // ee1_pose.position.y = ee_pose.position.y;
  // ee1_pose.position.z = ee_pose.position.z;
  // ee1_pose.orientation.w = ee_pose.orientation.w;
  // ee1_pose.orientation.x = ee_pose.orientation.x;
  // ee1_pose.orientation.y = ee_pose.orientation.y;
  // ee1_pose.orientation.z = ee_pose.orientation.z;
  ee1_pos_ << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z;
  ee1_pos_ = R_EE_ * ee1_pos_;
}

void AirHockey::iiwa2EEPoseCallback(const geometry_msgs::Pose ee_pose) {
  // TODO NEEDED OR DELETE?
  // geometry_msgs::Pose ee2_pose;
  //   ee2_pose.position.x = ee_pose.position.x;
  //   ee2_pose.position.y = ee_pose.position.y;
  //   ee2_pose.position.z = ee_pose.position.z;
  //   ee2_pose.orientation.w = ee_pose.orientation.w;
  //   ee2_pose.orientation.x = ee_pose.orientation.x;
  //   ee2_pose.orientation.y = ee_pose.orientation.y;
  //   ee2_pose.orientation.z = ee_pose.orientation.z;
  ee2_pos_ << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z;
  ee2_pos_ = R_EE_ * ee2_pos_;
}

//i_am_predict or estimate_sim
void AirHockey::estimateObjectCallback(std_msgs::Float64MultiArray estimation) {
  // TODO NEVER USED DELETE
  // double stdev;
  // stdev = estimation.data[0];
  //predict_th = estimation.data[8];
  ETA_ = estimation.data[1];
  predict_pos_ << estimation.data[2], estimation.data[3], estimation.data[4];
  object_vel_ << estimation.data[5], estimation.data[6], estimation.data[7];
}

//manual control
void AirHockey::modeCallback(std_msgs::Int16 msg) { key_ctrl_ = msg.data; }

int AirHockey::modeSelektor(Eigen::Vector3d object_pos,
                            Eigen::Vector3d object_pos_init,
                            Eigen::Vector3d object_vel,
                            Eigen::Vector3d predict_pos,
                            double ETA,
                            Eigen::Vector3d ee_pos,
                            Eigen::Vector3d center1,
                            Eigen::Vector3d center2,
                            Eigen::Vector2d ee_offset,
                            Eigen::Vector4d hittable_params,
                            const int prev_mode) {
  // TODO COMMENTS REVIEW
  int mode = prev_mode;// if none of the conditions are met, mode remains the same

  Eigen::Vector3d d_center = center2 - center1;
  Eigen::Vector3d d_points = center2 - object_pos;
  Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};

  bool cur_hittable, cur_too_far, pred_hittable, pred_too_far;
  std::tie(cur_hittable, cur_too_far) = hittable(object_pos, center1, center2, hittable_params);

  std::tie(pred_hittable, pred_too_far) = hittable(predict_pos, center1, center2, hittable_params);

  bool too_far;
  if (!pred_hittable && pred_too_far) {
    too_far = true;
  } else {
    too_far = false;
  }

  bool too_close;
  if (!pred_hittable && !pred_too_far) {
    too_close = true;
  } else {
    too_close = false;
  }

  bool moving;
  if (object_vel.norm() > 0.1) {
    moving = true;
  } else {
    moving = false;
  }

  bool towards;
  if (object_vel.dot(d_center) < -0.05) {
    towards = true;
  }//
  else {
    towards = false;
  }

  bool ee_ready;
  if ((ee_pos - (predict_pos - ee_offset[0] * d_points / d_points.norm() + v_offset)).norm() < 0.05) {
    ee_ready = true;
  } else {
    ee_ready = false;
  }

  bool ee_hit;
  if (ee_pos.dot(d_center) > object_pos_init.dot(d_center) - 0.13) {
    ee_hit = true;
  } else {
    ee_hit = false;
  }

  //std::cout << "cur_hittable: " << cur_hittable << "\n";
  //std::cout << "too_far:      " << too_far << "\n";
  //std::cout << "too_close:    " << too_close << "\n";
  //std::cout << "towards:      " << towards << "\n";
  //std::cout << "ee_ready:     " << ee_ready << "\n";

  switch (prev_mode) {
    case 1:                                //track
      if (too_far && ETA < 3) { mode = 2; }//if object will go too far, try to stop it
      if (pred_hittable && ETA < 0.3 && ee_ready) {
        mode = 3;
      }//if object will be in feasible position and stops in 0.5s and ee is in correct position, go to hit
      if (too_close && ETA < 3) { mode = 5; }//if object will not make it into reach, give up and go to rest
      //if (!cur_hittable) {mode = 5;}
      break;

    case 2://stop
      if (!towards || pred_hittable) {
        mode = 1;
      }//if the object no longer moves towards, it has been stopped succesfully so go to track for correct ee

    case 3://hit
      if (!towards && moving) {
        mode = 4;
      }//|| ee_hit == true       //if object starts moving because it is hit, go to post hit and initialize kalman
      break;

    case 4:                                                 //post hit
      if (!cur_hittable || towards || !moving) { mode = 5; }//if object has left the range of arm, go to rest
      break;

    case 5:                                //rest
      if (too_far && ETA < 3) { mode = 2; }//if object will go too far, try to stop it
      if (pred_hittable && ETA < 0.3 && ee_ready) {
        mode = 3;
      }//same as when being in tracking mode, since mode is initialized in rest
      if (pred_hittable && ETA < 3 && !ee_ready) {
        mode = 1;
      }//if object is going to be hittable but ee is not in the right position, lets track!
      break;
  }
  return mode;
}

int AirHockey::maniModeSelektor(Eigen::Vector3d object_pos,
                                Eigen::Vector3d object_pos_init,
                                Eigen::Vector3d object_vel,
                                Eigen::Vector3d ee_pos,
                                Eigen::Vector3d center1,
                                Eigen::Vector3d center2,
                                Eigen::Vector2d ee_offset,
                                Eigen::Vector4d hittable_params,
                                const int prev_mode,
                                const int key_ctrl,
                                const int iiwa_no) {
  int mode = prev_mode;// if none of the conditions are met, mode remains the same

  Eigen::Vector3d d_center = center2 - center1;
  Eigen::Vector3d d_points = center2 - object_pos;
  Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};

  bool cur_hittable, cur_too_far;
  std::tie(cur_hittable, cur_too_far) = hittable(object_pos, center1, center2, hittable_params);

  bool moving;
  if (object_vel.norm() > 0.07) {
    moving = true;
  } else {
    moving = false;
  }

  bool towards;
  if (object_vel.dot(d_center) < -0.05) {
    towards = true;
  } else {
    towards = false;
  }

  bool ee_ready;
  if ((ee_pos - (object_pos - ee_offset[0] * d_points / d_points.norm() + v_offset)).norm() < 0.02) {
    ee_ready = true;
  } else {
    ee_ready = false;
  }

  bool ee_hit;
  if (ee_pos.dot(d_center) > object_pos_init.dot(d_center)) {
    ee_hit = true;
  } else {
    ee_hit = false;
  }

  if (iiwa_no == 1) {
    std::cout << (ee_pos - (object_pos - ee_offset[0] * d_points / d_points.norm() + v_offset)).norm() << std::endl;
  }

  switch (prev_mode) {
    case 1://track
      if (cur_hittable && ee_ready && key_ctrl == iiwa_no) { mode = 3; }
      if (!cur_hittable) { mode = 5; }
      if (key_ctrl == 3) { mode = 5; }
      break;

    case 3://hit
      if (!towards && moving || ee_hit == true) {
        mode = 4;
      }//if object starts moving because it is hit, go to post hit and initialize kalman
      if (key_ctrl == 3) { mode = 5; }
      break;

    case 4:                                      //post hit
      if (!cur_hittable || towards) { mode = 5; }//if object has left the range of arm, go to rest
      if (key_ctrl == 3) { mode = 5; }
      break;

    case 5://rest
      if (cur_hittable && ee_ready && key_ctrl == iiwa_no) {
        mode = 3;
      }//same as when being in tracking mode, since mode is initialized in rest
      if (cur_hittable && !ee_ready && key_ctrl == iiwa_no) { mode = 1; }
      break;
  }
  return mode;
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
    play_air_hockey->run();
  }
}