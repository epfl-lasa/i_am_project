#include "AirHockey.hpp"

bool AirHockey::init() {

  // Check if sim or real
  if (!nh_.getParam("simulation_referential",isSim_)) { ROS_ERROR("Param simulation not found"); }

  // Get topics names
  if (!nh_.getParam("recorder_topic", pubFSMTopic_)) {ROS_ERROR("Topic /recorder/robot_states not found");}

  if (!nh_.getParam("/passive_control/vel_quat_7", pubVelQuatTopic_[IIWA_7])) {ROS_ERROR("Topic /passive_control iiwa 7 not found");}
  if (!nh_.getParam("/passive_control/vel_quat_14", pubVelQuatTopic_[IIWA_14])) {ROS_ERROR("Topic /passive_control iiwa 14 not found");}

  if (!nh_.getParam("/iiwa/inertia/taskPos_7", iiwaInertiaTopic_[IIWA_7])) {ROS_ERROR("Topic /iiwa/inertia/taskPos not found");}
  if (!nh_.getParam("/iiwa/inertia/taskPos_14", iiwaInertiaTopic_[IIWA_14])) {ROS_ERROR("Topic /iiwa/inertia/taskPos not found");}

  if(isSim_){
    if (!nh_.getParam("/gazebo/link_states", iiwaPositionTopicSim_)) {ROS_ERROR("Topic /gazebo/link_states not found");}
    if (!nh_.getParam("/gazebo/model_states", objectPositionTopic_)) {ROS_ERROR("Topic /gazebo/model_states not found");}
  }

  else if (!isSim_){
    if (!nh_.getParam("/iiwa/info_7/pose", iiwaPositionTopicReal_[IIWA_7])) {ROS_ERROR("Topic /iiwa1/ee_info/pose not found");}
    if (!nh_.getParam("/iiwa/info_14/pose", iiwaPositionTopicReal_[IIWA_14])) {ROS_ERROR("Topic /iiwa2/ee_info/pose not found");}
    if (!nh_.getParam("/iiwa/info_7/vel", iiwaVelocityTopicReal_[IIWA_7])) {ROS_ERROR("Topic /iiwa1/ee_info/vel not found");}
    if (!nh_.getParam("/iiwa/info_14/vel", iiwaVelocityTopicReal_[IIWA_14])) {ROS_ERROR("Topic /iiwa2/ee_info/vel not found");}
    if (!nh_.getParam("/vrpn_client_node/object_1/pose", objectPositionTopic_)) {ROS_ERROR("Topic vrpn/object1 not found");}
    if (!nh_.getParam("/vrpn_client_node/iiwa_7_base/pose", iiwaBasePositionTopic_[IIWA_7])) {ROS_ERROR("Topic vrpn/iiwa7 not found");}
    if (!nh_.getParam("/vrpn_client_node/iiwa_14_base/pose", iiwaBasePositionTopic_[IIWA_14])) {ROS_ERROR("Topic vrpn/iiwa14 not found");}
  }
  
  // Init publishers
  pubVelQuat_[IIWA_7] = nh_.advertise<geometry_msgs::Pose>(pubVelQuatTopic_[IIWA_7], 1);
  pubVelQuat_[IIWA_14] = nh_.advertise<geometry_msgs::Pose>(pubVelQuatTopic_[IIWA_14], 1);
  pubFSM_ = nh_.advertise<i_am_project::FSM_state>(pubFSMTopic_, 1);

  // Init subscribers
  if(isSim_){
    objectPosition_ = nh_.subscribe(objectPositionTopic_,
                                  1,
                                  &AirHockey::objectPositionCallbackGazebo,
                                  this,
                                  ros::TransportHints().reliable().tcpNoDelay());
    iiwaPosition_ = nh_.subscribe(iiwaPositionTopicSim_,
                                1,
                                &AirHockey::iiwaPositionCallbackGazebo,
                                this,
                                ros::TransportHints().reliable().tcpNoDelay());

  }
  else if (!isSim_){
    objectPosition_ = nh_.subscribe(objectPositionTopic_,
                                   1,
                                   &AirHockey::objectPositionCallbackReal,
                                   this,
                                   ros::TransportHints().reliable().tcpNoDelay());

    iiwaPositionReal_[IIWA_7] = 
        nh_.subscribe<geometry_msgs::Pose>(iiwaPositionTopicReal_[IIWA_7],
                                            1,
                                            boost::bind(&AirHockey::iiwaPoseCallbackReal, this, _1, IIWA_7),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());    

    iiwaPositionReal_[IIWA_14] = 
        nh_.subscribe<geometry_msgs::Pose>(iiwaPositionTopicReal_[IIWA_14],
                                            1,
                                            boost::bind(&AirHockey::iiwaPoseCallbackReal, this, _1, IIWA_14),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());

    iiwaVelocityReal_[IIWA_7] = 
        nh_.subscribe<geometry_msgs::Twist>(iiwaVelocityTopicReal_[IIWA_7],
                                            1,
                                            boost::bind(&AirHockey::iiwaVelocityCallbackReal, this, _1, IIWA_7),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());    

    iiwaVelocityReal_[IIWA_14] = 
        nh_.subscribe<geometry_msgs::Twist>(iiwaVelocityTopicReal_[IIWA_14],
                                            1,
                                            boost::bind(&AirHockey::iiwaVelocityCallbackReal, this, _1, IIWA_14),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());

    iiwaBasePosition_[IIWA_7] = 
        nh_.subscribe<geometry_msgs::PoseStamped>(iiwaBasePositionTopic_[IIWA_7],
                                                  1,
                                                  boost::bind(&AirHockey::iiwaBasePositionCallbackReal, this, _1, IIWA_7),
                                                  ros::VoidPtr(),
                                                  ros::TransportHints().reliable().tcpNoDelay());
    
    iiwaBasePosition_[IIWA_14] = 
        nh_.subscribe<geometry_msgs::PoseStamped>(iiwaBasePositionTopic_[IIWA_14],
                                                  1,
                                                  boost::bind(&AirHockey::iiwaBasePositionCallbackReal, this, _1, IIWA_14),
                                                  ros::VoidPtr(),
                                                  ros::TransportHints().reliable().tcpNoDelay());
  }
  
  iiwaInertia_[IIWA_7] =
      nh_.subscribe<geometry_msgs::Inertia>(iiwaInertiaTopic_[IIWA_7],
                                            1,
                                            boost::bind(&AirHockey::iiwaInertiaCallback, this, _1, IIWA_7),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());
  iiwaInertia_[IIWA_14] =
      nh_.subscribe<geometry_msgs::Inertia>(iiwaInertiaTopic_[IIWA_14],
                                            1,
                                            boost::bind(&AirHockey::iiwaInertiaCallback, this, _1, IIWA_14),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());


  // get object offset values
  if (!nh_.getParam("iiwa7/object_offset/x", objectOffset_[IIWA_7][0])) { ROS_ERROR("Topic iiwa7/object_offset/x not found"); }
  if (!nh_.getParam("iiwa7/object_offset/y", objectOffset_[IIWA_7][1])) { ROS_ERROR("Topic iiwa7/object_offset/y not found"); }
  if (!nh_.getParam("iiwa7/object_offset/z", objectOffset_[IIWA_7][2])) { ROS_ERROR("Topic iiwa7/object_offset/z not found"); }
  if (!nh_.getParam("iiwa14/object_offset/x", objectOffset_[IIWA_14][0])) { ROS_ERROR("Topic iiwa14/object_offset/x not found"); }
  if (!nh_.getParam("iiwa14/object_offset/y", objectOffset_[IIWA_14][1])) { ROS_ERROR("Topic iiwa14/object_offset/y not found"); }
  if (!nh_.getParam("iiwa14/object_offset/z", objectOffset_[IIWA_14][2])) { ROS_ERROR("Topic iiwa14/object_offset/x not found"); }

  while (objectPositionFromSource_.norm() == 0) {
    this->updateDSAttractor();
    ros::spinOnce();
    rate_.sleep();
  }

  generateHitting7_->set_current_position(iiwaPositionFromSource_[IIWA_7]);
  generateHitting14_->set_current_position(iiwaPositionFromSource_[IIWA_14]);
  this->updateDSAttractor(); //update attractor position

  // Get hitting parameters
  if (!nh_.getParam("iiwa7/ref_velocity/y", refVelocity_[IIWA_7][1])) { ROS_ERROR("Param ref_velocity/y not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/y", refVelocity_[IIWA_14][1])) { ROS_ERROR("Param ref_velocity/y not found"); }
  if (!nh_.getParam("iiwa7/ref_velocity/x", refVelocity_[IIWA_7][0])) { ROS_ERROR("Param ref_velocity/x not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/x", refVelocity_[IIWA_14][0])) { ROS_ERROR("Param ref_velocity/x not found"); }
  if (!nh_.getParam("iiwa7/ref_velocity/z", refVelocity_[IIWA_7][2])) { ROS_ERROR("Param ref_velocity/z not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/z", refVelocity_[IIWA_14][2])) { ROS_ERROR("Param ref_velocity/z not found"); }
  if (!nh_.getParam("iiwa7/return_position/x", returnPos_[IIWA_7][0])) { ROS_ERROR("Param ref_quat/x not found"); }
  if (!nh_.getParam("iiwa14/return_position/x", returnPos_[IIWA_14][0])) { ROS_ERROR("Param return_position/x not found"); }
  if (!nh_.getParam("iiwa7/return_position/y", returnPos_[IIWA_7][1])) { ROS_ERROR("Param return_position/y not found"); }
  if (!nh_.getParam("iiwa14/return_position/y", returnPos_[IIWA_14][1])) { ROS_ERROR("Param return_position/y not found"); }
  if (!nh_.getParam("iiwa7/return_position/z", returnPos_[IIWA_7][2])) { ROS_ERROR("Param return_position/z not found"); }
  if (!nh_.getParam("iiwa14/return_position/z", returnPos_[IIWA_14][2])) { ROS_ERROR("Param return_position/z not found"); }
  if (!nh_.getParam("iiwa7/hit_direction/x", hitDirection_[IIWA_7][0])) {ROS_ERROR("Param hit_direction/x not found");}
  if (!nh_.getParam("iiwa14/hit_direction/x", hitDirection_[IIWA_14][0])) {ROS_ERROR("Param hit_direction/x not found");}
  if (!nh_.getParam("iiwa7/hit_direction/y", hitDirection_[IIWA_7][1])) {ROS_ERROR("Param hit_direction/y not found");}
  if (!nh_.getParam("iiwa14/hit_direction/y", hitDirection_[IIWA_14][1])) {ROS_ERROR("Param hit_direction/y not found");}
  if (!nh_.getParam("iiwa7/hit_direction/z", hitDirection_[IIWA_7][2])) {ROS_ERROR("Param hit_direction/z not found");}
  if (!nh_.getParam("iiwa14/hit_direction/z", hitDirection_[IIWA_14][2])) {ROS_ERROR("Param hit_direction/z not found");}
  if (!nh_.getParam("iiwa7/hitting_flux", hittingFlux_[IIWA_7])) {ROS_ERROR("Param iiwa7/hitting_flux not found");}
  if (!nh_.getParam("iiwa14/hitting_flux", hittingFlux_[IIWA_14])) {ROS_ERROR("Param iiwa14/hitting_flux not found");}
  if (!nh_.getParam("object_mass", objectMass_)) {ROS_ERROR("Param object_mass not found");}

  generateHitting7_->set_des_direction(hitDirection_[IIWA_7]);
  generateHitting14_->set_des_direction(hitDirection_[IIWA_14]);

  // add gazebo offset (from position argument in gazebo node in launch file)
  if(isSim_){
    returnPos_[IIWA_7][1] -= 0.6;
    returnPos_[IIWA_14][1] += 0.6;
  }

  return true;
}

// CALLBACKS
void AirHockey::objectPositionCallbackGazebo(const gazebo_msgs::ModelStates& modelStates) {
  int boxIndex = getIndex(modelStates.name, "box_model");
  boxPose_ = modelStates.pose[boxIndex];
  objectPositionFromSource_ << boxPose_.position.x, boxPose_.position.y, boxPose_.position.z;
  objectOrientationFromSource_ << boxPose_.orientation.x, boxPose_.orientation.y, boxPose_.orientation.z,
      boxPose_.orientation.w;
}

void AirHockey::iiwaPositionCallbackGazebo(const gazebo_msgs::LinkStates& linkStates) {
  int indexIiwa1 = getIndex(linkStates.name, "iiwa1::iiwa1_link_7");// End effector is the 7th link in KUKA IIWA
  int indexIiwa2 = getIndex(linkStates.name, "iiwa2::iiwa2_link_7");// End effector is the 7th link in KUKA IIWA

  iiwaPose_[IIWA_7] = linkStates.pose[indexIiwa1];
  iiwaPositionFromSource_[IIWA_7] << iiwaPose_[IIWA_7].position.x, iiwaPose_[IIWA_7].position.y,
      iiwaPose_[IIWA_7].position.z;
  iiwaVel_[IIWA_7] = linkStates.twist[indexIiwa1];

  iiwaPose_[IIWA_14] = linkStates.pose[indexIiwa2];
  iiwaPositionFromSource_[IIWA_14] << iiwaPose_[IIWA_14].position.x, iiwaPose_[IIWA_14].position.y,
      iiwaPose_[IIWA_14].position.z;
  iiwaVel_[IIWA_14] = linkStates.twist[indexIiwa2];
}

void AirHockey::iiwaInertiaCallback(const geometry_msgs::Inertia::ConstPtr& msg, int k) {
  iiwaTaskInertiaPos_[k](0, 0) = msg->ixx;
  iiwaTaskInertiaPos_[k](2, 2) = msg->izz;
  iiwaTaskInertiaPos_[k](1, 1) = msg->iyy;
  iiwaTaskInertiaPos_[k](0, 1) = msg->ixy;
  iiwaTaskInertiaPos_[k](1, 0) = msg->ixy;
  iiwaTaskInertiaPos_[k](0, 2) = msg->ixz;
  iiwaTaskInertiaPos_[k](2, 0) = msg->ixz;
  iiwaTaskInertiaPos_[k](1, 2) = msg->iyz;
  iiwaTaskInertiaPos_[k](2, 1) = msg->iyz;
}

void AirHockey::iiwaPoseCallbackReal(const geometry_msgs::Pose::ConstPtr& msg, int k){
  iiwaPositionFromSource_[k]  << msg->position.x, msg->position.y, msg->position.z;
  iiwaOrientationFromSource_[k] << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
  iiwaPose_[k] = *msg;
}

void AirHockey::iiwaVelocityCallbackReal(const geometry_msgs::Twist::ConstPtr& msg, int k){
  iiwaVelocityFromSource_[k]  << msg->linear.x, msg->linear.y, msg->linear.z;
  iiwaVel_[k] = *msg;
}

void AirHockey::iiwaBasePositionCallbackReal(const geometry_msgs::PoseStamped::ConstPtr& msg, int k){
  iiwaBasePositionFromSource_[k]  << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void AirHockey::objectPositionCallbackReal(const geometry_msgs::PoseStamped::ConstPtr& msg){
  objectPositionFromSource_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  objectOrientationFromSource_ << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w;
}

// UPDATES AND CALCULATIONS
void AirHockey::objectPositionIiwaFrames() {
  rotationMat_ << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  objectPositionForIiwa_[IIWA_7] = rotationMat_ * (objectPositionFromSource_ - iiwaBasePositionFromSource_[IIWA_7]);
  objectPositionForIiwa_[IIWA_14] = rotationMat_ * (objectPositionFromSource_ - iiwaBasePositionFromSource_[IIWA_14]);
}

void AirHockey::updateDSAttractor() {

  if(isSim_){
    generateHitting7_->set_DS_attractor(objectPositionFromSource_);
    generateHitting14_->set_DS_attractor(objectPositionFromSource_);
  }
  else if(!isSim_){
    objectPositionIiwaFrames();

    generateHitting7_->set_DS_attractor(objectPositionForIiwa_[IIWA_7]+ objectOffset_[IIWA_7]);
    generateHitting14_->set_DS_attractor(objectPositionForIiwa_[IIWA_14]+ objectOffset_[IIWA_14]);
  }
}

int AirHockey::getIndex(std::vector<std::string> v, std::string value) {
  for (int i = 0; i < v.size(); i++) {
    if (v[i].compare(value) == 0) return i;
  }
  return -1;
}

void AirHockey::updateCurrentEEPosition(Eigen::Vector3f new_position[]) {
  generateHitting7_->set_current_position(new_position[IIWA_7]);
  generateHitting14_->set_current_position(new_position[IIWA_14]);
}

void AirHockey::publishVelQuat(Eigen::Vector3f DS_vel[], Eigen::Vector4f DS_quat[]) {
  geometry_msgs::Pose ref_vel_publish_7, ref_vel_publish_14;
  ref_vel_publish_7.position.x = DS_vel[IIWA_7](0);
  ref_vel_publish_7.position.y = DS_vel[IIWA_7](1);
  ref_vel_publish_7.position.z = DS_vel[IIWA_7](2);
  ref_vel_publish_7.orientation.x = DS_quat[IIWA_7](0);
  ref_vel_publish_7.orientation.y = DS_quat[IIWA_7](1);
  ref_vel_publish_7.orientation.z = DS_quat[IIWA_7](2);
  ref_vel_publish_7.orientation.w = DS_quat[IIWA_7](3);
  pubVelQuat_[IIWA_7].publish(ref_vel_publish_7);

  ref_vel_publish_14.position.x = DS_vel[IIWA_14](0);
  ref_vel_publish_14.position.y = DS_vel[IIWA_14](1);
  ref_vel_publish_14.position.z = DS_vel[IIWA_14](2);
  ref_vel_publish_14.orientation.x = DS_quat[IIWA_14](0);
  ref_vel_publish_14.orientation.y = DS_quat[IIWA_14](1);
  ref_vel_publish_14.orientation.z = DS_quat[IIWA_14](2);
  ref_vel_publish_14.orientation.w = DS_quat[IIWA_14](3);
  pubVelQuat_[IIWA_14].publish(ref_vel_publish_14);
}

void AirHockey::publishFSM(FSMState current_state){
  i_am_project::FSM_state msg;
  msg.mode_iiwa7 = static_cast<uint8_t>(current_state.mode_iiwa7);
  msg.mode_iiwa14 = static_cast<uint8_t>(current_state.mode_iiwa14);
  msg.isHit = current_state.isHit;
  msg.hit_time = current_state.hit_time;

  pubFSM_.publish(msg);
}

float AirHockey::calculateDirFlux(Robot robot_name) {
  return (iiwaTaskInertiaPos_[robot_name](1, 1) / (iiwaTaskInertiaPos_[robot_name](1, 1) + objectMass_)) * iiwaVelocityFromSource_[robot_name](1);
}

// KEYBOARD INTERACTIONS
AirHockey::FSMState AirHockey::getKeyboard(FSMState current_state ) {

  nonBlock(1);

  if (khBit() != 0) {
    char keyboardCommand = fgetc(stdin);
    fflush(stdin);

    switch (keyboardCommand) {
      case 'q': {
        current_state.mode_iiwa7 = HIT;
        std::cout << "q is pressed " << std::endl;
        
      } break;
      case 'p': {
        current_state.mode_iiwa14 = HIT;
        std::cout << "p is pressed " << std::endl;
      } break;
      case 'r': {
        current_state.mode_iiwa7 = REST;
        current_state.mode_iiwa14 = REST;
      } break;
      case 'h': { // toggle isHit 
        if(current_state.isHit){ current_state.isHit = 0;}
        else if(!current_state.isHit){ current_state.isHit= 1;}
      } break;

    }
  }
  nonBlock(0);

  return current_state;
}

void AirHockey::run() {

  // Set up counters and bool variables
  int print_count = 0;
  
  FSMState fsm_state;

  std::cout << "READY TO RUN " << std::endl;

  while (ros::ok()) {

    fsm_state = getKeyboard(fsm_state);

    // Publish for Recorder
    publishFSM(fsm_state);

    // DEBUG
    if(print_count%200 == 0 ){
      // std::cout << "ishit : " << fsm_state.isHit << std::endl;
      // std::cout << "iiwa7_state : " << fsm_state.mode_iiwa7 << " \n iiwa14_state : " << fsm_state.mode_iiwa14<< std::endl;
      // std::cout << "iiwa7_vel : " << iiwaVel_[IIWA_7] << std::endl;
      // std::cout << "iiwaPos_ 7  " << iiwaPositionFromSource_[IIWA_7]<< std::endl;
      // std::cout << "returnPos_ 7  " << returnPos_[IIWA_7]<< std::endl;
    }
    print_count +=1 ;

    // Update DS attractor if at rest + record if object is moved by hand
    if(fsm_state.mode_iiwa7 == REST && fsm_state.mode_iiwa14 == REST){
      // update only at REST so object state conditions for isHit works 
      updateDSAttractor();
    }

    // UPDATE robot state
    if(fsm_state.mode_iiwa7 == HIT){
      refVelocity_[IIWA_7] = generateHitting7_->flux_DS(hittingFlux_[IIWA_7], iiwaTaskInertiaPos_[IIWA_7]);
    }

    if(fsm_state.mode_iiwa14 == HIT){
      refVelocity_[IIWA_14] = generateHitting14_->flux_DS(hittingFlux_[IIWA_7], iiwaTaskInertiaPos_[IIWA_14]);
    }

    if(fsm_state.mode_iiwa7 == REST || fsm_state.isHit == 1){
      refVelocity_[IIWA_7] = generateHitting7_->linear_DS(returnPos_[IIWA_7]);
      fsm_state.mode_iiwa7 = REST;
      if (fsm_state.mode_iiwa14 == REST) { // only reset if 14 is at rest, otherwise it skips next if and never send iiwa14 to rest
              fsm_state.isHit = 0;
      }
    }

    if(fsm_state.mode_iiwa14 == REST || fsm_state.isHit == 1){
      refVelocity_[IIWA_14] = generateHitting14_->linear_DS(returnPos_[IIWA_14]);
      fsm_state.mode_iiwa14 = REST;
      fsm_state.isHit = 0;
    }

    // Update isHit 
    if (!fsm_state.isHit && generateHitting7_->get_des_direction().dot(generateHitting7_->get_DS_attractor()
                                                      - generateHitting7_->get_current_position()) < 0) {
      fsm_state.isHit = 1;
      fsm_state.hit_time = ros::Time::now();
    }

    if (!fsm_state.isHit && generateHitting14_->get_des_direction().dot(generateHitting14_->get_DS_attractor()
                                                      - generateHitting14_->get_current_position())  < 0) {
      fsm_state.isHit = 1;
      fsm_state.hit_time = ros::Time::now();
    }

    updateCurrentEEPosition(iiwaPositionFromSource_);
    publishVelQuat(refVelocity_, refQuat_);

    ros::spinOnce();
    rate_.sleep();
  }

  publishVelQuat(refVelocity_, refQuat_);
  ros::spinOnce();
  rate_.sleep();
  ros::shutdown();
}

int main(int argc, char** argv) {

  //ROS Initialization
  ros::init(argc, argv, "air_hockey");
  ros::NodeHandle nh;
  float frequency = 200.0f;

  std::unique_ptr<AirHockey> generate_motion = std::make_unique<AirHockey>(nh, frequency);

  if (!generate_motion->init()) {
    return -1;
  } else {
    std::cout << "OK "<< std::endl;;
    generate_motion->run();
  }

  return 0;
}
