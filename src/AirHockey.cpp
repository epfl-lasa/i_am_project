
#include "AirHockey.hpp"

bool AirHockey::init() {

  // Check if sim or real
  if (!nh_.getParam("simulation_referential",isSim_)) { ROS_ERROR("Param simulation not found"); }

  // check if recording
  if (!nh_.getParam("recording",isRecording_)) { ROS_ERROR("Param recording not found"); }
  if (!nh_.getParam("recorder_folder", recordingFolderPath_)) { ROS_ERROR("Param recorder_path not found"); }
  if (!nh_.getParam("time_object_record", recordingTimeObject_)) { ROS_ERROR("Param recorder_path not found"); }

  // Get topics names
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
    if (!nh_.getParam("/iiwa/info_7/joint_state", iiwaJointStateTopicReal_[IIWA_7])) {ROS_ERROR("Topic /iiwa1/ee_info/vel not found");}
    if (!nh_.getParam("/iiwa/info_14/joint_state", iiwaJointStateTopicReal_[IIWA_14])) {ROS_ERROR("Topic /iiwa2/ee_info/vel not found");}
    if (!nh_.getParam("/vrpn_client_node/object_1/pose", objectPositionTopic_)) {ROS_ERROR("Topic vrpn/object1 not found");}
    if (!nh_.getParam("/vrpn_client_node/iiwa_7_base/pose", iiwaBasePositionTopic_[IIWA_7])) {ROS_ERROR("Topic vrpn/iiwa7 not found");}
    if (!nh_.getParam("/vrpn_client_node/iiwa_14_base/pose", iiwaBasePositionTopic_[IIWA_14])) {ROS_ERROR("Topic vrpn/iiwa14 not found");}
  }
  
  // Init publishers
  pubVelQuat_[IIWA_7] = nh_.advertise<geometry_msgs::Pose>(pubVelQuatTopic_[IIWA_7], 1);
  pubVelQuat_[IIWA_14] = nh_.advertise<geometry_msgs::Pose>(pubVelQuatTopic_[IIWA_14], 1);

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

    iiwaJointStateReal_[IIWA_7] = 
        nh_.subscribe<sensor_msgs::JointState>(iiwaJointStateTopicReal_[IIWA_7],
                                            1,
                                            boost::bind(&AirHockey::iiwaJointStateCallbackReal, this, _1, IIWA_7),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());    

    iiwaJointStateReal_[IIWA_14] = 
        nh_.subscribe<sensor_msgs::JointState>(iiwaJointStateTopicReal_[IIWA_14],
                                            1,
                                            boost::bind(&AirHockey::iiwaJointStateCallbackReal, this, _1, IIWA_14),
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
  previousObjectPositionFromSource_ = objectPositionFromSource_; // set prev position

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

  moved_manually_count_ = 1; // for recording object moved manually

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
}

void AirHockey::iiwaVelocityCallbackReal(const geometry_msgs::Twist::ConstPtr& msg, int k){
  iiwaVelocityFromSource_[k]  << msg->linear.x, msg->linear.y, msg->linear.z;
}

void AirHockey::iiwaBasePositionCallbackReal(const geometry_msgs::PoseStamped::ConstPtr& msg, int k){
  iiwaBasePositionFromSource_[k]  << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void AirHockey::objectPositionCallbackReal(const geometry_msgs::PoseStamped::ConstPtr& msg){
  objectPositionFromSource_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  objectOrientationFromSource_ << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w;

}

void AirHockey::iiwaJointStateCallbackReal(const sensor_msgs::JointState::ConstPtr& msg, int k){
  iiwaJointState_[k].name =  msg->name;
  iiwaJointState_[k].position =  msg->position;
  iiwaJointState_[k].velocity =  msg->velocity;
  iiwaJointState_[k].effort =  msg->effort;

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

float AirHockey::calculateDirFlux(Robot robot_name) {
  return (iiwaTaskInertiaPos_[robot_name](1, 1) / (iiwaTaskInertiaPos_[robot_name](1, 1) + objectMass_)) * iiwaVelocityFromSource_[robot_name](1);
}

// KEYBOARD INTERACTIONS
AirHockey::StatesVar AirHockey::getKeyboard(StatesVar statesvar ) {

  nonBlock(1);

  if (khBit() != 0) {
    char keyboardCommand = fgetc(stdin);
    fflush(stdin);

    switch (keyboardCommand) {
      case 'q': {
        statesvar.state_robot7_ = HIT;
        std::cout << "q is pressed " << std::endl;
        
      } break;
      case 'p': {
        statesvar.state_robot14_ = HIT;
        std::cout << "p is pressed " << std::endl;
      } break;
      case 'r': {
        statesvar.state_robot7_ = REST;
        statesvar.state_robot14_ = REST;
      } break;
      case 'h': { // toggle isHit_ 
        if(statesvar.isHit_){ statesvar.isHit_ = 0;}
        else if(!statesvar.isHit_){ statesvar.isHit_= 1;}
      } break;

    }
  }
  nonBlock(0);

  return statesvar;
}

// RECORDING FUNCTIONS 
std::string AirHockey::robotToString(Robot robot_name) {
  switch (robot_name) {
      case IIWA_7:
          return "IIWA_7";
      case IIWA_14:
          return "IIWA_14";
      default:
          return "Unknown Robot";
  }
}

void AirHockey::recordRobot(Robot robot_name){
  // record robot data during HIT phase
  // joint state, joint velocity, EEF position + velocity

  RecordedRobotState newState;
  newState.robot_name = robotToString(robot_name);
  // Get the current time
  newState.time = ros::Time::now();

  newState.joint_pos.resize(7);
  Eigen::Map<Eigen::VectorXd> tempVector1(iiwaJointState_[robot_name].position.data(), iiwaJointState_[robot_name].position.size());
  newState.joint_pos = tempVector1;

  newState.joint_vel.resize(7);
  Eigen::Map<Eigen::VectorXd> tempVector2(iiwaJointState_[robot_name].velocity.data(), iiwaJointState_[robot_name].velocity.size());
  newState.joint_vel =  tempVector2;

  newState.eef_pos.resize(3);
  newState.eef_pos = iiwaPositionFromSource_[robot_name];

  newState.eef_orientation.resize(4);
  newState.eef_orientation = iiwaOrientationFromSource_[robot_name];

  newState.eef_vel.resize(3);
  newState.eef_vel = iiwaVelocityFromSource_[robot_name];

  newState.inertia.resize(9);
  Eigen::Map<Eigen::Matrix<float, 9, 1>> tempVector3(iiwaTaskInertiaPos_[robot_name].data());
  newState.inertia = tempVector3;

  newState.hitting_flux = calculateDirFlux(robot_name);

  // Add the new state to the vector
  robotStatesVector_[robot_name].push_back(newState);

}

void AirHockey::recordObject(){
  // record object position 
  // Start when robot enters HIT phase, ends with a timer after X seconds

  RecordedObjectState newState;

  // Get the current time
  newState.time = ros::Time::now();

  // Get the current time
  newState.position = objectPositionFromSource_;

  // Add the new state to the vector
  objectStatesVector_.push_back(newState);

}

void AirHockey::recordObjectMovedByHand(int hit_count){
  // Called when both robots are at rest
  // Detects if object is moving and record if so. Writes to file when stops moving

  float stopped_threshold = 2*1e-4;
  float moving_threshold = 1e-3;
  float norm = (previousObjectPositionFromSource_-objectPositionFromSource_).norm();
  if(norm == 0){return;} // object callback has not yet been updated
  // Need to update prevPosition -> we consider object cannot manually be moved more than 10cm in 5ms
  if(norm > 0.1){norm = 0;}// hack to avoid writing file right after hit 

  if((norm < stopped_threshold) && isObjectMoving_){
    // was moving but stopped -> write to file
    std::string fn = recordingFolderPath_ + "object_moved_manually_after_hit_"+ std::to_string(hit_count)+"-"+std::to_string(moved_manually_count_)+".csv";
    writeObjectStatesToFile(hit_count);
    objectStatesVector_.clear(); // clear vector data
    moved_manually_count_ += 1;
    isObjectMoving_ = 0;
    std::cout << "Writing motion for object moved manually : " << fn << std::endl;
  }
  else if(norm < stopped_threshold){
    // not moving -> do nothing
    isObjectMoving_ = 0;
  }
  else if(norm > moving_threshold){
    // moving -> record object
    isObjectMoving_ = 1;
    recordObject();
  }

  previousObjectPositionFromSource_ = objectPositionFromSource_;
}

void AirHockey::writeRobotStatesToFile(Robot robot_name, int hit_count) {
    
    std::string filename = recordingFolderPath_ + robotToString(robot_name) +"_hit_"+ std::to_string(hit_count)+".csv";
    std::ofstream outFile(filename, std::ios::app); // Open file in append mode

    if(!outFile){
        std::cerr << "Error opening file: " << filename << std::endl;
        perror("Error");
        return;
    }

    // Write CSV header
    outFile << "RobotName,RosTime,JointPosition,JointVelocity,EEF_Position,EEF_Orientation,EEF_Velocity,Inertia,HittingFlux\n";

    // Write each RobotState structure to the file
    for (const auto& state : robotStatesVector_[robot_name]) {
        // Write CSV row
        outFile << state.robot_name << ","
                << std::setprecision(std::numeric_limits<double>::max_digits10) << state.time.toSec() + 3600 << "," // add precision and 1h for GMT
                << state.joint_pos.transpose() << ","
                << state.joint_vel.transpose() << ","
                << state.eef_pos.transpose() << ","
                << state.eef_orientation.transpose() << ","
                << state.eef_vel.transpose() << ","
                << state.inertia.transpose() << ","
                << state.hitting_flux << "\n";
    }

    outFile.close();

    robotStatesVector_[robot_name].clear(); // clear vector data

    std::cout << "Finished writing hit " << hit_count << " for "<< robotToString(robot_name) << std::endl;
    
}

void AirHockey::writeObjectStatesToFile(int hit_count) {

  std::string filename = recordingFolderPath_ + "object_hit_"+ std::to_string(hit_count)+".csv";

  std::ofstream outFile(filename, std::ios::app); // Open file in append mode

  if(!outFile){
      std::cerr << "Error opening file: " << filename << std::endl;
      return;
  }

  // Write CSV header
  outFile << "RosTime,Position\n";

  // Write each RobotState structure to the file
  for (const auto& state : objectStatesVector_) {
      // outFile << "Object Name: " << state.robot_name << "\n";
      outFile << std::setprecision(std::numeric_limits<double>::max_digits10) << state.time.toSec()+3600 << "," // add precision and 1h for GMT
              << state.position.transpose() << "\n";
  }

  outFile.close();

  objectStatesVector_.clear(); // clear vector data
  std::cout << "Finished writing hit " << hit_count << " for object!" << std::endl;
}

void AirHockey::copyYamlFile(std::string inFilePath, std::string outFilePath){
  // Open the original YAML file for reading
  std::ifstream inFile(inFilePath);

  if (!inFile.is_open()) {
      std::cerr << "Error opening original YAML file for reading\n";
      perror("Error");
      return;
  }

  // Open a new file for writing (copying)
  std::ofstream outFile(outFilePath);

  if (!outFile.is_open()) {
      std::cerr << "Error opening new YAML file for writing\n";
      perror("Error");
      return;
  }

  // Read and copy each line from the original file to the new file
  std::string line;
  while (std::getline(inFile, line)) {
      outFile << line << "\n";
  }

  // Close the files
  inFile.close();
  outFile.close();
}

void AirHockey::setUpRecordingDir(){
  // Set up data structure here
  // data/airhockey/TimeStampedDir/

  // create airhockey dir
  if (mkdir(recordingFolderPath_.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
      std::cout << "Directory created successfully: " << recordingFolderPath_ << std::endl;
  } else {
      std::cerr << "Error creating directory." << std::endl;
      perror("Error");
  }

  // giving writing authorization to recording directory -- NOT needed 
  // if (chmod(recordingFolderPath_.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
  //     std::cout << "Write permissions set successfully for the directory." << std::endl;
  // } else {
  //     std::cerr << "Error setting write permissions for the directory." << std::endl;
  //     perror("Error");
  //     return;
  // }

  // Create Time Stamped Directory
  // Get the current time
  auto now = std::chrono::system_clock::now();

  // Convert the current time to a string
  std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
  currentTime += 3600; // add 1h for GMT correct time (due to docker not having correct timezone)
  std::tm* timeinfo = std::localtime(&currentTime);
  std::stringstream ss;
  ss << std::put_time(timeinfo, "%Y-%m-%d_%H:%M:%S");
  std::string timestamp = ss.str();

  // Create the directory
  std::string directoryPath = recordingFolderPath_ + timestamp +"/";
  if (mkdir(directoryPath.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
      std::cout << "Directory created successfully: " << directoryPath << std::endl;
      recordingFolderPath_ = directoryPath; // Update recording path
  } else {
      std::cerr << "Error creating directory." << std::endl;
      perror("Error");
      directoryPath.clear(); // Return an empty string on error
  }

  // Copy hit_properties_air_hockey.yaml to data file to save params of recorded data 
  std::string hit_params_to_copy = "/home/ros/ros_ws/src/i_am_project/config/hit_properties_air_hockey.yaml";
  std::string hitting_params_fn = recordingFolderPath_ + "hitting_params.yaml";
  std::string toolkit_params_to_copy = "/home/ros/ros_ws/src/iiwa_toolkit_ns/config/passive_track_params_dual_real.yaml";
  std::string toolkit_params_fn = recordingFolderPath_ + "toolkit_params.yaml";

  copyYamlFile(hit_params_to_copy, hitting_params_fn);
  copyYamlFile(toolkit_params_to_copy, toolkit_params_fn);
}


void AirHockey::run() {

  // Set up recording directory structure
  if(isRecording_){setUpRecordingDir();}

  // Set up counters and bool variables
  int print_count = 0;
  int hit_count = 1;
  bool write_once_7 = 0;
  bool write_once_14 = 0;
  bool write_once_object = 0;

  ros::Time hit_time = ros::Time::now();
  ros::Duration max_recording_time = ros::Duration(recordingTimeObject_);

  std::cout << "READY TO RUN " << std::endl;

  while (ros::ok()) {

    statesvar = getKeyboard(statesvar);

    // DEBUG
    if(print_count%200 == 0 ){
      // std::cout << "ishit : " << statesvar.isHit_ << std::endl;
      // std::cout << "iiwa7_state : " << statesvar.state_robot7_ << " \n iiwa14_state : " << statesvar.state_robot14_<< std::endl;
      // std::cout << "iiwa7_vel : " << iiwaVel_[IIWA_7] << std::endl;
      // std::cout << "iiwaPos_ 7  " << iiwaPositionFromSource_[IIWA_7]<< std::endl;
      // std::cout << "returnPos_ 7  " << returnPos_[IIWA_7]<< std::endl;
    }
    print_count +=1 ;

    // Update DS attractor if at rest + record if object is moved by hand
    if(statesvar.state_robot7_ == REST && statesvar.state_robot14_ == REST){
      // update only at REST so object state conditions for isHit_ works 
      updateDSAttractor();
    }

    // UPDATE robot state
    if(statesvar.state_robot7_ == HIT){
      refVelocity_[IIWA_7] = generateHitting7_->flux_DS(hittingFlux_[IIWA_7], iiwaTaskInertiaPos_[IIWA_7]);
      
      if(isRecording_){
        recordRobot(IIWA_7);
        recordObject();
        write_once_7 = 1;
        write_once_object =1;
      }
    }

    if(statesvar.state_robot14_ == HIT){
      refVelocity_[IIWA_14] = generateHitting14_->flux_DS(hittingFlux_[IIWA_7], iiwaTaskInertiaPos_[IIWA_14]);
      
      if(isRecording_){
        recordRobot(IIWA_14);
        recordObject();
        write_once_14 = 1;
        write_once_object =1;
      }
    }

    if(statesvar.state_robot7_ == REST || statesvar.isHit_ == 1){
      refVelocity_[IIWA_7] = generateHitting7_->linear_DS(returnPos_[IIWA_7]);
      statesvar.state_robot7_ = REST;
      if (statesvar.state_robot14_ == REST) { // only reset if 14 is at rest, otherwise it skips next if and never send iiwa14 to rest
              statesvar.isHit_ = 0;
      }
    }

    if(statesvar.state_robot14_ == REST || statesvar.isHit_ == 1){
      refVelocity_[IIWA_14] = generateHitting14_->linear_DS(returnPos_[IIWA_14]);
      statesvar.state_robot14_ = REST;
      statesvar.isHit_ = 0;
    }

    // Update isHit 
    if (!statesvar.isHit_ && generateHitting7_->get_des_direction().dot(generateHitting7_->get_DS_attractor()
                                                      - generateHitting7_->get_current_position()) < 0) {
      statesvar.isHit_ = 1;
      hit_time = ros::Time::now();
    }

    if (!statesvar.isHit_ && generateHitting14_->get_des_direction().dot(generateHitting14_->get_DS_attractor()
                                                      - generateHitting14_->get_current_position())  < 0) {
      statesvar.isHit_ = 1;
      hit_time = ros::Time::now();
    }

    if(isRecording_){
      // Keep recording object for X seconds after hit (only when robots are at rest to avoid overlap)
      auto time_since_hit = ros::Time::now() - hit_time;
      if(statesvar.state_robot7_ == REST && statesvar.state_robot14_ == REST && time_since_hit < max_recording_time){
        recordObject();
      }
      else if(statesvar.state_robot7_ == REST && statesvar.state_robot14_ == REST && 
              time_since_hit > max_recording_time && write_once_object){        
       
        writeObjectStatesToFile(hit_count);
        write_once_object = 0;
        hit_count += 1;
        moved_manually_count_ = 1; // reset count for moved manually
      }
      else if(statesvar.state_robot7_ == REST && statesvar.state_robot14_ == REST && 
              time_since_hit > max_recording_time && !write_once_object){
        // If we are done writing hit for object, check if it is moved by hand and record if so
        recordObjectMovedByHand(hit_count-1);
      }

      // Writing data logic
      if(statesvar.state_robot7_ == REST && write_once_7){
        writeRobotStatesToFile(IIWA_7, hit_count);
        write_once_7 = 0;
      }

      if(statesvar.state_robot14_ == REST && write_once_14){
        writeRobotStatesToFile(IIWA_14, hit_count);
        write_once_14 = 0;
      }
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
