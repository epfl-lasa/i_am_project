
#include "AirHockey.hpp"

bool AirHockey::init() {

  // Check if sim or real
  if (!nh_.getParam("simulation_referential",isSim_)) { ROS_ERROR("Param simulation not found"); }

  // check if recording
  if (!nh_.getParam("recording",isRecording_)) { ROS_ERROR("Param recording not found"); }
  if (!nh_.getParam("recorder_path", recordingFolderPath_)) { ROS_ERROR("Param recorder_path not found"); }


  // Get topics names
  if (!nh_.getParam("/passive_control/vel_quat_7", pubVelQuatTopic_[IIWA_7])) {ROS_ERROR("Topic /passive_control iiwa 7 not found");}
  if (!nh_.getParam("/passive_control/vel_quat_14", pubVelQuatTopic_[IIWA_14])) {ROS_ERROR("Topic /passive_control iiwa 14 not found");}

  if (!nh_.getParam("/iiwa/inertia/taskPos_7", iiwaInertiaTopic_[IIWA_7])) {ROS_ERROR("Topic /iiwa/inertia/taskPos not found");}
  if (!nh_.getParam("/iiwa/inertia/taskPos_14", iiwaInertiaTopic_[IIWA_14])) {ROS_ERROR("Topic /iiwa/inertia/taskPos not found");}

  if( isSim_){
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
                                            boost::bind(&AirHockey::iiwaInertiaCallbackGazebo, this, _1, IIWA_7),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());
  iiwaInertia_[IIWA_14] =
      nh_.subscribe<geometry_msgs::Inertia>(iiwaInertiaTopic_[IIWA_14],
                                            1,
                                            boost::bind(&AirHockey::iiwaInertiaCallbackGazebo, this, _1, IIWA_14),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());

  while (objectPositionFromSource_.norm() == 0) {
    this->updateCurrentObjectPosition();
    ros::spinOnce();
    rate_.sleep();
  }

  generateHitting7_->set_current_position(iiwaPositionFromSource_[IIWA_7]);
  generateHitting14_->set_current_position(iiwaPositionFromSource_[IIWA_14]);
  this->updateCurrentObjectPosition(); //update attracotr position

 
  if (!nh_.getParam("iiwa7/ref_velocity/y", refVelocity_[IIWA_7][1])) { ROS_ERROR("Topic ref_velocity/y not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/y", refVelocity_[IIWA_14][1])) { ROS_ERROR("Topic ref_velocity/y not found"); }
  if (!nh_.getParam("iiwa7/ref_velocity/x", refVelocity_[IIWA_7][0])) { ROS_ERROR("Topic ref_velocity/x not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/x", refVelocity_[IIWA_14][0])) { ROS_ERROR("Topic ref_velocity/x not found"); }
  if (!nh_.getParam("iiwa7/ref_velocity/z", refVelocity_[IIWA_7][2])) { ROS_ERROR("Topic ref_velocity/z not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/z", refVelocity_[IIWA_14][2])) { ROS_ERROR("Topic ref_velocity/z not found"); }
  if (!nh_.getParam("iiwa7/return_position/x", returnPos_[IIWA_7][0])) { ROS_ERROR("Topic ref_quat/x not found"); }
  if (!nh_.getParam("iiwa14/return_position/x", returnPos_[IIWA_14][0])) { ROS_ERROR("Topic return_position/x not found"); }
  if (!nh_.getParam("iiwa7/return_position/y", returnPos_[IIWA_7][1])) { ROS_ERROR("Topic return_position/y not found"); }
  if (!nh_.getParam("iiwa14/return_position/y", returnPos_[IIWA_14][1])) { ROS_ERROR("Topic return_position/y not found"); }
  if (!nh_.getParam("iiwa7/return_position/z", returnPos_[IIWA_7][2])) { ROS_ERROR("Topic return_position/z not found"); }
  if (!nh_.getParam("iiwa14/return_position/z", returnPos_[IIWA_14][2])) { ROS_ERROR("Topic return_position/z not found"); }
  if (!nh_.getParam("iiwa7/hit_direction/x", hitDirection_[IIWA_7][0])) {ROS_ERROR("Topic hit_direction/x not found");}
  if (!nh_.getParam("iiwa14/hit_direction/x", hitDirection_[IIWA_14][0])) {ROS_ERROR("Topic hit_direction/x not found");}
  if (!nh_.getParam("iiwa7/hit_direction/y", hitDirection_[IIWA_7][1])) {ROS_ERROR("Topic hit_direction/y not found");}
  if (!nh_.getParam("iiwa14/hit_direction/y", hitDirection_[IIWA_14][1])) {ROS_ERROR("Topic hit_direction/y not found");}
  if (!nh_.getParam("iiwa7/hit_direction/z", hitDirection_[IIWA_7][2])) {ROS_ERROR("Topic hit_direction/z not found");}
  if (!nh_.getParam("iiwa14/hit_direction/z", hitDirection_[IIWA_14][2])) {ROS_ERROR("Topic hit_direction/z not found");}

  generateHitting7_->set_des_direction(hitDirection_[IIWA_7]);
  generateHitting14_->set_des_direction(hitDirection_[IIWA_14]);

  std::cout << "finish initializing !!!!"<< std::endl;

  return true;
}

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

void AirHockey::iiwaInertiaCallbackGazebo(const geometry_msgs::Inertia::ConstPtr& msg, int k) {
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

void AirHockey::objectPositionIiwaFrames() {
  rotationMat_ << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  objectPositionForIiwa_[IIWA_7] = rotationMat_ * (objectPositionFromSource_ - iiwaBasePositionFromSource_[IIWA_7]);
  objectPositionForIiwa_[IIWA_14] = rotationMat_ * (objectPositionFromSource_ - iiwaBasePositionFromSource_[IIWA_14]);
}

void AirHockey::updateCurrentObjectPosition() {

  if(isSim_){
    generateHitting7_->set_DS_attractor(objectPositionFromSource_);
    generateHitting14_->set_DS_attractor(objectPositionFromSource_);
  }
  else if(!isSim_){
    objectPositionIiwaFrames();

    generateHitting7_->set_DS_attractor(objectPositionForIiwa_[IIWA_7]);
    generateHitting14_->set_DS_attractor(objectPositionForIiwa_[IIWA_14]);
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

void AirHockey::recordRobot(int robot_name){
  // record robot data during HIT phase
  // joint state, joint velocity, EEF position + velocity

  RecordedRobotState newState;
  newState.robot_name = robot_name;
  // Get the current time
  newState.absolute_time = std::chrono::system_clock::now();

  newState.joint_pos.resize(7);
  Eigen::Map<Eigen::VectorXd> tempVector(iiwaJointState_[robot_name].position.data(), iiwaJointState_[robot_name].position.size());
  newState.joint_pos = tempVector;

  newState.joint_vel.resize(7);
  Eigen::Map<Eigen::VectorXd> tempVector2(iiwaJointState_[robot_name].velocity.data(), iiwaJointState_[robot_name].velocity.size());
  newState.joint_vel =  tempVector2;

  newState.eef_pos.resize(3);
  newState.eef_pos = iiwaPositionFromSource_[robot_name];

  newState.eef_vel.resize(3);
  newState.eef_vel = iiwaVelocityFromSource_[robot_name];

  // Add the new state to the vector
  robotStatesVector_[robot_name].push_back(newState);

}

void AirHockey::recordObject(){
  // record object position 
  // Start when robot enters HIT phase, ends ?? (with a timer like after 3 seconds)
  // ends when object has stopped moving 

  RecordedObjectState newState;

  // Get the current time
  newState.absolute_time = std::chrono::system_clock::now();

  // Get the current time
  newState.position = objectPositionFromSource_;

  // Add the new state to the vector
  objectStatesVector_.push_back(newState);

}

// Function to write a vector of RobotState structures to a file
void AirHockey::writeRobotStatesToFile(int robot_name, const std::string& filename) {
    std::ofstream outFile(filename, std::ios::app); // Open file in append mode

    if(!outFile){
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write each RobotState structure to the file
    for (const auto& state : robotStatesVector_[robot_name]) {
        // Convert time point to a string representation
        std::time_t t = std::chrono::system_clock::to_time_t(state.absolute_time);
        std::string timeStr = std::ctime(&t);

        outFile << "Robot Name: " << state.robot_name << "\n";
        outFile <<", Time: " << timeStr << "\n";
        outFile << "Joint Positions: " << state.joint_pos.transpose() << "\n";
        outFile << "Joint Velocities: " << state.joint_vel.transpose() << "\n";
        outFile << "End Effector Position: " << state.eef_pos.transpose() << "\n";
        outFile << "End Effector Velocity: " << state.eef_vel.transpose() << "\n";
        outFile << "-----------------\n"; // Separate entries with a line
    }

    outFile.close();
}

void AirHockey::writeObjectStatesToFile(const std::string& filename) {
    std::ofstream outFile(filename, std::ios::app); // Open file in append mode

    if(!outFile){
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write each RobotState structure to the file
    for (const auto& state : objectStatesVector_) {
        // Convert time point to a string representation
        std::time_t t = std::chrono::system_clock::to_time_t(state.absolute_time);
        std::string timeStr = std::ctime(&t);

        // outFile << "Object Name: " << state.robot_name << "\n";
        outFile <<", Time: " << timeStr << "\n";
        outFile << "Positions: " << state.position.transpose() << "\n";
        outFile << "-----------------\n"; // Separate entries with a line
    }

    outFile.close();
}

void AirHockey::setUpRecordingDir(){
  // figure out data structure here

  // if(!std::filesystem::is_directory(recordingFolderPath_)){
    
  //   ROS_WARN("Recording folder does not exists, creating it");
  //   std::filesystem::create_directory(recordingFolderPath_);
  //   }

}


void AirHockey::run() {

  std::cout << "IS hit : " << isHit_ << std::endl;

  // To change when got some time this is gross 
  StatesVar statesvar;
  statesvar.state_robot7_ = REST;
  statesvar.state_robot14_ = REST;
  statesvar.state_object_ = STOPPED_IN_1;
  statesvar.isHit_ = 0;

  int print_count = 0;

  while (ros::ok()) {

    statesvar = getKeyboard(statesvar);

    // DEBUG
    if(print_count%200 == 0 ){
      // std::cout << "ishit : " << statesvar.isHit_ << std::endl;
      std::cout << "Box Pos from source: " << objectPositionFromSource_ << std::endl;
      std::cout << "Box Pos from 7: " <<  objectPositionForIiwa_[IIWA_7] << std::endl;
      std::cout << "Box Pos from 14: " <<  objectPositionForIiwa_[IIWA_14] << std::endl;
      std::cout << "iiwa7_state : " << statesvar.state_robot7_ << " \n iiwa14_state : " << statesvar.state_robot14_<< std::endl;
      // std::cout << "iiwa7_vel : " << iiwaVel_[IIWA_7] << std::endl;
      // std::cout << "iiwaPos_ 7  " << iiwaPositionFromSource_[IIWA_7]<< std::endl;
      // std::cout << "returnPos_ 7  " << returnPos_[IIWA_7]<< std::endl;
    }
    print_count +=1 ;

    // Update object postion if at rest 
    if(statesvar.state_robot7_ == REST && statesvar.state_robot14_ == REST){
      // update only at REST so object state conditions works 
      updateCurrentObjectPosition();
    }

    // UPDATE robot state
    if(statesvar.state_robot7_ == HIT){
      refVelocity_[IIWA_7] = generateHitting7_->flux_DS(1.2, iiwaTaskInertiaPos_[IIWA_7]);
    }

    if(statesvar.state_robot14_ == HIT){
      refVelocity_[IIWA_14] = generateHitting14_->flux_DS(1.2, iiwaTaskInertiaPos_[IIWA_14]);
    }

    if(statesvar.state_robot7_ == REST || statesvar.isHit_ == 1){
      refVelocity_[IIWA_7] = generateHitting7_->linear_DS(returnPos_[IIWA_7]);
      statesvar.state_robot7_ = REST;
      if (statesvar.state_robot14_ == REST) { // only reset if 14 is at rest, otherwise we skip next if
              statesvar.isHit_ = 0;
      }
    }

    if(statesvar.state_robot14_ == REST || statesvar.isHit_ == 1){
      refVelocity_[IIWA_14] = generateHitting14_->linear_DS(returnPos_[IIWA_14]);
      statesvar.state_robot14_ = REST;
      statesvar.isHit_ = 0;
    }
  
    // if (!isHit_) {
    //   refVelocity_[IIWA_7] = generateHitting7_->flux_DS(0.5, iiwaTaskInertiaPos_[IIWA_7]);
    //   refVelocity_[IIWA_14] = generateHitting14_->flux_DS(0.5, iiwaTaskInertiaPos_[IIWA_14]);
    // } else {
    //   refVelocity_[IIWA_7] = generateHitting7_->linear_DS(iiwa_return_position);
    //   refVelocity_[IIWA_14] = generateHitting14_->linear_DS(iiwa_return_position);
    // }

    // if (!isHit_ && generateHitting7_->get_des_direction().dot(generateHitting7_->get_DS_attractor()
    //                                                   - generateHitting7_->get_current_position())
    //         < 0) {
    //   isHit_ = 1;
    // }

    // Update isHit 
    if (!statesvar.isHit_ && generateHitting7_->get_des_direction().dot(generateHitting7_->get_DS_attractor()
                                                      - generateHitting7_->get_current_position()) < 0) {
      statesvar.isHit_ = 1;
    }

    if (!statesvar.isHit_ && generateHitting14_->get_des_direction().dot(generateHitting14_->get_DS_attractor()
                                                      - generateHitting14_->get_current_position())  < 0) {
      statesvar.isHit_ = 1;
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
