
#include "AirHockey.hpp"

bool AirHockey::init() {
  // Get topics names
  if (!nh_.getParam("/passive_control/vel_quat_7", pubVelQuatTopic_[IIWA_7])) {
    ROS_ERROR("Topic /passive_control iiwa 7 not found");
  }
  if (!nh_.getParam("/passive_control/vel_quat_14", pubVelQuatTopic_[IIWA_14])) {
    ROS_ERROR("Topic /passive_control iiwa 14 not found");
  }

  if (!nh_.getParam("/gazebo/link_states", iiwaPositionTopic_)) { ROS_ERROR("Topic /gazebo/link_states not found"); }

  if (!nh_.getParam("/iiwa/inertia/taskPos_7", iiwaInertiaTopic_[IIWA_7])) {
    ROS_ERROR("Topic /iiwa/inertia/taskPos not found");
  }
  if (!nh_.getParam("/iiwa/inertia/taskPos_14", iiwaInertiaTopic_[IIWA_14])) {
    ROS_ERROR("Topic /iiwa/inertia/taskPos not found");
  }

  if (!nh_.getParam("/gazebo/model_states", objectPositionTopic_)) {
    ROS_ERROR("Topic /gazebo/model_states not found");
  }

  // Init publishers
  pubVelQuat_[IIWA_7] = nh_.advertise<geometry_msgs::Pose>(pubVelQuatTopic_[IIWA_7], 1);
  pubVelQuat_[IIWA_14] = nh_.advertise<geometry_msgs::Pose>(pubVelQuatTopic_[IIWA_14], 1);

  // Init subscribers
  objectPosition_ = nh_.subscribe(objectPositionTopic_,
                                  1,
                                  &AirHockey::objectPositionCallbackGazebo,
                                  this,
                                  ros::TransportHints().reliable().tcpNoDelay());
  iiwaPosition_ = nh_.subscribe(iiwaPositionTopic_,
                                1,
                                &AirHockey::iiwaPositionCallbackGazebo,
                                this,
                                ros::TransportHints().reliable().tcpNoDelay());

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
    this->updateCurrentObjectPosition(objectPositionFromSource_);
    ros::spinOnce();
    rate_.sleep();
  }

  generateHitting7_->set_current_position(iiwaPositionFromSource_[IIWA_7]);
  generateHitting14_->set_current_position(iiwaPositionFromSource_[IIWA_14]);
  generateHitting7_->set_DS_attractor(objectPositionFromSource_);
  generateHitting14_->set_DS_attractor(objectPositionFromSource_);

  if (!nh_.getParam("iiwa7/ref_velocity/y", refVelocity_[IIWA_7][1])) { ROS_ERROR("Topic ref_velocity/y not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/y", refVelocity_[IIWA_14][1])) { ROS_ERROR("Topic ref_velocity/y not found"); }
  if (!nh_.getParam("iiwa7/ref_velocity/x", refVelocity_[IIWA_7][0])) { ROS_ERROR("Topic ref_velocity/x not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/x", refVelocity_[IIWA_14][0])) { ROS_ERROR("Topic ref_velocity/x not found"); }
  if (!nh_.getParam("iiwa7/ref_velocity/z", refVelocity_[IIWA_7][2])) { ROS_ERROR("Topic ref_velocity/z not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/z", refVelocity_[IIWA_14][2])) { ROS_ERROR("Topic ref_velocity/z not found"); }
  // if (!nh_.getParam("iiwa7/ref_quat/w", refQuat_[IIWA_7][0])) { ROS_ERROR("Topic ref_quat/w not found"); }
  // if (!nh_.getParam("iiwa14/ref_quat/w", refQuat_[IIWA_14][0])) { ROS_ERROR("Topic ref_quat/w not found"); }
  // if (!nh_.getParam("iiwa7/ref_quat/x", refQuat_[IIWA_7][1])) { ROS_ERROR("Topic ref_quat/x not found"); }
  // if (!nh_.getParam("iiwa14/ref_quat/x", refQuat_[IIWA_14][1])) { ROS_ERROR("Topic ref_quat/x not found"); }
  // if (!nh_.getParam("iiwa7/ref_quat/y", refQuat_[IIWA_7][2])) { ROS_ERROR("Topic ref_quat/y not found"); }
  // if (!nh_.getParam("iiwa14/ref_quat/y", refQuat_[IIWA_14][2])) { ROS_ERROR("Topic ref_quat/y not found"); }
  // if (!nh_.getParam("iiwa7/ref_quat/z", refQuat_[IIWA_7][3])) { ROS_ERROR("Topic ref_quat/z not found"); }
  // if (!nh_.getParam("iiwa14/ref_quat/z", refQuat_[IIWA_14][3])) { ROS_ERROR("Topic ref_quat/z not found"); }
  if (!nh_.getParam("iiwa7/return_position/x", returnPos_[IIWA_7][0])) { ROS_ERROR("Topic ref_quat/x not found"); }
  if (!nh_.getParam("iiwa14/return_position/x", returnPos_[IIWA_14][0])) { ROS_ERROR("Topic return_position/x not found"); }
  if (!nh_.getParam("iiwa7/return_position/y", returnPos_[IIWA_7][1])) { ROS_ERROR("Topic return_position/y not found"); }
  if (!nh_.getParam("iiwa14/return_position/y", returnPos_[IIWA_14][1])) { ROS_ERROR("Topic return_position/y not found"); }
  if (!nh_.getParam("iiwa7/return_position/z", returnPos_[IIWA_7][2])) { ROS_ERROR("Topic return_position/z not found"); }
  if (!nh_.getParam("iiwa14/return_position/z", returnPos_[IIWA_14][2])) { ROS_ERROR("Topic return_position/z not found"); }

  if (!nh_.getParam("iiwa7/hit_direction/x", hitDirection_[IIWA_7][0])) {
    ROS_ERROR("Topic hit_direction/x not found");
  }
  if (!nh_.getParam("iiwa14/hit_direction/x", hitDirection_[IIWA_14][0])) {
    ROS_ERROR("Topic hit_direction/x not found");
  }
  if (!nh_.getParam("iiwa7/hit_direction/y", hitDirection_[IIWA_7][1])) {
    ROS_ERROR("Topic hit_direction/y not found");
  }
  if (!nh_.getParam("iiwa14/hit_direction/y", hitDirection_[IIWA_14][1])) {
    ROS_ERROR("Topic hit_direction/y not found");
  }
  if (!nh_.getParam("iiwa7/hit_direction/z", hitDirection_[IIWA_7][2])) {
    ROS_ERROR("Topic hit_direction/z not found");
  }
  if (!nh_.getParam("iiwa14/hit_direction/z", hitDirection_[IIWA_14][2])) {
    ROS_ERROR("Topic hit_direction/z not found");
  }

  generateHitting7_->set_des_direction(hitDirection_[IIWA_7]);
  generateHitting14_->set_des_direction(hitDirection_[IIWA_14]);

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
  int indexIiwa1 = getIndex(linkStates.name, "iiwa7::iiwa7_link_7");// End effector is the 7th link in KUKA IIWA
  int indexIiwa2 = getIndex(linkStates.name, "iiwa14::iiwa14_link_7");// End effector is the 7th link in KUKA IIWA

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

void AirHockey::updateCurrentObjectPosition(Eigen::Vector3f& new_position) {
  generateHitting7_->set_DS_attractor(new_position);
  generateHitting14_->set_DS_attractor(new_position);
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
      std::cout << "IS hit : " << statesvar.isHit_ << std::endl;
      std::cout << "Box Pos : " << objectPositionFromSource_ << std::endl;
      std::cout << "iiwa7_state : " << statesvar.state_robot7_ << " \n iiwa14_state : " << statesvar.state_robot14_<< std::endl;
      std::cout << "iiwa7_vel : " << iiwaVel_[IIWA_7] << std::endl;
      std::cout << "pos 7 from gazebo: " <<  iiwaPositionFromSource_[IIWA_7] << std::endl;
      std::cout << "pos 14 from gazebo: " <<  iiwaPositionFromSource_[IIWA_14] << std::endl;
      std::cout << "refVelocity_ 7  " << refVelocity_[IIWA_7] << std::endl;
      // std::cout << "DS 7 pos : " << generateHitting7_->get_current_position()<< std::endl;
      // std::cout << "DS 14 pos : " << generateHitting14_->get_current_position()<< std::endl;
      // std::cout << "DS 7 attr : " << generateHitting7_->get_DS_attractor()<< std::endl;
      // std::cout << "DS 14 attr: " << generateHitting14_->get_DS_attractor()<< std::endl;
    }
    print_count +=1 ;


    if(statesvar.state_robot7_ == HIT){
      refVelocity_[IIWA_7] = generateHitting7_->flux_DS(0.5, iiwaTaskInertiaPos_[IIWA_7]);
    }

    if(statesvar.state_robot14_ == HIT){
      refVelocity_[IIWA_14] = generateHitting14_->flux_DS(0.5, iiwaTaskInertiaPos_[IIWA_14]);
    }

    if(statesvar.state_robot7_ == REST){
      refVelocity_[IIWA_7] = generateHitting7_->linear_DS(returnPos_[IIWA_7]);
    }

    if(statesvar.state_robot14_ == REST){
      refVelocity_[IIWA_14] = generateHitting14_->linear_DS(returnPos_[IIWA_14]);
    }

    // if (!isHit_) {
    //   refVelocity_[IIWA_7] = generateHitting7_->flux_DS(0.5, iiwaTaskInertiaPos_[IIWA_7]);
    //   refVelocity_[IIWA_14] = generateHitting14_->flux_DS(0.5, iiwaTaskInertiaPos_[IIWA_14]);
    // } else {
    //   refVelocity_[IIWA_7] = generateHitting7_->linear_DS(iiwa_return_position);
    //   refVelocity_[IIWA_14] = generateHitting14_->linear_DS(iiwa_return_position);
    // }

    // if (!isHit_
    //     && generateHitting7_->get_des_direction().dot(generateHitting7_->get_DS_attractor()
    //                                                   - generateHitting7_->get_current_position())
    //         < 0) {
    //   isHit_ = 1;
    // }

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
    std::cout << "OK ";
    generate_motion->run();
  }

  return 0;
}
