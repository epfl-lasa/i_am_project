
#include "AirHockey.hpp"

bool AirHockey::init() {
  // Get topics names
  if (!nh_.getParam("/passive_control/vel_quat_7", pubVelQuatTopic_[IIWA_1])) {
    ROS_ERROR("Topic /passive_control iiwa 7 not found");
  }
  if (!nh_.getParam("/passive_control/vel_quat_14", pubVelQuatTopic_[IIWA_2])) {
    ROS_ERROR("Topic /passive_control iiwa 14 not found");
  }

  if (!nh_.getParam("/gazebo/link_states", iiwaPositionTopic_)) { ROS_ERROR("Topic /gazebo/link_states not found"); }

  if (!nh_.getParam("/iiwa/inertia/taskPos_7", iiwaInertiaTopic_[IIWA_1])) {
    ROS_ERROR("Topic /iiwa/inertia/taskPos not found");
  }
  if (!nh_.getParam("/iiwa/inertia/taskPos_14", iiwaInertiaTopic_[IIWA_2])) {
    ROS_ERROR("Topic /iiwa/inertia/taskPos not found");
  }

  if (!nh_.getParam("/gazebo/model_states", objectPositionTopic_)) {
    ROS_ERROR("Topic /gazebo/model_states not found");
  }

  // Init publishers
  pubVelQuat_[IIWA_1] = nh_.advertise<geometry_msgs::Pose>(pubVelQuatTopic_[IIWA_1], 1);
  pubVelQuat_[IIWA_2] = nh_.advertise<geometry_msgs::Pose>(pubVelQuatTopic_[IIWA_2], 1);

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

  iiwaInertia_[IIWA_1] =
      nh_.subscribe<geometry_msgs::Inertia>(iiwaInertiaTopic_[IIWA_1],
                                            1,
                                            boost::bind(&AirHockey::iiwaInertiaCallbackGazebo, this, _1, IIWA_1),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());
  iiwaInertia_[IIWA_2] =
      nh_.subscribe<geometry_msgs::Inertia>(iiwaInertiaTopic_[IIWA_2],
                                            1,
                                            boost::bind(&AirHockey::iiwaInertiaCallbackGazebo, this, _1, IIWA_2),
                                            ros::VoidPtr(),
                                            ros::TransportHints().reliable().tcpNoDelay());

  while (objectPositionFromSource_.norm() == 0) {
    this->updateCurrentObjectPosition(objectPositionFromSource_);
    ros::spinOnce();
    rate_.sleep();
  }

  generateHitting7_->set_current_position(iiwaPositionFromSource_[IIWA_1]);
  generateHitting14_->set_current_position(iiwaPositionFromSource_[IIWA_2]);
  generateHitting7_->set_DS_attractor(objectPositionFromSource_);
  generateHitting14_->set_DS_attractor(objectPositionFromSource_);

  if (!nh_.getParam("iiwa7/ref_velocity/y", refVelocity_[IIWA_1][1])) { ROS_ERROR("Topic ref_velocity/y not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/y", refVelocity_[IIWA_2][1])) { ROS_ERROR("Topic ref_velocity/y not found"); }
  if (!nh_.getParam("iiwa7/ref_velocity/x", refVelocity_[IIWA_1][0])) { ROS_ERROR("Topic ref_velocity/x not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/x", refVelocity_[IIWA_2][0])) { ROS_ERROR("Topic ref_velocity/x not found"); }
  if (!nh_.getParam("iiwa7/ref_velocity/z", refVelocity_[IIWA_1][2])) { ROS_ERROR("Topic ref_velocity/z not found"); }
  if (!nh_.getParam("iiwa14/ref_velocity/z", refVelocity_[IIWA_2][2])) { ROS_ERROR("Topic ref_velocity/z not found"); }
  if (!nh_.getParam("iiwa7/ref_quat/w", refQuat_[IIWA_1][0])) { ROS_ERROR("Topic ref_quat/w not found"); }
  if (!nh_.getParam("iiwa14/ref_quat/w", refQuat_[IIWA_2][0])) { ROS_ERROR("Topic ref_quat/w not found"); }
  if (!nh_.getParam("iiwa7/ref_quat/x", refQuat_[IIWA_1][1])) { ROS_ERROR("Topic ref_quat/x not found"); }
  if (!nh_.getParam("iiwa14/ref_quat/x", refQuat_[IIWA_2][1])) { ROS_ERROR("Topic ref_quat/x not found"); }
  if (!nh_.getParam("iiwa7/ref_quat/y", refQuat_[IIWA_1][2])) { ROS_ERROR("Topic ref_quat/y not found"); }
  if (!nh_.getParam("iiwa14/ref_quat/y", refQuat_[IIWA_2][2])) { ROS_ERROR("Topic ref_quat/y not found"); }
  if (!nh_.getParam("iiwa7/ref_quat/z", refQuat_[IIWA_1][3])) { ROS_ERROR("Topic ref_quat/z not found"); }
  if (!nh_.getParam("iiwa14/ref_quat/z", refQuat_[IIWA_2][3])) { ROS_ERROR("Topic ref_quat/z not found"); }
  if (!nh_.getParam("iiwa7/hit_direction/x", hitDirection_[IIWA_1][0])) {
    ROS_ERROR("Topic hit_direction/x not found");
  }
  if (!nh_.getParam("iiwa14/hit_direction/x", hitDirection_[IIWA_2][0])) {
    ROS_ERROR("Topic hit_direction/x not found");
  }
  if (!nh_.getParam("iiwa7/hit_direction/y", hitDirection_[IIWA_1][1])) {
    ROS_ERROR("Topic hit_direction/y not found");
  }
  if (!nh_.getParam("iiwa14/hit_direction/y", hitDirection_[IIWA_2][1])) {
    ROS_ERROR("Topic hit_direction/y not found");
  }
  if (!nh_.getParam("iiwa7/hit_direction/z", hitDirection_[IIWA_1][2])) {
    ROS_ERROR("Topic hit_direction/z not found");
  }
  if (!nh_.getParam("iiwa14/hit_direction/z", hitDirection_[IIWA_2][2])) {
    ROS_ERROR("Topic hit_direction/z not found");
  }

  generateHitting7_->set_des_direction(hitDirection_[IIWA_1]);
  generateHitting14_->set_des_direction(hitDirection_[IIWA_2]);

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
  int indexIiwa1 = getIndex(linkStates.name, "iiwa1::iiwa_link_7");// End effector is the 7th link in KUKA IIWA
  int indexIiwa2 = getIndex(linkStates.name, "iiwa2::iiwa_link_7");// End effector is the 7th link in KUKA IIWA

  iiwaPose_[IIWA_1] = linkStates.pose[indexIiwa1];
  iiwaPositionFromSource_[IIWA_1] << iiwaPose_[IIWA_1].position.x, iiwaPose_[IIWA_1].position.y,
      iiwaPose_[IIWA_1].position.z;
  iiwaVel_[IIWA_1] = linkStates.twist[indexIiwa1];

  iiwaPose_[IIWA_2] = linkStates.pose[indexIiwa2];
  iiwaPositionFromSource_[IIWA_2] << iiwaPose_[IIWA_2].position.x, iiwaPose_[IIWA_2].position.y,
      iiwaPose_[IIWA_2].position.z;
  iiwaVel_[IIWA_2] = linkStates.twist[indexIiwa2];
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
  generateHitting7_->set_current_position(new_position[IIWA_1]);
  generateHitting14_->set_current_position(new_position[IIWA_2]);
}

void AirHockey::publishVelQuat(Eigen::Vector3f DS_vel[], Eigen::Vector4f DS_quat[]) {
  geometry_msgs::Pose ref_vel_publish_7, ref_vel_publish_14;
  ref_vel_publish_7.position.x = DS_vel[IIWA_1](0);
  ref_vel_publish_7.position.y = DS_vel[IIWA_1](1);
  ref_vel_publish_7.position.z = DS_vel[IIWA_1](2);
  ref_vel_publish_7.orientation.x = DS_quat[IIWA_1](0);
  ref_vel_publish_7.orientation.y = DS_quat[IIWA_1](1);
  ref_vel_publish_7.orientation.z = DS_quat[IIWA_1](2);
  ref_vel_publish_7.orientation.w = DS_quat[IIWA_1](3);
  pubVelQuat_[IIWA_1].publish(ref_vel_publish_7);

  ref_vel_publish_14.position.x = DS_vel[IIWA_2](0);
  ref_vel_publish_14.position.y = DS_vel[IIWA_2](1);
  ref_vel_publish_14.position.z = DS_vel[IIWA_2](2);
  ref_vel_publish_14.orientation.x = DS_quat[IIWA_2](0);
  ref_vel_publish_14.orientation.y = DS_quat[IIWA_2](1);
  ref_vel_publish_14.orientation.z = DS_quat[IIWA_2](2);
  ref_vel_publish_14.orientation.w = DS_quat[IIWA_2](3);
  pubVelQuat_[IIWA_2].publish(ref_vel_publish_14);
}

void AirHockey::run() {
  Eigen::Vector3f iiwa_return_position = {0.3, 0.0, 0.5};

  std::cout << "IS hit : " << isHit_ << std::endl;

  while (ros::ok()) {

    std::cout << "IS hit : " << isHit_ << std::endl;
    std::cout << "Box Pos : " << objectPositionFromSource_ << std::endl;

    if (!isHit_) {
      refVelocity_[IIWA_1] = generateHitting7_->flux_DS(0.5, iiwaTaskInertiaPos_[IIWA_1]);
      refVelocity_[IIWA_2] = generateHitting14_->flux_DS(0.5, iiwaTaskInertiaPos_[IIWA_2]);
    } else {
      refVelocity_[IIWA_1] = generateHitting7_->linear_DS(iiwa_return_position);
      refVelocity_[IIWA_2] = generateHitting14_->linear_DS(iiwa_return_position);
    }

    // if (!isHit_
    //     && generateHitting7_->get_des_direction().dot(generateHitting7_->get_DS_attractor()
    //                                                   - generateHitting7_->get_current_position())
    //         < 0) {
    //   isHit_ = 1;
    // }
    std::cout << "refVelocity_ 7  " << refVelocity_[IIWA_1] << std::endl;

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
    // generate_motion->run();
  }

  return 0;
}
