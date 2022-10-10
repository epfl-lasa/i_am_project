//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "../include/node_inertia_QP.h"
#include <experimental/filesystem>

class InertiaMotion{

  public:
    Eigen::Vector3f iiwa_position_from_source;
    Eigen::Vector3f iiwa_vel_from_source;
    Eigen::Vector4f iiwa_orientation_from_source;
    Eigen::Matrix3f iiwa_task_inertia_pos;

    void iiwaPositionCallback(const geometry_msgs::Pose::ConstPtr& msg){
      iiwa_position_from_source << msg->position.x, msg->position.y, msg->position.z;
      iiwa_orientation_from_source << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
    }

    void iiwaInertiaCallback(const geometry_msgs::Inertia& inertia_msg){
      iiwa_task_inertia_pos(0,0) = inertia_msg.ixx;
      iiwa_task_inertia_pos(1,1) = inertia_msg.iyy;
      iiwa_task_inertia_pos(2,2) = inertia_msg.izz;
      iiwa_task_inertia_pos(0,1) = inertia_msg.ixy;
      iiwa_task_inertia_pos(1,0) = inertia_msg.ixy;
      iiwa_task_inertia_pos(0,2) = inertia_msg.ixz;
      iiwa_task_inertia_pos(2,0) = inertia_msg.ixz;
      iiwa_task_inertia_pos(1,2) = inertia_msg.iyz;
      iiwa_task_inertia_pos(2,1) = inertia_msg.iyz;
    }

    InertiaMotion(ros::NodeHandle &nh, float frequency):
    _nh(nh), _rate(frequency){}

    void setGains(Eigen::Matrix3f &gain){
      _generate_inertia_motion->gain = gain;
    }

    void updateCurrentEEPosition(Eigen::Vector3f &new_position){
      _generate_inertia_motion->current_position = new_position;
    }

    void publishJointPosition(const Eigen::VectorXf& joint_pos){
      std_msgs::Float64MultiArray ref_joint_position;

      for (int i = 0; i < joint_pos.size(); ++i){
        ref_joint_position.data.push_back(joint_pos[i]);
      }

      _pub_ref_position = _nh.advertise<std_msgs::Float64MultiArray>("/iiwa/PositionController/command", 1);
      _pub_ref_position.publish(ref_joint_position);
    }

    void publishVelQuat(Eigen::Vector3f& DS_vel, Eigen::Vector4f& DS_quat){
      geometry_msgs::Pose ref_vel_publish;
      ref_vel_publish.position.x = DS_vel(0);
      ref_vel_publish.position.y = DS_vel(1);
      ref_vel_publish.position.z = DS_vel(2);
      ref_vel_publish.orientation.x = DS_quat(0);
      ref_vel_publish.orientation.y = DS_quat(1);
      ref_vel_publish.orientation.z = DS_quat(2);
      ref_vel_publish.orientation.w = DS_quat(3);

      _pub_vel_quat = _nh.advertise<geometry_msgs::Pose>("/passive_control/vel_quat", 1);
      _pub_vel_quat.publish(ref_vel_publish);
    }


    bool init(){
      _iiwa_position = _nh.subscribe("/iiwa/ee_info/Pose", 1 , &InertiaMotion::iiwaPositionCallback, this,ros::TransportHints().reliable().tcpNoDelay());
      _iiwa_inertia = _nh.subscribe("/iiwa/Inertia/taskPos", 1 , &InertiaMotion::iiwaInertiaCallback, this,ros::TransportHints().reliable().tcpNoDelay());

      _generate_inertia_motion->current_position = iiwa_position_from_source;
      return true;
      
    }

    void run(){
      
      while(ros::ok()){
        ref_velocity = _generate_inertia_motion->linear_DS();
        updateCurrentEEPosition(iiwa_position_from_source);
        publishVelQuat(ref_velocity, ref_quat);
        ros::spinOnce();
        _rate.sleep();
      }

      publishVelQuat(ref_velocity, ref_quat);
      ros::spinOnce(); 
      _rate.sleep();
      ros::shutdown();
    }


  protected:
    ros::Rate _rate;
    ros::NodeHandle _nh;
    ros::Publisher _pub_ref_position;
    ros::Subscriber _iiwa_position;
    ros::Subscriber _iiwa_inertia;
    std::unique_ptr<hitting_DS> _generate_inertia_motion = std::make_unique<hitting_DS>(iiwa_position_from_source);
    Eigen::Vector4f ref_quat = Eigen::Vector4f::Zero();
    Eigen::Vector3f ref_velocity = Eigen::Vector3f::Zero();
    ros::Publisher _pub_vel_quat;



};


int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "inertia_QP");
  float frequency = 200.0f;
  ros::NodeHandle nh;

  std::unique_ptr<InertiaMotion> generate_motion = std::make_unique<InertiaMotion>(nh, frequency);

  if (!generate_motion->init()){
    return -1;
  }else{
    generate_motion->run();
  }

  return 0;

}
