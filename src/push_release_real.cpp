//|    Copyright (C) 2022 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "../include/push_release_real.h"
#include <experimental/filesystem>

class HitMotion{

  public:
    geometry_msgs::Pose box_pose, iiwa_pose;
    geometry_msgs::Twist iiwa_vel;
    geometry_msgs::Inertia iiwa_inertia;

    Eigen::Vector3f object_position_from_source;
    Eigen::Vector4d object_orientation_from_source;
    Eigen::Vector3f iiwa_position_from_source;
    Eigen::Vector3f iiwa_base_position_from_source;
    Eigen::Vector3f object_position_world;
    Eigen::Vector3f iiwa_vel_from_source;
    Eigen::Vector4f iiwa_orientation_from_source;
    Eigen::Vector4f iiwa_base_orientation_from_source;
    Eigen::Matrix3f iiwa_task_inertia_pos;
    Eigen::Matrix3f rotation;

    bool is_pushed = 0;
  
    
    void iiwaBasePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  
      iiwa_base_position_from_source << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
      iiwa_base_orientation_from_source << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
    }

    void iiwaPositionCallback(const geometry_msgs::Pose::ConstPtr& msg){
      iiwa_position_from_source << msg->position.x, msg->position.y, msg->position.z;
      iiwa_orientation_from_source << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
    }

    void iiwaVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
      iiwa_vel_from_source << msg->linear.x, msg->linear.y, msg->linear.z;
    }

    void objectPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
      object_position_from_source << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
      object_orientation_from_source << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
    }

    void objectPositionWorldFrame(){
          rotation << 0.0, -1.0, 0.0,
                      1.0, 0.0, 0.0,
                      0.0, 0.0, 1.0;

          object_position_world = rotation * (object_position_from_source - iiwa_base_position_from_source);
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

    HitMotion(ros::NodeHandle &nh, float frequency):
    _nh(nh), _rate(frequency){}

    void setGains(Eigen::Matrix3f &gain){
      _generate_hitting->gain = gain;
    }

    void updateCurrentEEPosition(Eigen::Vector3f &new_position){
      _generate_hitting->current_position = new_position;
    }

    void updateCurrentObjectPosition(Eigen::Vector3f &new_position){
      _generate_hitting->DS_attractor = new_position;
    }


    void publishPosQuat(const Eigen::Vector3f& DS_pos, const Eigen::Vector4f& DS_quat){
      geometry_msgs::Pose ref_pos_publish;
      ref_pos_publish.position.x = DS_pos(0);
      ref_pos_publish.position.y = DS_pos(1);
      ref_pos_publish.position.z = DS_pos(2);
      ref_pos_publish.orientation.x = DS_quat(0);
      ref_pos_publish.orientation.y = DS_quat(1);
      ref_pos_publish.orientation.z = DS_quat(2);
      ref_pos_publish.orientation.w = DS_quat(3);

      _pub_pos_quat = _nh.advertise<geometry_msgs::Pose>("/passive_control/pos_quat", 1);
      _pub_pos_quat.publish(ref_pos_publish);
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

    void publishFlux(Eigen::Matrix3f& iiwa_task_inertia_pos, Eigen::Vector3f& iiwa_vel_from_source){
      std_msgs::Float32 dir_flux_publish;
      float m_obj = 0.4;
      dir_flux_publish.data = (iiwa_task_inertia_pos(1,1)/ (iiwa_task_inertia_pos(1,1) + m_obj)) * iiwa_vel_from_source(1);
      _pub_dir_flux = _nh.advertise<std_msgs::Float32>("/iiwa/dir_flux", 1);
      _pub_dir_flux.publish(dir_flux_publish);
    }

    bool init(){
      _object_position = _nh.subscribe("/vrpn_client_node/object_1/pose", 1 , &HitMotion::objectPositionCallback, this,ros::TransportHints().reliable().tcpNoDelay());
      _iiwa_position = _nh.subscribe("/iiwa/ee_info/Pose", 1 , &HitMotion::iiwaPositionCallback, this,ros::TransportHints().reliable().tcpNoDelay());
      _iiwa_vel = _nh.subscribe("/iiwa/ee_info/Vel", 1 , &HitMotion::iiwaVelCallback, this,ros::TransportHints().reliable().tcpNoDelay());
      _iiwa_inertia = _nh.subscribe("/iiwa/Inertia/taskPos", 1 , &HitMotion::iiwaInertiaCallback, this,ros::TransportHints().reliable().tcpNoDelay());
      _iiwa_base_position = _nh.subscribe("/vrpn_client_node/iiwa_7_base/pose", 1 , &HitMotion::iiwaBasePositionCallback, this,ros::TransportHints().reliable().tcpNoDelay());

      while(object_position_from_source.norm() == 0){
        updateCurrentObjectPosition(object_position_from_source);
        ros::spinOnce();
        _rate.sleep();
      }

      objectPositionWorldFrame();
      std::cout << "object at: " << object_position_world.transpose() << std::endl;


      Eigen::Vector3f object_offset = {0.0, -0.2, -0.00}; //normal
      // Eigen::Vector3f object_offset = {0.05, -0.15, -0.05}; //small object

      _generate_hitting->current_position = iiwa_position_from_source;
      _generate_hitting->DS_attractor = object_position_world + object_offset;

      _nh.getParam("ref_velocity/x", ref_velocity[0]);
      _nh.getParam("ref_velocity/y", ref_velocity[1]);
      _nh.getParam("ref_velocity/z", ref_velocity[2]);
      _nh.getParam("ref_quat/w", ref_quat[0]);
      _nh.getParam("ref_quat/x", ref_quat[1]);
      _nh.getParam("ref_quat/y", ref_quat[2]);
      _nh.getParam("ref_quat/z", ref_quat[3]);
      _nh.getParam("hit_direction/x", hit_direction[0]);
      _nh.getParam("hit_direction/y", hit_direction[1]);
      _nh.getParam("hit_direction/z", hit_direction[2]);

      _generate_hitting->des_direction = hit_direction;

      std::cout << "INITIALISED" << std::endl;

      return true;
      
    }

    void run(){
      Eigen::Vector3f iiwa_return_position = {0.5, -0.25, 0.3};
      Eigen::Vector3f final_position = {0.6, 0.1, 0.2};
      while(ros::ok()){

        if (!is_pushed){
          ref_velocity = _generate_hitting->flux_DS(0.5, iiwa_task_inertia_pos);

        }else{
          ref_velocity = _generate_hitting->linear_DS(iiwa_return_position);
        }
        if(!is_pushed && _generate_hitting->des_direction.dot(_generate_hitting->DS_attractor - _generate_hitting->current_position) < 0){
          is_pushed = 1;
        }

        updateCurrentEEPosition(iiwa_position_from_source);
        publishVelQuat(ref_velocity, ref_quat);
        publishFlux(iiwa_task_inertia_pos, iiwa_vel_from_source);
        ros::spinOnce();
        _rate.sleep();
      }

      publishVelQuat(ref_velocity, ref_quat);
      publishFlux(iiwa_task_inertia_pos, iiwa_vel_from_source);
      ros::spinOnce(); 
      _rate.sleep();
      ros::shutdown();
    }


  protected:
    ros::Rate _rate;
    ros::NodeHandle _nh;
    ros::Publisher _pub_vel_quat;
    ros::Publisher _pub_pos_quat;
    ros::Publisher _pub_dir_flux;
    ros::Subscriber _object_position;
    ros::Subscriber _iiwa_position;
    ros::Subscriber _iiwa_vel;
    ros::Subscriber _iiwa_base_position;
    ros::Subscriber _iiwa_inertia;
    std::unique_ptr<hitting_DS> _generate_hitting = std::make_unique<hitting_DS>(iiwa_position_from_source, object_position_world);
    Eigen::Vector3f ref_velocity = {0.0, 0.0, 0.0}; 
    Eigen::Vector3f test_velocity = {0.0 ,0.0 ,0.0}; 
    Eigen::Vector3f hit_direction = {0.0, 1.0, 0.0};
    float hit_momentum;
    Eigen::Vector4f ref_quat = Eigen::Vector4f::Zero();

};


int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "push_release_real");
  float frequency = 200.0f;
  ros::NodeHandle nh;

  std::unique_ptr<HitMotion> generate_motion = std::make_unique<HitMotion>(nh, frequency);

  if (!generate_motion->init()){
    return -1;
  }else{
    sleep(5);
    generate_motion->run();
  }


  return 0;

}
