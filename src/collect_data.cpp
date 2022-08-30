//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    website: lasa.epfl.ch

#include "../include/1v1_collect_data.h"

using namespace std;

bool object_real;
bool iiwa_real;
bool manual_mode;
bool hollow;

int mode1 = 5;
int mode2 = 5;

Eigen::Matrix3d R_Opti; 

// Stuff to measure
geometry_msgs::Pose object_pose, ee1_pose, ee2_pose;
geometry_msgs::Twist object_twist, ee1_twist, ee2_twist;

Eigen::Vector3d object_pos, object_vel;

Eigen::Vector3d object_rpy;
Eigen::Vector3d ee1_rpy;
Eigen::Vector3d ee2_rpy;
double object_th, object_th_dot, object_th_mod;

std::vector<double> iiwa1_joint_angles, iiwa2_joint_angles;


// Stuff to estimate
Eigen::Vector3d predict_pos;
double ETA;
double stdev;

//double predict_th;


//Callback functions for subscribers
//Gazebo
int getIndex(std::vector<std::string> v, std::string value){
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void objectSimCallback(const gazebo_msgs::ModelStates model_states){
  int box_index = getIndex(model_states.name, "my_box");

  object_pose = model_states.pose[box_index];
  object_twist = model_states.twist[box_index];
  object_pos << object_pose.position.x, object_pose.position.y, object_pose.position.z;
  object_vel << object_twist.linear.x, object_twist.linear.y, object_twist.linear.z;

  object_rpy = quatToRPY({object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z}); //get orientation in rpy
  object_th = object_rpy[2];        
  object_th_dot = object_twist.angular.z;                                                                                     //only the z-axis
  object_th_mod = std::fmod(object_th+M_PI+M_PI/4,M_PI/2)-M_PI/4;                                                            //get relative angle of box face facing the arm
}

void iiwaSimCallback(const gazebo_msgs::LinkStates link_states){
  int iiwa1_ee_index = getIndex(link_states.name, "iiwa1::iiwa1_link_7");
  int iiwa2_ee_index = getIndex(link_states.name, "iiwa2::iiwa2_link_7");

  ee1_pose = link_states.pose[iiwa1_ee_index];
  ee1_rpy = quatToRPY({ee1_pose.orientation.w, ee1_pose.orientation.x, ee1_pose.orientation.y, ee1_pose.orientation.z}); //get orientation in rpy
  ee2_pose = link_states.pose[iiwa2_ee_index];
  ee2_rpy = quatToRPY({ee2_pose.orientation.w, ee2_pose.orientation.x, ee2_pose.orientation.y, ee2_pose.orientation.z}); //get orientation in rpy
}

//Optitrack
void objectCallback(const geometry_msgs::Pose object_pose){
  object_pos << object_pose.position.x, object_pose.position.y, object_pose.position.z;
  object_pos = R_Opti*object_pos;

  object_rpy = quatToRPY({object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z}); //get orientation in rpy
  object_th = object_rpy[2];                                                                                             //only the z-axis
  object_th_mod = std::fmod(object_th+M_PI+M_PI/4,M_PI/2)-M_PI/4;                                                            //get relative angle of box face facing the arm
}

void iiwa1EEPoseCallback(const geometry_msgs::Pose ee_pose){
  ee1_pose.position.x = ee_pose.position.x;
  ee1_pose.position.y = ee_pose.position.y;
  ee1_pose.position.z = ee_pose.position.z;
  ee1_pose.orientation.w = ee_pose.orientation.w;
  ee1_pose.orientation.x = ee_pose.orientation.x;
  ee1_pose.orientation.y = ee_pose.orientation.y;
  ee1_pose.orientation.z = ee_pose.orientation.z;
  ee1_rpy = quatToRPY({ee1_pose.orientation.w, ee1_pose.orientation.x, ee1_pose.orientation.y, ee1_pose.orientation.z}); //get orientation in rpy
}

void iiwa2EEPoseCallback(const geometry_msgs::Pose ee_pose){
  ee2_pose.position.x = ee_pose.position.x;
  ee2_pose.position.y = ee_pose.position.y;
  ee2_pose.position.z = ee_pose.position.z;
  ee2_pose.orientation.w = ee_pose.orientation.w;
  ee2_pose.orientation.x = ee_pose.orientation.x;
  ee2_pose.orientation.y = ee_pose.orientation.y;
  ee2_pose.orientation.z = ee_pose.orientation.z;
  ee2_rpy = quatToRPY({ee2_pose.orientation.w, ee2_pose.orientation.x, ee2_pose.orientation.y, ee2_pose.orientation.z}); //get orientation in rpy
}

//Passive_track (iiwa_toolkit)
void iiwa1EETwistCallback(const geometry_msgs::Twist ee_twist){
  ee1_twist.linear.x = ee_twist.linear.x;
  ee1_twist.linear.y = ee_twist.linear.y;
  ee1_twist.linear.z = ee_twist.linear.z;
  ee1_twist.angular.x = ee_twist.angular.x;
  ee1_twist.angular.y = ee_twist.angular.y;
  ee1_twist.angular.z = ee_twist.angular.z;
}

void iiwa2EETwistCallback(const geometry_msgs::Twist ee_twist){
  ee2_twist.linear.x = ee_twist.linear.x;
  ee2_twist.linear.y = ee_twist.linear.y;
  ee2_twist.linear.z = ee_twist.linear.z;
  ee2_twist.angular.x = ee_twist.angular.x;
  ee2_twist.angular.y = ee_twist.angular.y;
  ee2_twist.angular.z = ee_twist.angular.z;
}

void iiwa1JointCallback(sensor_msgs::JointState joint_states){
  iiwa1_joint_angles = joint_states.position;
}

void iiwa2JointCallback(sensor_msgs::JointState joint_states){
  iiwa2_joint_angles = joint_states.position;
}

//i_am_predict
void estimateObjectCallback(std_msgs::Float64MultiArray estimation){
  stdev = estimation.data[0];
  ETA = estimation.data[1];
  predict_pos << estimation.data[2], estimation.data[3], estimation.data[4];
  if(object_real){
    object_vel << estimation.data[5], estimation.data[6], estimation.data[7];
  }
  //predict_th = estimation.data[8];
}


void modeIwaa1Callback(std_msgs::Int16 msg){
  mode1 = msg.data;
}

void modeIwaa2Callback(std_msgs::Int16 msg){
  mode2 = msg.data;
}



int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "collect_data");
  ros::NodeHandle nh;
  ros::Rate rate(100);


  //Get environment setting from param
  if(!nh.getParam("object_real", object_real)){ROS_ERROR("Param object_real not found");}
  if(!nh.getParam("iiwa_real", iiwa_real)){ROS_ERROR("Param iiwa_real not found");}
  if(!nh.getParam("manual_mode", manual_mode)){ROS_ERROR("Param manual_mode not found");}
  if(!nh.getParam("hollow", hollow)){ROS_ERROR("Param hollow not found");}


  //Subscribers to object and IIWA states
  ros::Subscriber object_subs;
  ros::ServiceClient set_state_client;
  if (object_real){
    object_subs = nh.subscribe("/simo_track/object_pose", 10 , objectCallback);
    }
    else{
    object_subs = nh.subscribe("/gazebo/model_states", 10, objectSimCallback);
    //Client to reset object pose
    ros::service::waitForService("gazebo/set_model_state");
    set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
  }
  

  ros::Subscriber iiwa1_ee_subs;
  ros::Subscriber iiwa2_ee_subs;
  ros::Subscriber iiwa_subs;
  if (iiwa_real){
    iiwa1_ee_subs = nh.subscribe("/simo_track/robot_left/ee_pose", 10, iiwa1EEPoseCallback);
    iiwa2_ee_subs = nh.subscribe("/simo_track/robot_right/ee_pose", 10, iiwa2EEPoseCallback);
    }
    else{
    iiwa_subs = nh.subscribe("/gazebo/link_states", 10, iiwaSimCallback);
  }

  ros::Subscriber iiwa1_ee_twist_subs = nh.subscribe("iiwa1/ee_twist", 100, iiwa1EETwistCallback);          //from passive_control and iiwa_ros
  ros::Subscriber iiwa2_ee_twist_subs = nh.subscribe("iiwa2/ee_twist", 100, iiwa2EETwistCallback);
  ros::Subscriber iiwa1_joint_subs = nh.subscribe("/iiwa1/joint_states", 100, iiwa1JointCallback);
  ros::Subscriber iiwa2_joint_subs = nh.subscribe("/iiwa2/joint_states", 100, iiwa2JointCallback);
  
  //Subcriber to position estimator
  ros::Subscriber estimate_object_subs = nh.subscribe("estimate/object",10,estimateObjectCallback);

  //Subscriber to Robot modes from master node
  ros::Subscriber mode1_sub = nh.subscribe("/mode/iiwa1",10,modeIwaa1Callback);
  ros::Subscriber mode2_sub = nh.subscribe("/mode/iiwa2",10,modeIwaa2Callback);


  //Center points of desired workspaces (used to aim for and tell what orientation)
  std::vector<double> center1vec;
  std::vector<double> center2vec;
  Eigen::Vector3d center1;
  Eigen::Vector3d center2;
  if(!nh.getParam("center1", center1vec)){ROS_ERROR("Param center1 not found");}
  if(!nh.getParam("center2", center2vec)){ROS_ERROR("Param center2 not found");}
  center1 << center1vec[0], center1vec[1], center1vec[2];
  center2 << center2vec[0], center2vec[1], center2vec[2];

  double x_reach;
  double y_reach;
  double x_offset;
  double y_offset;
  if(!nh.getParam("hittable/reach/x", x_reach)){ROS_ERROR("Param hittable/reach/x not found");}
  if(!nh.getParam("hittable/reach/y", y_reach)){ROS_ERROR("Param hittable/reach/y not found");}
  if(!nh.getParam("hittable/offset/x", x_offset)){ROS_ERROR("Param hittable/offset/x not found");}
  if(!nh.getParam("hittable/offset/y", y_offset)){ROS_ERROR("Param hittable/offset/y not found");}
  Eigen::Vector4d hittable_params = {x_reach, y_reach, x_offset, y_offset};


  //Object properties from param file
  struct modelProperties {
    double size_x;
    double size_y;
    double size_z;
    double com_x;
    double com_y;
    double com_z;
    double mass;
    double mu;
    double mu2;
    double izz;
  };
  struct modelProperties box;
    nh.getParam("box/properties/size/x", box.size_x);
    nh.getParam("box/properties/size/y", box.size_y);
    nh.getParam("box/properties/size/z", box.size_z);
    nh.getParam("box/properties/COM/x", box.com_x);
    nh.getParam("box/properties/COM/y", box.com_y);
    nh.getParam("box/properties/COM/z", box.com_z);
    nh.getParam("box/properties/mass", box.mass);
    nh.getParam("box/properties/mu", box.mu);
    nh.getParam("box/properties/mu2", box.mu2);
  box.izz = 1.0/12* box.mass* (pow(box.size_x,2) + pow(box.size_y,2));
  

  R_Opti << 1.0, 0.0, 0.0, 0.0,1.0, 0.0, 0.0, 0.0, 1.0;
  
  

  //Storing data to get the right values in time. The data we want is actually from right before events that we can recognize (e.g. speed right before hitting)
    std::queue<ros::Time> time_q;
    std::queue<ros::Time> pred_time_q;
    std::queue<Eigen::Vector3d> object_pos_q;
    std::queue<Eigen::Vector3d> object_vel_q;
    std::queue<Eigen::Vector3d> predict_pos_q;
    std::queue<double> object_theta_q;
    std::queue<double> object_theta_dot_q;
    std::queue<geometry_msgs::Pose> ee1_pose_q;
    std::queue<geometry_msgs::Pose> ee2_pose_q;
    std::queue<Eigen::Vector3d> ee1_rpy_q;
    std::queue<Eigen::Vector3d> ee2_rpy_q;
    std::queue<geometry_msgs::Twist> ee1_twist_q;
    std::queue<geometry_msgs::Twist> ee2_twist_q;
    std::queue<std::vector<double>> iiwa1_joint_angles_q;
    std::queue<std::vector<double>> iiwa2_joint_angles_q;

    bool once11 = true;
    bool once12 = true;
    bool once21 = false;
    bool once22 = false;
    bool tracking = false;
    int oneinten;
    std::stringstream data_path;
    data_path << ros::package::getPath("i_am_project") << "/data/collect/object_data.csv";
    std::ofstream object_data;
  //


  while(ros::ok()){
 
    object_data.open(data_path.str(), std::ofstream::out | std::ofstream::app);
    if(!object_data.is_open()){
      std::cerr << "Error opening output file.\n";
      std::cout << "Current path is " << std::experimental::filesystem::current_path() << '\n';
    }

    bool hitta1, hitta2, farra1, farra2;
    std::tie(hitta1, farra1) = hittable(object_pos, center1, center2, hittable_params);
    std::tie(hitta2, farra2) = hittable(object_pos, center2, center1, hittable_params);

    // Reset object position once out of reach if it is in gazebo (otherwise, there is little our code can do for us)
    if (object_real == false && object_vel.norm()<0.01 && (!hitta1 && !hitta2)) {

      tracking = false;
      if (once11 == false){
        once12 = false;
        tracking = false;
        object_data << "end, reset" << "\n\n\n";
        once21 = true;
      }
      if (once21 == false){
        once22 = false;
        tracking = false;
        object_data << "end, reset" << "\n\n\n";
        once11 = true;
      }

      ros::Duration(0.5).sleep();
    }


    ros::Time current_time = ros::Time::now();
    //Store the actual data for 3 time steps. This turns out to be the right time. In other words, post-hit is initiated 3 time steps after the step right before impact, which is the step we want data from.
    time_q.push(current_time);
    pred_time_q.push(current_time);
    object_pos_q.push(object_pos);
    object_vel_q.push(object_vel);
    predict_pos_q.push(predict_pos);
    object_theta_q.push(object_th);
    object_theta_dot_q.push(object_th_dot);
    ee1_pose_q.push(ee1_pose);
    ee2_pose_q.push(ee2_pose);
    ee1_rpy_q.push(ee1_rpy);
    ee2_rpy_q.push(ee2_rpy);
    ee1_twist_q.push(ee1_twist);
    ee2_twist_q.push(ee2_twist);
    iiwa1_joint_angles_q.push(iiwa1_joint_angles);
    iiwa2_joint_angles_q.push(iiwa2_joint_angles);


    while (time_q.size() > 4){time_q.pop();}
    while (object_pos_q.size() > 4){object_pos_q.pop();}
    while (object_vel_q.size() > 4){object_vel_q.pop();}
    while (pred_time_q.size() > 104){pred_time_q.pop();}
    while (predict_pos_q.size() > 104){predict_pos_q.pop();}
    while (object_theta_q.size() > 4){object_theta_q.pop();}
    while (object_theta_dot_q.size() > 4){object_theta_dot_q.pop();}
    while (ee1_pose_q.size() > 4){ee1_pose_q.pop();}
    while (ee2_pose_q.size() > 4){ee2_pose_q.pop();}
    while (ee1_rpy_q.size() > 4){ee1_rpy_q.pop();}
    while (ee2_rpy_q.size() > 4){ee2_rpy_q.pop();}
    while (ee1_twist_q.size() > 4){ee1_twist_q.pop();}
    while (ee2_twist_q.size() > 4){ee2_twist_q.pop();}
    while (iiwa1_joint_angles_q.size() > 4){iiwa1_joint_angles_q.pop();}
    while (iiwa2_joint_angles_q.size() > 4){iiwa2_joint_angles_q.pop();}


    

    if (mode1 == 4 && once11 == true){  //store data right before hit (this is once, directly after hit ; mode 4 --> post hit)
      once11 = false;
      once12 = true;
      object_data << "pre_post_time,    " << time_q.front()<< ", " << current_time.toSec()<< "\n";
      object_data << "box_properties,   " << box.size_x << ", " << box.size_y << ", " << box.size_z << ", " << box.com_x << ", " << box.com_y << ", " << box.com_z << ", " << box.mass << ", " << box.mu << ", " << box.mu2<< ", " << box.izz << "\n";
      object_data << "pre_hit_joints,   " << iiwa1_joint_angles_q.front()[0] << ", " << iiwa1_joint_angles_q.front()[1] << ", " << iiwa1_joint_angles_q.front()[2] << ", " << iiwa1_joint_angles_q.front()[3] << ", " << iiwa1_joint_angles_q.front()[4] << ", " << iiwa1_joint_angles_q.front()[5] << ", " << iiwa1_joint_angles_q.front()[6] << "\n";
      object_data << "post_hit_joints,  " << iiwa1_joint_angles[0] << ", " << iiwa1_joint_angles[1] << ", " << iiwa1_joint_angles[2] << ", " << iiwa1_joint_angles[3] << ", " << iiwa1_joint_angles[4] << ", " << iiwa1_joint_angles[5] << ", " << iiwa1_joint_angles[6] << "\n";
      object_data << "pre_hit_ee,       " << ee1_pose_q.front().position.x << ", " << ee1_pose_q.front().position.y << ", " << ee1_pose_q.front().position.z << ", " << ee1_pose_q.front().orientation.x << ", " << ee1_pose_q.front().orientation.y << ", " << ee1_pose_q.front().orientation.z << ", " << ee1_pose_q.front().orientation.w << ", " << ee1_rpy_q.front()[0] << ", " << ee1_rpy_q.front()[1] << ", " << ee1_rpy_q.front()[2] << ", " << ee1_twist_q.front().linear.x << ", " << ee1_twist_q.front().linear.y << ", " << ee1_twist_q.front().linear.z << ", " << ee1_twist_q.front().angular.x << ", " << ee1_twist_q.front().angular.y << ", " << ee1_twist_q.front().angular.z << "\n";
      object_data << "post_hit_ee,      " << ee1_pose.position.x << ", " << ee1_pose.position.y << ", " << ee1_pose.position.z << ", " << ee1_pose.orientation.x << ", " << ee1_pose.orientation.y << ", " << ee1_pose.orientation.z << ", " << ee1_pose.orientation.w << ", " << ee1_rpy[0] << ", " << ee1_rpy[1] << ", " << ee1_rpy[2] << ", " << ee1_twist.linear.x << ", " << ee1_twist.linear.y << ", " << ee1_twist.linear.z << ", " << ee1_twist.angular.x << ", " << ee1_twist.angular.y << ", " << ee1_twist.angular.z << "\n";
      object_data << "pred_stop_pos,    " << pred_time_q.front()<< ", " << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n";
      object_data << "pre_hit_object,   " << time_q.front()<< ", "<< object_pos_q.front()[0] << ", " << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << ", " << object_theta_q.front() << ", "<< object_vel_q.front()[0] << ", " << object_vel_q.front()[1] << ", " << object_vel_q.front()[2] << ", " << object_theta_dot_q.front()<< "\n";
      object_data << "post_hit_object,  " << current_time.toSec() << ", " << object_pos[0] << ", " << object_pos[1] << ", " << object_pos[2] << ", " << object_th << ", "<< object_vel[0] << ", " << object_vel[1] << ", " << object_vel[2] << ", " << object_th_dot<< "\n";
      object_data << "post_hit_trajectory:\n";
      tracking = true;
      oneinten = 10;
    }

    if (mode2 == 2 && object_pos[1] > 0.1 && once12 == true){  // it will be stopped before reaching the end position
      once12 = false;
      tracking = false;
      object_data << "end, stopped" << "\n\n\n";
      //object_data << "pred_stop_pos,   " << pred_time_q.front()<< ", " << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n";
      once21 = true;
    }
    if (mode2 == 3 && once12 == true){  //or if it will be hit back just before it came to a halt
      once12 = false;
      tracking = false;
      object_data << current_time.toSec() << ", " << object_pos[0] << ", " << object_pos[1] << ", " << object_pos[2] << ", " << object_th << ", "<< object_vel[0] << ", " << object_vel[1] << ", " << object_vel[2] << ", " << object_th_dot<< "\n";
      object_data << "end, free m3, "<< "\n\n\n";
      once21 = true;
    }
    
    if (mode2 == 4 && once21 == true){
      once21 = false;
      once22 = true;
      object_data << "pre_post_time,    " << time_q.front()<< ", " << current_time.toSec()<< "\n";
      object_data << "box_properties,   " << box.size_x << ", " << box.size_y << ", " << box.size_z << ", " << box.com_x << ", " << box.com_y << ", " << box.com_z << ", " << box.mass << ", " << box.mu << ", " << box.mu2<< ", " << box.izz << "\n";
      object_data << "pre_hit_joints,   " << iiwa2_joint_angles_q.front()[0] << ", " << iiwa2_joint_angles_q.front()[1] << ", " << iiwa2_joint_angles_q.front()[2] << ", " << iiwa2_joint_angles_q.front()[3] << ", " << iiwa2_joint_angles_q.front()[4] << ", " << iiwa2_joint_angles_q.front()[5] << ", " << iiwa2_joint_angles_q.front()[6] << "\n";
      object_data << "post_hit_joints,  " << iiwa2_joint_angles[0] << ", " << iiwa2_joint_angles[1] << ", " << iiwa2_joint_angles[2] << ", " << iiwa2_joint_angles[3] << ", " << iiwa2_joint_angles[4] << ", " << iiwa2_joint_angles[5] << ", " << iiwa2_joint_angles[6] << "\n";
      object_data << "pre_hit_ee,       " << ee2_pose_q.front().position.x << ", " << ee2_pose_q.front().position.y << ", " << ee2_pose_q.front().position.z << ", " << ee2_pose_q.front().orientation.x << ", " << ee2_pose_q.front().orientation.y << ", " << ee2_pose_q.front().orientation.z << ", " << ee2_pose_q.front().orientation.w << ", " << ee2_rpy_q.front()[0] << ", " << ee2_rpy_q.front()[1] << ", " << ee2_rpy_q.front()[2] << ", " << ee2_twist_q.front().linear.x << ", " << ee2_twist_q.front().linear.y << ", " << ee2_twist_q.front().linear.z << ", " << ee2_twist_q.front().angular.x << ", " << ee2_twist_q.front().angular.y << ", " << ee2_twist_q.front().angular.z << "\n";
      object_data << "post_hit_ee,      " << ee2_pose.position.x << ", " << ee2_pose.position.y << ", " << ee2_pose.position.z << ", " << ee2_pose.orientation.x << ", " << ee2_pose.orientation.y << ", " << ee2_pose.orientation.z << ", " << ee2_pose.orientation.w << ", " << ee2_rpy[0] << ", " << ee2_rpy[1] << ", " << ee2_rpy[2] << ", " << ee2_twist.linear.x << ", " << ee2_twist.linear.y << ", " << ee2_twist.linear.z << ", " << ee2_twist.angular.x << ", " << ee2_twist.angular.y << ", " << ee2_twist.angular.z << "\n";
      object_data << "pred_stop_pos,    " << pred_time_q.front()<< ", " << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n";
      object_data << "pre_hit_object,   " << time_q.front()<< ", "<< object_pos_q.front()[0] << ", " << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << ", " << object_theta_q.front() << ", "<< object_vel_q.front()[0] << ", " << object_vel_q.front()[1] << ", " << object_vel_q.front()[2] << ", " << object_theta_dot_q.front()<< "\n";
      object_data << "post_hit_object,  " << current_time.toSec() << ", " << object_pos[0] << ", " << object_pos[1] << ", " << object_pos[2] << ", " << object_th << ", "<< object_vel[0] << ", " << object_vel[1] << ", " << object_vel[2] << ", " << object_th_dot<< "\n";
      object_data << "post_hit_trajectory:\n";
      tracking = true;
      oneinten = 10;
    }
    if (mode1 == 2 && object_pos[1] < -0.1 && once22 == true){
      once22 = false;
      tracking = false;
      object_data << "end, stopped" << "\n\n\n";
      //object_data << "pred_stop_pos,   " << pred_time_q.front()<< ", " << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n";
      once11 = true;
    }
    if (mode1 == 3 && once22 == true){
      once22 = false;
      tracking = false;
      object_data << current_time.toSec() << ", " << object_pos[0] << ", " << object_pos[1] << ", " << object_pos[2] << ", " << object_th << ", "<< object_vel[0] << ", " << object_vel[1] << ", " << object_vel[2] << ", " << object_th_dot<< "\n";
      object_data << "end, free m3, " << "\n\n\n";
      once11 = true;
    }
    if (tracking == true && oneinten >= 10){
      oneinten = 0;
      object_data << current_time.toSec() << ", " << object_pos[0] << ", " << object_pos[1] << ", " << object_pos[2] << ", " << object_th << ", "<< object_vel[0] << ", " << object_vel[1] << ", " << object_vel[2] << ", " << object_th_dot<< "\n";
      if (object_vel.norm()<0.05){
        tracking = false;
        if (once11 == false){
          once12 = false;
          tracking = false;
          object_data << "end, free v=0, "<< current_time.toSec() << "\n\n\n";
          once21 = true;
          once22 = true;
        }
        if (once21 == false){
          once22 = false;
          tracking = false;
          object_data << "end, free v=0, "<< current_time.toSec() << "\n\n\n";
          once11 = true;
          once12 = true;
        }
      }
    }
    oneinten++;

  
    object_data.close();
    
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
