//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "../include/1v1_master.h"

#include "../include/track.h"
#include "../include/block.h"
#include "../include/hit_DS.h"
#include "../include/post_hit.h"
#include "../include/rest.h"

#include "../include/kalman.h"

#include <experimental/filesystem>

using namespace std;


double des_speed;
double theta;

int mode1 = 5;
int mode2 = 5;
int manual_mode1 = 5;
int manual_mode2 = 5;

geometry_msgs::Pose box_pose, ee1_pose, ee2_pose, iiwa1_base_pose, iiwa2_base_pose;
geometry_msgs::Twist box_twist, ee1_twist, ee2_twist;

Eigen::Vector3d object_pos, object_vel, ee1_pos, ee2_pos, ee1_vel, ee2_vel, iiwa1_base_pos, iiwa2_base_pos;
Eigen::Vector3d object_rpy;
double object_theta;
double theta_mod;
Eigen::Vector4d theta_quat;

std::vector<double> iiwa1_joint_angles, iiwa2_joint_angles;

Eigen::Vector3d rest1_pos = {0.5, -0.2, 0.4};           //relative to iiwa base
Eigen::Vector3d rest2_pos = {0.5, -0.2, 0.4};
Eigen::Vector4d rest1_quat = {0.707, -0.707, 0.0, 0.0};
Eigen::Vector4d rest2_quat = {0.707, -0.707, 0.0, 0.0};

Eigen::Vector3d ee_offset = {0.0, -0.4, 0.225};         //relative to object

Eigen::Vector3d des_pos = {0.0, 0.8, 0.0};              // Seen from IIWA 1, relative to world base. Same coordinate is used for iiwa 2, flipped 180 deg

//State estimation initialization
Eigen::Matrix<double, 6,6> P = Eigen::Matrix<double, 6,6>::Zero();
Eigen::Matrix<double, 6,1> state = Eigen::Matrix<double, 6,1>::Zero();
//Eigen::Matrix<double, 9,9> P = Eigen::Matrix<double, 9,9>::Zero();  //use for experimental kalman2
//Eigen::Matrix<double, 9,1> state = Eigen::Matrix<double, 9,1>::Zero();
Eigen::Vector3d predict_pos;
double ETA;


Eigen::Vector4d quatProd(Eigen::Vector4d a, Eigen::Vector4d b){
  Eigen::Vector4d p;
  p[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  p[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  p[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  p[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
  return p;
}

Eigen::Vector3d quatToRPY(Eigen::Vector4d q) {
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Eigen::Vector4d rpyToQuat(double r, double p, double y){
  Eigen::Vector4d q_base = {std::cos(-M_PI/4), std::sin(-M_PI/4), 0.0, 0.0}; //rotate from base quaternion (ee pointing up) to 'ready to play' rotation

  Eigen::Vector4d q_r = {std::cos(r/2), std::sin(r/2), 0.0, 0.0};
  Eigen::Vector4d q_p = {std::cos(p/2), 0.0, std::sin(p/2), 0.0};
  Eigen::Vector4d q_y = {std::cos(y/2), 0.0, 0.0, std::sin(y/2)};

  Eigen::Vector4d q = quatProd(q_r, quatProd(q_p, quatProd(q_y,q_base)));     //start with last rotation and then backwards

  return q;
}


int getIndex(std::vector<std::string> v, std::string value){
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void objectCallback(const gazebo_msgs::ModelStates model_states){
  int box_index = getIndex(model_states.name, "my_box");

  box_pose = model_states.pose[box_index];
  box_twist = model_states.twist[box_index];

  object_pos << box_pose.position.x, box_pose.position.y, box_pose.position.z;
  object_vel << box_twist.linear.x, box_twist.linear.y, box_twist.linear.z;

  object_rpy = quatToRPY({box_pose.orientation.w, box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z}); //get orientation in rpy
  object_theta = object_rpy[2];                                                                                             //only the z-axis
  theta_mod = std::fmod(object_theta+M_PI+M_PI/4,M_PI/2)-M_PI/4;                                                            //get relative angle of box face facing the arm
  theta_quat = rpyToQuat(0.0, 0.0, theta_mod);                                                                                    //convert back to quat
}

void iiwaCallback(const gazebo_msgs::LinkStates link_states){
  int ee1_index = getIndex(link_states.name, "iiwa1::iiwa1_link_7"); // End effector is the 7th link in KUKA IIWA
  int ee2_index = getIndex(link_states.name, "iiwa2::iiwa2_link_7");
  int iiwa1_base_index = getIndex(link_states.name, "iiwa1::iiwa1_link_0");
  int iiwa2_base_index = getIndex(link_states.name, "iiwa2::iiwa2_link_0");

  ee1_pose = link_states.pose[ee1_index];
  ee2_pose = link_states.pose[ee2_index];
  ee1_twist = link_states.twist[ee1_index];
  ee2_twist = link_states.twist[ee2_index];

  ee1_pos << ee1_pose.position.x, ee1_pose.position.y, ee1_pose.position.z;
  ee2_pos << ee2_pose.position.x, ee2_pose.position.y, ee2_pose.position.z;
  ee1_vel << ee1_twist.linear.x, ee1_twist.linear.y, ee1_twist.linear.z;
  ee2_vel << ee2_twist.linear.x, ee2_twist.linear.y, ee2_twist.linear.z;
  

  iiwa1_base_pose = link_states.pose[iiwa1_base_index];
  iiwa2_base_pose = link_states.pose[iiwa2_base_index];
  iiwa1_base_pos << iiwa1_base_pose.position.x,iiwa1_base_pose.position.y,iiwa1_base_pose.position.z;
  iiwa2_base_pos << iiwa2_base_pose.position.x,iiwa2_base_pose.position.y,iiwa2_base_pose.position.z;
}

void iiwa1JointCallback(sensor_msgs::JointState joint_states){
  iiwa1_joint_angles = joint_states.position;
}

void iiwa2JointCallback(sensor_msgs::JointState joint_states){
  iiwa2_joint_angles = joint_states.position;
}


void modeCallback(std_msgs::Int16 msg){
  if (msg.data <= 5){manual_mode1 = msg.data;}
  if (msg.data >= 6){manual_mode2 = msg.data-5;} 
  if (msg.data == 0){manual_mode2 = 5;}
}

int modeSelektor(Eigen::Vector3d predict_pos, double ETA, Eigen::Vector3d ee_pos, const int prev_mode, const int iiwa_no){
  int mode = prev_mode;           // if none of the conditions are met, mode remains the same
  
  int iiwa_sel = 3-2*iiwa_no;        // multiplier used in conditional statements. iiwa = 1 -> selector = 1, iiwa = 2 -> selector = -1. This allows for directions to flip
  Eigen::Matrix3d sel_mat;
  sel_mat << iiwa_sel, 0.0,      0.0,
             0.0,      iiwa_sel, 0.0,
             0.0,      0.0,      1.0;
  

  
  bool cur_hittable;
  if (object_pos[1]*iiwa_sel < -0.5 && object_pos[1]*iiwa_sel > -0.9){cur_hittable = true;}
  else {cur_hittable = false;}

  bool pred_hittable;
  if (predict_pos[1]*iiwa_sel < -0.5 && predict_pos[1]*iiwa_sel > -0.9){pred_hittable = true;}
  else {pred_hittable = false;}

  bool too_far;
  if (predict_pos[1]*iiwa_sel < -0.9){too_far = true;}
  else {too_far = false;}

  bool too_close;
  if (predict_pos[1]*iiwa_sel > -0.5){too_close = true;}
  else {too_close = false;}

  bool moving;
  if (object_vel.norm() > 0.03){moving = true;}
  else {moving = false;}

  bool towards;
  if (object_vel[1]*iiwa_sel < -0.01){towards = true;}
  else {towards = false;}

  bool ee_ready;
  if ((ee_pos-(predict_pos+sel_mat*ee_offset)).norm() < 0.1){ee_ready = true;}
  else {ee_ready = false;}



  switch (prev_mode){
    case 1:   //track
      if (too_far == true && ETA < 3) {mode = 2;}                                         //if object will go too far, try to stop it
      if (pred_hittable == true && ETA < 0.5 && ee_ready == true) {mode = 3;}             //if object will be in feasible position and stops in 0.5s and ee is in correct position, go to hit
      if (too_close == true && ETA < 3) {mode = 5;}                                       //if object will not make it into reach, give up and go to rest
      break;

    case 2:   //stop
      if (towards == false || pred_hittable == true) {mode = 1;}                            //if the object no longer moves towards, it has been stopped succesfully so go to track for correct ee
    
    case 3:   //hit
      if (towards == false && moving == true) {mode = 4;                                    //if object starts moving because it is hit, go to post hit and initialize kalman
        state = Eigen::Matrix<double,6,1>::Zero();
        state << object_vel[0],object_vel[1],object_vel[2],0.0,0.03*9.81*0.3*iiwa_sel,0.0;
        P = Eigen::Matrix<double, 6,6>::Zero();
        //state = Eigen::Matrix<double,9,1>::Zero();
        //state << object_pos[0],object_pos[1],object_pos[2],object_vel[0],object_vel[1],object_vel[2],0.0,0.03*9.81*0.3*iiwa_sel,0.0;
        //P = Eigen::Matrix<double, 9,9>::Zero();
      }                                           
      break;

    case 4:   //post hit
      if (cur_hittable == false || towards == false) {mode = 5;}                            //if object has left the range of arm, go to rest
      break;

    case 5:   //rest
      if (too_far == true && ETA < 3) {mode = 2;}                               //if object will go too far, try to stop it
      if (pred_hittable == true && ETA < 0.5 && ee_ready == true) {mode = 3;}   //same as when being in tracking mode, since mode is initialized in rest
      if (pred_hittable == true && ETA < 3 && ee_ready == false) {mode = 1;}    //if object is going to be hittable but ee is not in the right position, lets track!                               
      break;
  }
  return mode;
}




int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "AH_master");
  ros::NodeHandle nh;
  ros::Rate rate(100);

  //Subscribers to object and IIWA states
  ros::Subscriber object_subs = nh.subscribe("/gazebo/model_states", 100 , objectCallback);
  ros::Subscriber iiwa_subs = nh.subscribe("/gazebo/link_states", 100, iiwaCallback);
  ros::Subscriber iiwa1_joint_subs = nh.subscribe("/iiwa1/joint_states", 100, iiwa1JointCallback);
  ros::Subscriber iiwa2_joint_subs = nh.subscribe("/iiwa2/joint_states", 100, iiwa2JointCallback);
  
  //Publishers for position and velocity commands for IIWA end-effectors
  ros::Publisher pub_vel_quat1 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/vel_quat", 1);
  ros::Publisher pub_pos_quat1 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/pos_quat", 1);
  ros::Publisher pub_vel_quat2 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/vel_quat", 1);
  ros::Publisher pub_pos_quat2 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/pos_quat", 1);

  //Client to reset object pose
  ros::service::waitForService("gazebo/set_model_state");
  ros::ServiceClient set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  
  ros::Subscriber mode_sub = nh.subscribe("mode",10,modeCallback);
  bool manual_mode;
  nh.getParam("manual_mode", manual_mode);

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

  
  //Set hitting speed and direction. Once we change attractor to something more related to the game, we no longer need this
    std::cout << "Enter the desired speed of hitting (between 0 and 1)" << std::endl;
    std::cin >> des_speed;

    while(des_speed < 0 || des_speed > 1){
      std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
      std::cin >> des_speed;
    }

    std::cout << "Enter the desired direction of hitting (between -pi/2 and pi/2)" << std::endl;
    std::cin >> theta;

    while(theta < -M_PI_2 || theta > M_PI_2){
      std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
      std::cin >> theta;
  }



  
  int prev_mode1 = mode1;
  int prev_mode2 = mode2;

  Eigen::Vector3d object_pos_init1 = object_pos;
  Eigen::Vector3d object_pos_init2 = object_pos;
  Eigen::Vector3d ee1_pos_init = ee1_pos;
  Eigen::Vector3d ee2_pos_init = ee2_pos;


  //Storing data to get the right values in time. The data we want is actually from right before events that we can recognize (e.g. speed right before hitting)
  std::queue<Eigen::Vector3d> object_pos_q;
  std::queue<Eigen::Vector3d> predict_pos_q;
  std::queue<double> object_theta_q;
  std::queue<geometry_msgs::Pose> ee1_pose_q;
  std::queue<geometry_msgs::Pose> ee2_pose_q;
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
  data_path << ros::package::getPath("i_am_project") << "/data/hitting/object_data.csv";
  std::ofstream object_data;
  bool store_data = 0;




  while(ros::ok()){
    // Predict the future
    tie(P, state, predict_pos, ETA) = predictPos(P, state, object_pos, object_vel, box.mass);
    
    double theta_des1 = -std::atan2((des_pos[0]-predict_pos[0]),(des_pos[1]-predict_pos[1]));
    double theta_des2 = -std::atan2((des_pos[0]+predict_pos[0]),(des_pos[1]+predict_pos[1]));

    // Select correct operating mode for both arms based on conditions on (predicted) object position and velocity and ee position
    if (manual_mode == false) {mode1 = modeSelektor(predict_pos, ETA, ee1_pos, prev_mode1, 1);}
    else {mode1 = manual_mode1;}
    switch (mode1) {
      case 1: //track
        pub_pos_quat1.publish(track(predict_pos, rest1_quat, ee_offset, iiwa1_base_pos, 1));
        ee1_pos_init = ee1_pos; //we can go from track to hit. For hit, we need initial
        break;
      case 2: //stop
        pub_pos_quat1.publish(block(predict_pos, theta_quat, iiwa1_base_pos, 1));
        break;
      case 3: //hit
        pub_vel_quat1.publish(hitDS(des_speed, theta_des1, object_pos, ee1_pos, ee1_pos_init, 1));
        object_pos_init1 = object_pos; //need this for post-hit to guide the arm right after hit
        break;
      case 4: //post hit
        pub_pos_quat1.publish(postHit(object_pos_init1, rest1_quat, iiwa1_base_pos, 1));
        break;
      case 5: //rest
        pub_pos_quat1.publish(rest(rest1_pos, rest1_quat));
        ee1_pos_init = ee1_pos; //we can also go from rest to hit..
        break;
    }
    prev_mode1 = mode1;

    if (manual_mode == false) {mode2 = modeSelektor(predict_pos, ETA, ee2_pos, prev_mode2, 2);}
    else {mode2 = manual_mode2;}
    switch (mode2) {
      case 1: //track
        pub_pos_quat2.publish(track(predict_pos, rest2_quat, ee_offset, iiwa2_base_pos, 2));
        ee2_pos_init = ee2_pos;
        break;
      case 2: //stop
        pub_pos_quat2.publish(block(predict_pos, theta_quat, iiwa2_base_pos, 2));
        break;
      case 3: //hit
        pub_vel_quat2.publish(hitDS(des_speed, theta_des2, object_pos, ee2_pos, ee2_pos_init, 2));
        object_pos_init2 = object_pos;
        break;
      case 4: //post hit
        pub_pos_quat2.publish(postHit(object_pos_init2, rest2_quat, iiwa2_base_pos, 2));
        break;
      case 5: //rest
        pub_pos_quat2.publish(rest(rest2_pos, rest2_quat));
        ee2_pos_init = ee2_pos;
        break;
    }
    prev_mode2 = mode2;
    






    // Reset object position once out of reach
    if (object_vel.norm()<0.01 && (object_pos[1] < -0.9 || (object_pos[1] > -0.5 && object_pos[1] < 0.5) || object_pos[1] > 0.9)) {
      //Set new pose of box
      geometry_msgs::Pose new_box_pose;
      nh.getParam("box/initial_pos/x",new_box_pose.position.x);
      nh.getParam("box/initial_pos/y",new_box_pose.position.y);
      nh.getParam("box/initial_pos/z",new_box_pose.position.z);
      new_box_pose.orientation.x = 0.0;
      new_box_pose.orientation.y = 0.0;
      new_box_pose.orientation.z = 0.0;
      new_box_pose.orientation.w = 0.0;

      gazebo_msgs::ModelState modelstate;
      modelstate.model_name = (std::string) "my_box";
      modelstate.reference_frame = (std::string) "world";
      modelstate.pose = new_box_pose;
      setmodelstate.request.model_state = modelstate;
      set_state_client.call(setmodelstate);


      //Set new pose of minibox
      geometry_msgs::Pose new_mini_pose;
      nh.getParam("mini/initial_pos/x", new_mini_pose.position.x);
      nh.getParam("mini/initial_pos/y", new_mini_pose.position.y);
      nh.getParam("mini/initial_pos/z", new_mini_pose.position.z);
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
      set_state_client.call(setmodelstate);

      ROS_INFO("Resetting object pose");
    } 



    // Some infos
      std::stringstream ss1;
      std::stringstream ss2;

      ss1 << "mode1: " << mode1;
      ss2 << "mode2: " << mode2;

      ROS_INFO("%s",ss1.str().c_str());
      ROS_INFO("%s",ss2.str().c_str());

      std::stringstream ss3;
      std::stringstream ss4;
      std::stringstream ss5;

      ss3 << "final  : " << predict_pos[1];
      ss4 << "current: " << object_pos[1];
      ss5 << "ETA  : " << ETA*5.0;

      ROS_INFO("%s",ss3.str().c_str());
      ROS_INFO("%s",ss4.str().c_str());
    ROS_INFO("%s",ss5.str().c_str());

    



    //Store the actual data for three time steps. This turns out to be the right time. In other words, post-hit is initiated three time steps after the step right before impact, which is the step we want data from.
    object_pos_q.push(object_pos);
    predict_pos_q.push(predict_pos);
    object_theta_q.push(object_theta);
    ee1_pose_q.push(ee1_pose);
    ee2_pose_q.push(ee2_pose);
    ee1_twist_q.push(ee1_twist);
    ee2_twist_q.push(ee2_twist);
    iiwa1_joint_angles_q.push(iiwa1_joint_angles);
    iiwa2_joint_angles_q.push(iiwa2_joint_angles);

    while (object_pos_q.size() > 3){object_pos_q.pop();}
    while (predict_pos_q.size() > 3){predict_pos_q.pop();}
    while (object_theta_q.size() > 3){object_theta_q.pop();}
    while (ee1_pose_q.size() > 3){ee1_pose_q.pop();}
    while (ee2_pose_q.size() > 3){ee2_pose_q.pop();}
    while (ee1_twist_q.size() > 3){ee1_twist_q.pop();}
    while (ee2_twist_q.size() > 3){ee2_twist_q.pop();}
    while (iiwa1_joint_angles_q.size() > 3){iiwa1_joint_angles_q.pop();}
    while (iiwa2_joint_angles_q.size() > 3){iiwa2_joint_angles_q.pop();}


    object_data.open(data_path.str(), std::ofstream::out | std::ofstream::app);
    if(!object_data.is_open()){
      std::cerr << "Error opening output file.\n";
      std::cout << "Current path is " << std::experimental::filesystem::current_path() << '\n';
    }

    if (mode1 == 4 && once11 == true){  //store data right before hit (this is once, directly after hit)
      once11 = false;
      object_data << "box_properties,  " << box.size_x << ", " << box.size_y << ", " << box.size_z << ", " << box.com_x << ", " << box.com_y << ", " << box.com_z << ", " << box.mass << ", " << box.mu << ", " << box.mu2 << "\n";
      object_data << "pre_hit_joints,  " << iiwa1_joint_angles_q.front()[0] << ", " << iiwa1_joint_angles_q.front()[1] << ", " << iiwa1_joint_angles_q.front()[2] << ", " << iiwa1_joint_angles_q.front()[3] << ", " << iiwa1_joint_angles_q.front()[4] << ", " << iiwa1_joint_angles_q.front()[5] << ", " << iiwa1_joint_angles_q.front()[6] << "\n";
      object_data << "post_hit_joints, " << iiwa1_joint_angles[0] << ", " << iiwa1_joint_angles[1] << ", " << iiwa1_joint_angles[2] << ", " << iiwa1_joint_angles[3] << ", " << iiwa1_joint_angles[4] << ", " << iiwa1_joint_angles[5] << ", " << iiwa1_joint_angles[6] << "\n";
      object_data << "pre_hit_ee,      " << ee1_pose_q.front().position.x << ", " << ee1_pose_q.front().position.y << ", " << ee1_pose_q.front().position.z << ", " << ee1_pose_q.front().orientation.x << ", " << ee1_pose_q.front().orientation.y << ", " << ee1_pose_q.front().orientation.z << ", " << ee1_pose_q.front().orientation.w << ", " << ee1_twist_q.front().linear.x << ", " << ee1_twist_q.front().linear.y << ", " << ee1_twist_q.front().linear.z << ", " << ee1_twist_q.front().angular.x << ", " << ee1_twist_q.front().angular.y << ", " << ee1_twist_q.front().angular.z << "\n";
      object_data << "post_hit_ee,     " << ee1_pose.position.x << ", " << ee1_pose.position.y << ", " << ee1_pose.position.z << ", " << ee1_pose.orientation.x << ", " << ee1_pose.orientation.y << ", " << ee1_pose.orientation.z << ", " << ee1_pose.orientation.w << ", " << ee1_twist.linear.x << ", " << ee1_twist.linear.y << ", " << ee1_twist.linear.z << ", " << ee1_twist.angular.x << ", " << ee1_twist.angular.y << ", " << ee1_twist.angular.z << "\n";
      object_data << "pre_hit_object,  " << object_pos_q.front()[0] << ", " << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << ", " << object_theta_q.front() << "\n";
      object_data << "post_hit_trajectory:\n";
      tracking = true;
      oneinten = 10;
    }
    if (mode2 == 2 && object_pos[1] > 0.75 && once12 == true){  //store predicted pos if it will be stopped
      once12 = false;
      tracking = false;
      object_data << "end, stopped" << "\n";
      object_data << "pred_stop_pos, " << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n\n\n";
      once21 = true;
      once22 = true;
    }
    if (mode2 == 3 && once12 == true){  //or if it will be hit back just before it came to a halt
      once12 = false;
      tracking = false;
      object_data << "end, free" << "\n\n\n";
      once21 = true;
      once22 = true;
    }
    
    if (mode2 == 4 && once21 == true){
      once21 = false;
      object_data << "box_properties,  " << box.size_x << ", " << box.size_y << ", " << box.size_z << ", " << box.com_x << ", " << box.com_y << ", " << box.com_z << ", " << box.mass << ", " << box.mu << ", " << box.mu2 << "\n";
      object_data << "pre_hit_joints,  " << iiwa2_joint_angles_q.front()[0] << ", " << iiwa2_joint_angles_q.front()[1] << ", " << iiwa2_joint_angles_q.front()[2] << ", " << iiwa2_joint_angles_q.front()[3] << ", " << iiwa2_joint_angles_q.front()[4] << ", " << iiwa2_joint_angles_q.front()[5] << ", " << iiwa2_joint_angles_q.front()[6] << "\n";
      object_data << "post_hit_joints, " << iiwa2_joint_angles[0] << ", " << iiwa2_joint_angles[1] << ", " << iiwa2_joint_angles[2] << ", " << iiwa2_joint_angles[3] << ", " << iiwa2_joint_angles[4] << ", " << iiwa2_joint_angles[5] << ", " << iiwa2_joint_angles[6] << "\n";
      object_data << "pre_hit_ee,      " << ee2_pose_q.front().position.x << ", " << ee2_pose_q.front().position.y << ", " << ee2_pose_q.front().position.z << ", " << ee2_pose_q.front().orientation.x << ", " << ee2_pose_q.front().orientation.y << ", " << ee2_pose_q.front().orientation.z << ", " << ee2_pose_q.front().orientation.w << ", " << ee2_twist_q.front().linear.x << ", " << ee2_twist_q.front().linear.y << ", " << ee2_twist_q.front().linear.z << ", " << ee2_twist_q.front().angular.x << ", " << ee2_twist_q.front().angular.y << ", " << ee2_twist_q.front().angular.z << "\n";
      object_data << "post_hit_ee,     " << ee2_pose.position.x << ", " << ee2_pose.position.y << ", " << ee2_pose.position.z << ", " << ee2_pose.orientation.x << ", " << ee2_pose.orientation.y << ", " << ee2_pose.orientation.z << ", " << ee2_pose.orientation.w << ", " << ee2_twist.linear.x << ", " << ee2_twist.linear.y << ", " << ee2_twist.linear.z << ", " << ee2_twist.angular.x << ", " << ee2_twist.angular.y << ", " << ee2_twist.angular.z << "\n";
      object_data << "pre_hit_object,  " << object_pos_q.front()[0] << ", " << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << ", " << object_theta_q.front() << "\n";
      object_data << "post_hit_trajectory:\n";
      tracking = true;
      oneinten = 10;
    }
    if (mode1 == 2 && object_pos[1] < -0.75 && once22 == true){
      once22 = false;
      tracking = false;
      object_data << "end, stopped" << "\n";
      object_data << "pred_final_pos, " << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << "\n\n\n";
      once11 = true;
      once12 = true;
    }
    if (mode1 == 3 && once22 == true){
      once22 = false;
      tracking = false;
      object_data << "end, free" << "\n\n\n";
      once11 = true;
      once12 = true;
    }
    
    if (tracking == true && oneinten >= 10){
      oneinten = 0;
      object_data << object_pos[0] << ", " << object_pos[1] << ", " << object_pos[2] << ", " << object_theta << ", " << box_twist.angular.z << "\n";
    }
    oneinten++;

    object_data.close();
    
    /*std_msgs::Float32 posfloat;
    posfloat.data = (float) ee1_pos[1];
    std_msgs::Float32 velfloat;
    velfloat.data = (float) ee1_vel[1];
    pub_ee1_pos.publish(posfloat);
    pub_ee1_vel.publish(velfloat);*/





    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
