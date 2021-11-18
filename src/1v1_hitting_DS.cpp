//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer)
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "../include/hitting_DS.h" 
#include "../include/calculate_alpha.h"
#include "../include/nominal_DS_aux.h"
#include "../include/nominal_DS_main.h"
#include "../include/modulated_DS.h"
#include "../include/kalman.h"

#include <experimental/filesystem>

geometry_msgs::Pose box_pose, ee1_pose, ee2_pose, iiwa1_base_pose, iiwa2_base_pose;
geometry_msgs::Twist box_twist, ee1_twist, ee2_twist;

Eigen::Vector3d object_pos, object_vel, ee1_pos, ee2_pos, ee1_vel, ee2_vel, iiwa1_base_pos, iiwa2_base_pos;

double des_speed;
double theta;

Eigen::Vector3d rest1_pos = {0.5, -0.2, 0.4};           //relative to iiwa base
Eigen::Vector3d rest2_pos = {0.5, -0.2, 0.4};
Eigen::Vector4d rest1_quat = {-0.707, 0.0, 0.0, 0.707};
Eigen::Vector4d rest2_quat = {-0.707, 0.0, 0.0, 0.707};

Eigen::Vector3d ee_offset = {0.0, -0.4, 0.225};         //relative to object

Eigen::Matrix<double, 6,1> state = Eigen::Matrix<double, 6,1>::Zero();
Eigen::Matrix<double, 6,6> P = Eigen::Matrix<double, 6,6>::Zero();



using namespace std;

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



std::tuple<Eigen::Vector3d, double> predictPos(){
  P = covarUpdate(P);
  state = stateUpdate(object_vel,state,P);

  Eigen::Vector3d vel_est;
  Eigen::Vector3d f_est;
  double ETA;

  vel_est << state[0], state[1], state[2];
  f_est << state[3], state[4], state[5];
  ETA = 0.3*vel_est.norm()/f_est.norm();
  

  Eigen::Vector3d predict_pos;
  for (int i = 0; i < 3; i++) {
    predict_pos[i] = object_pos[i] - 0.5*0.3*vel_est[i]*vel_est[i]/f_est[i];
  }


  return std::make_tuple(predict_pos,ETA);
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
      if (pred_hittable == true && ETA < 1.0 && ee_ready == true) {mode = 3;}             //if object will be in feasible position and stops in 0.5s and ee is in correct position, go to hit
      if (too_close == true && ETA < 3) {mode = 5;}                                       //if object will not make it into reach, give up and go to rest
      break;

    case 2:   //stop
      if (towards == false || pred_hittable == true) {mode = 1;}                            //if the object no longer moves towards, it has been stopped succesfully so go to track for correct ee
    
    case 3:   //hit
      if (towards == false && moving == true) {mode = 4;                                    //if object starts moving because it is hit, go to post hit and initialize kalman
        state = Eigen::Matrix<double,6,1>::Zero();
        state << object_vel[0],object_vel[1],object_vel[2],0.0,0.03*9.81*0.3*iiwa_sel,0.0;
        P = Eigen::Matrix<double, 6,6>::Zero();
      }                                           
      break;

    case 4:   //post hit
      if (cur_hittable == false || towards == false) {mode = 5;}                            //if object has left the range of arm, go to rest
      break;

    case 5:   //rest
      if (too_far == true && ETA < 3) {mode = 2;}                               //if object will go too far, try to stop it
      if (pred_hittable == true && ETA < 1.0 && ee_ready == true) {mode = 3;}   //same as when being in tracking mode, since mode is initialized in rest
      if (pred_hittable == true && ETA < 3 && ee_ready == false) {mode = 1;}    //if object is going to be hittable but ee is not in the right position, lets track!                               
      break;
  }
  return mode;
}


geometry_msgs::Pose trackDS(Eigen::Vector3d predict_pos, Eigen::Vector4d rest_quat, Eigen::Vector3d iiwa_base_pos, const int iiwa_no){
  geometry_msgs::Pose pos_quat;
  int iiwa_sel = 3-2*iiwa_no;
  
  pos_quat.position.x = (predict_pos[0] - iiwa_base_pos[0])*iiwa_sel + ee_offset[0];
  pos_quat.position.y = (predict_pos[1] - iiwa_base_pos[1])*iiwa_sel + ee_offset[1];
  pos_quat.position.z = predict_pos[2] - iiwa_base_pos[2] + ee_offset[2];
  pos_quat.orientation.x = rest_quat[0];
  pos_quat.orientation.y = rest_quat[1];
  pos_quat.orientation.z = rest_quat[2];
  pos_quat.orientation.w = rest_quat[3];

  return pos_quat;
}


geometry_msgs::Pose stopDS(Eigen::Vector3d predict_pos, Eigen::Vector4d rest_quat, Eigen::Vector3d iiwa_base_pos, const int iiwa_no){
  geometry_msgs::Pose pos_quat;
  int iiwa_sel = 3-2*iiwa_no;

  pos_quat.position.x = (predict_pos[0] - iiwa_base_pos[0])*iiwa_sel;
  pos_quat.position.y = -1*iiwa_base_pos[1]*iiwa_sel -1.0;
  pos_quat.position.z = predict_pos[2];
  pos_quat.orientation.x = rest_quat[0];
  pos_quat.orientation.y = rest_quat[1];
  pos_quat.orientation.z = rest_quat[2];
  pos_quat.orientation.w = rest_quat[3];
  return pos_quat;
}


geometry_msgs::Pose hitDS(Eigen::Vector3d ee_pos, Eigen::Vector3d ee_pos_init, const int iiwa_no){
  geometry_msgs::Pose vel_quat;
  Eigen::Vector3d object_offset = {0.0, 0.0, 0.025};

  int iiwa_sel = 3-2*iiwa_no;
  Eigen::Matrix3d sel_mat;
  sel_mat << iiwa_sel, 0.0,      0.0,
             0.0,      iiwa_sel, 0.0,
             0.0,      0.0,      1.0;

  //instead of flipping direction of attractor and such, make use of the fact that iiwa2 is exactly the same as iiwa1 but rotated around the worlds z-axis with pi
  //if then the only parameters that are defined w.r.t. the world frame are rotated also, the whole situation becomes identical
  //(passive_track gives cmds w.r.t. respective iiwa frame)
  Eigen::Vector3d object_pos2  = sel_mat*(object_pos+object_offset); 
  Eigen::Vector3d ee_pos2      = sel_mat*ee_pos;
  Eigen::Vector3d ee_pos_init2 = sel_mat*ee_pos_init;


  

  Eigen::Matrix3d gain_main;
  Eigen::Matrix3d gain_aux;
  gain_main << -0.9,  0.0, 0.0,
                0.0, -0.1, 0.0,
                0.0, 0.0, -0.9;

  gain_aux << -0.1,  0.0, 0.0,
               0.0, -0.1, 0.0,
               0.0, 0.0, -0.1;

  double modulated_sigma = 3.0;



  Eigen::Matrix3d rot_mat;
  rot_mat << cos(theta), -sin(theta), 0, 
             sin(theta), cos(theta), 0,
             0, 0, 1;
  

  
  Eigen::Vector3d unit_x = {0.0, 1.0, 0.0};

  Eigen::Vector3d attractor_main;
  Eigen::Vector3d attractor_aux;
  
  attractor_main = object_pos2 + 1*rot_mat*unit_x;
  attractor_aux = (4.0/3.0)*object_pos2 - (1.0/3.0)*attractor_main;
  
  

  Eigen::Vector4d des_quat = Eigen::Vector4d::Zero();
  Eigen::Vector3d ee_direction_x;
  Eigen::Vector3d ee_direction_y;
  Eigen::Vector3d ee_direction_z;
  Eigen::Matrix3d rot_ee;
  Eigen::Quaterniond rot_quat;
  
  ee_direction_z = attractor_main - object_pos2; 
  ee_direction_x = {0.0, 0.0, 1.0};
  ee_direction_y = ee_direction_z.cross(ee_direction_x);
  ee_direction_z = ee_direction_z/ee_direction_z.norm();
  ee_direction_y = ee_direction_y/ee_direction_y.norm();
  ee_direction_x = ee_direction_x/ee_direction_x.norm();
  rot_ee.col(0) = ee_direction_x;
  rot_ee.col(1) = ee_direction_y;
  rot_ee.col(2) = ee_direction_z;
  rot_quat =  rot_ee;
  des_quat << rot_quat.x(), rot_quat.y(), rot_quat.z(), rot_quat.w();

  

  Eigen::Vector3d des_vel = {0.0, 0.0, 0.0};
  double alpha = calculate_alpha(ee_pos2, ee_pos_init2, object_pos2, attractor_main);
  
  des_vel = alpha*nominal_aux(rot_mat, gain_aux, ee_pos2, attractor_aux) + (1 - alpha)*nominal_main(rot_mat, gain_main, ee_pos2, attractor_main)
            + modulated_DS(attractor_main, object_pos2, ee_pos2, modulated_sigma);

  des_vel = des_vel*des_speed / des_vel.norm();


  vel_quat.position.x = des_vel(0);
  vel_quat.position.y = des_vel(1);
  vel_quat.position.z = des_vel(2);
  vel_quat.orientation.x = des_quat(0);
  vel_quat.orientation.y = des_quat(1);
  vel_quat.orientation.z = des_quat(2);
  vel_quat.orientation.w = des_quat(3);

  return vel_quat;
}


geometry_msgs::Pose postHit(const Eigen::Vector3d object_pos_init, const Eigen::Vector4d rest_quat, const geometry_msgs::Pose iiwa_base_pose, const int iiwa_no){
  geometry_msgs::Pose pos_quat;
  int iiwa_sel = 3-2*iiwa_no;
  pos_quat.position.x = (object_pos_init[0] - iiwa_base_pose.position.x)*iiwa_sel;
  pos_quat.position.y = (object_pos_init[1] - iiwa_base_pose.position.y)*iiwa_sel;
  pos_quat.position.z = object_pos_init[2] - iiwa_base_pose.position.z;
  pos_quat.orientation.x = rest_quat[0];
  pos_quat.orientation.y = rest_quat[1];
  pos_quat.orientation.z = rest_quat[2];
  pos_quat.orientation.w = rest_quat[3];
  return pos_quat;
}


geometry_msgs::Pose rest(const Eigen::Vector3d rest_pos, const Eigen::Vector4d rest_quat){
  geometry_msgs::Pose pos_quat;
  pos_quat.position.x = rest_pos[0];
  pos_quat.position.y = rest_pos[1];
  pos_quat.position.z = rest_pos[2];
  pos_quat.orientation.x = rest_quat[0];
  pos_quat.orientation.y = rest_quat[1];
  pos_quat.orientation.z = rest_quat[2];
  pos_quat.orientation.w = rest_quat[3];
  return pos_quat;
}







int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "hitting_DS");
  ros::NodeHandle nh;
  ros::Rate rate(100);

  ros::Subscriber object_subs = nh.subscribe("/gazebo/model_states", 100 , objectCallback);
  ros::Subscriber iiwa_subs = nh.subscribe("/gazebo/link_states", 100, iiwaCallback);
  
  
  ros::Publisher pub_vel_quat1 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/vel_quat", 1);
  ros::Publisher pub_pos_quat1 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/pos_quat", 1);
  ros::Publisher pub_vel_quat2 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/vel_quat", 1);
  ros::Publisher pub_pos_quat2 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/pos_quat", 1);

  ros::service::waitForService("gazebo/set_model_state");
  ros::ServiceClient set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  
  
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





  int mode1 = 5;
  int mode2 = 5;
  int prev_mode1 = mode1;
  int prev_mode2 = mode2;

  Eigen::Vector3d object_pos_init1 = object_pos;
  Eigen::Vector3d object_pos_init2 = object_pos;
  Eigen::Vector3d ee1_pos_init = ee1_pos;
  Eigen::Vector3d ee2_pos_init = ee2_pos;


  std::ofstream object_data;
  bool store_data = 0;

  while(ros::ok()){
    //object_position_from_source = world_to_base * (object_position_mocap - iiwa_base_position_from_source);
    //std::cout << "object is at: " << object_pos <<std::endl;

    Eigen::Vector3d predict_pos;
    double ETA;
    tie(predict_pos, ETA) = predictPos();
    

    mode1 = modeSelektor(predict_pos, ETA, ee1_pos, prev_mode1, 1);
    switch (mode1) {
      case 1: //track
        pub_pos_quat1.publish(trackDS(predict_pos, rest1_quat, iiwa1_base_pos, 1));
        ee1_pos_init = ee1_pos;
        break;
      case 2: //stop
        pub_pos_quat1.publish(stopDS(predict_pos, rest1_quat, iiwa1_base_pos, 1));
        break;
      case 3: //hit
        pub_vel_quat1.publish(hitDS(ee1_pos, ee1_pos_init, 1));
        object_pos_init1 = object_pos;
        break;
      case 4: //post hit
        pub_pos_quat1.publish(postHit(object_pos_init1, rest1_quat, iiwa1_base_pose, 1));
        break;
      case 5: //rest
        pub_pos_quat1.publish(rest(rest1_pos, rest1_quat));
        ee1_pos_init = ee1_pos;
        break;
    }
    prev_mode1 = mode1;


    mode2 = modeSelektor(predict_pos, ETA, ee2_pos, prev_mode2, 2);
    switch (mode2) {
      case 1: //track
        pub_pos_quat2.publish(trackDS(predict_pos, rest2_quat, iiwa2_base_pos, 2));
        ee2_pos_init = ee2_pos;
        break;
      case 2: //stop
        pub_pos_quat2.publish(stopDS(predict_pos, rest2_quat, iiwa2_base_pos, 2));
        break;
      case 3: //hit
        pub_vel_quat2.publish(hitDS(ee2_pos, ee2_pos_init, 2));
        object_pos_init2 = object_pos;
        break;
      case 4: //post hit
        pub_pos_quat2.publish(postHit(object_pos_init2, rest2_quat, iiwa2_base_pose, 2));
        break;
      case 5: //rest
        pub_pos_quat2.publish(rest(rest2_pos, rest2_quat));
        ee2_pos_init = ee2_pos;
        break;
    }
    prev_mode2 = mode2;
    


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


    if (object_vel.norm()<0.01 && (object_pos[1] < -0.9 || (object_pos[1] > -0.5 && object_pos[1] < 0.5) || object_pos[1] > 0.9)) {
      geometry_msgs::Pose new_object_pose;
      new_object_pose.position.x = 0.0;
      new_object_pose.position.y = -0.8;
      new_object_pose.position.z = 0.175;
      new_object_pose.orientation.x = 0.0;
      new_object_pose.orientation.y = 0.0;
      new_object_pose.orientation.z = 0.0;
      new_object_pose.orientation.w = 0.0;

      gazebo_msgs::ModelState modelstate;
      modelstate.model_name = (std::string) "my_box";
      modelstate.reference_frame = (std::string) "world";
      modelstate.pose = new_object_pose;

      setmodelstate.request.model_state = modelstate;
      set_state_client.call(setmodelstate);
      ROS_INFO("Resetting object pose");
    } //reset object position once out of reach
    /*
    object_data.open("/home/ros/ros_overlay_ws/src/i_am_project/data/object_data.csv", std::ofstream::out | std::ofstream::app);
    if(!object_data.is_open())
    {
      std::cerr << "Error opening output file.\n";
      std::cout << "Current path is " << std::experimental::filesystem::current_path() << '\n';
    }
    object_data << des_speed << ", " << theta << ", " << object_pos_init1(0) << ", " << object_pos_init1(1) << ", " << object_pos_init1(2) << ", " << object_pos(0) << ", " << object_pos(1) << ", " << object_pos(2) << "\n";
    object_data.close();
    */

    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
