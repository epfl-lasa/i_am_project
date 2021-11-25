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

geometry_msgs::Pose box_pose, ee1_pose, ee2_pose, iiwa1_base_pose, iiwa2_base_pose;
geometry_msgs::Twist box_twist, ee1_twist, ee2_twist;

Eigen::Vector3d object_pos, object_vel, ee1_pos, ee2_pos, ee1_vel, ee2_vel, iiwa1_base_pos, iiwa2_base_pos;
Eigen::Vector4d object_ori, ee1_ori, ee2_ori;

Eigen::Vector3d rest1_pos = {0.5, -0.2, 0.4};           //relative to iiwa base
Eigen::Vector3d rest2_pos = {0.5, -0.2, 0.4};
Eigen::Vector4d rest1_quat = {-0.707, 0.0, 0.0, 0.707};
Eigen::Vector4d rest2_quat = {-0.707, 0.0, 0.0, 0.707};

Eigen::Vector3d ee_offset = {0.0, -0.4, 0.225};         //relative to object

//State estimation initialization
Eigen::Matrix<double, 6,6> P = Eigen::Matrix<double, 6,6>::Zero();
Eigen::Matrix<double, 6,1> state = Eigen::Matrix<double, 6,1>::Zero();
Eigen::Vector3d predict_pos;
double ETA;





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

  object_ori << box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z, box_pose.orientation.w;
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
      if (pred_hittable == true && ETA < 0.5 && ee_ready == true) {mode = 3;}             //if object will be in feasible position and stops in 0.8s and ee is in correct position, go to hit
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
  
  //Publishers for position and velocity commands for IIWA end-effectors
  ros::Publisher pub_vel_quat1 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/vel_quat", 1);
  ros::Publisher pub_pos_quat1 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/pos_quat", 1);
  ros::Publisher pub_vel_quat2 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/vel_quat", 1);
  ros::Publisher pub_pos_quat2 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/pos_quat", 1);

  //Client to reset object pose
  ros::service::waitForService("gazebo/set_model_state");
  ros::ServiceClient set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  
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



  int mode1 = 5;
  int mode2 = 5;
  int prev_mode1 = mode1;
  int prev_mode2 = mode2;

  Eigen::Vector3d object_pos_init1 = object_pos;
  Eigen::Vector3d object_pos_init2 = object_pos;
  Eigen::Vector3d ee1_pos_init = ee1_pos;
  Eigen::Vector3d ee2_pos_init = ee2_pos;


  //Storing data to get the right values in time. The data we want is actually from right before events that we can recognize (e.g. speed right before hitting)
  std::queue<Eigen::Vector3d> object_pos_q;
  std::queue<Eigen::Vector3d> predict_pos_q;
  std::queue<Eigen::Vector3d> ee1_pos_q;
  std::queue<Eigen::Vector3d> ee1_vel_q;
  std::queue<Eigen::Vector3d> ee2_pos_q;
  std::queue<Eigen::Vector3d> ee2_vel_q;

  bool once11 = true;
  bool once12 = true;
  bool once21 = false;
  bool once22 = false;
  std::ofstream object_data;
  bool store_data = 0;




  while(ros::ok()){
    // Predict the future
    tie(P, state, predict_pos, ETA) = predictPos(P, state, object_pos, object_vel, 0.3);
    
    // Select correct operating mode for both arms based on conditions on (predicted) object position and velocity and ee position
    mode1 = modeSelektor(predict_pos, ETA, ee1_pos, prev_mode1, 1);
    switch (mode1) {
      case 1: //track
        pub_pos_quat1.publish(track(predict_pos, rest1_quat, ee_offset, iiwa1_base_pos, 1));
        ee1_pos_init = ee1_pos; //we can go from track to hit. For hit, we need initial
        break;
      case 2: //stop
        pub_pos_quat1.publish(block(predict_pos, rest1_quat, iiwa1_base_pos, 1));
        break;
      case 3: //hit
        pub_vel_quat1.publish(hitDS(des_speed, theta, object_pos, ee1_pos, ee1_pos_init, 1));
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

    mode2 = modeSelektor(predict_pos, ETA, ee2_pos, prev_mode2, 2);
    switch (mode2) {
      case 1: //track
        pub_pos_quat2.publish(track(predict_pos, rest2_quat, ee_offset, iiwa2_base_pos, 2));
        ee2_pos_init = ee2_pos;
        break;
      case 2: //stop
        pub_pos_quat2.publish(block(predict_pos, rest2_quat, iiwa2_base_pos, 2));
        break;
      case 3: //hit
        pub_vel_quat2.publish(hitDS(des_speed, theta, object_pos, ee2_pos, ee2_pos_init, 2));
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
    ee1_pos_q.push(ee1_pos);
    ee1_vel_q.push(ee1_vel);
    ee2_pos_q.push(ee2_pos);
    ee2_vel_q.push(ee2_vel);

    while (object_pos_q.size() > 3){object_pos_q.pop();}
    while (predict_pos_q.size() > 3){predict_pos_q.pop();}
    while (ee1_pos_q.size() > 3){ee1_pos_q.pop();}
    while (ee1_vel_q.size() > 3){ee1_vel_q.pop();}
    while (ee2_pos_q.size() > 3){ee2_pos_q.pop();}
    while (ee2_vel_q.size() > 3){ee2_vel_q.pop();}


    object_data.open("/home/daan/catkin_ws/src/i_am_project/data/hitting/object_data.csv", std::ofstream::out | std::ofstream::app);
    if(!object_data.is_open())
    {
      std::cerr << "Error opening output file.\n";
      std::cout << "Current path is " << std::experimental::filesystem::current_path() << '\n';
    }


    if (mode1 == 4 && once11 == true){  //store data right before hit
      once11 = false;
      object_data << box.size_x << ", " << box.size_y << ", " << box.size_z << ", " << box.com_x << ", " << box.com_y << ", " << box.com_z << ", " << box.mass << ", " << box.mu << ", " << box.mu2 << ", ";
      object_data << object_pos_q.front()[0] << ", " << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << ", " << ee1_pos_q.front()[0] << ", " << ee1_pos_q.front()[1] << ", " << ee1_pos_q.front()[2] << ", " << ee1_vel_q.front()[0] << ", " << ee1_vel_q.front()[1] << ", " << ee1_vel_q.front()[2] << ", ";
    }

    if (mode2 == 2 && object_pos[1] > 0.75 && once12 == true){  //store predicted pos if it will be stopped
      once12 = false;
      object_data << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << " stopped" << "\n";
      once21 = true;
      once22 = true;
    }

    if (mode2 == 3 && once12 == true){  //or if it will be hit back just before it came to a halt
      once12 = false;
      object_data << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << " free" << "\n";
      once21 = true;
      once22 = true;
    }

    
    
    if (mode2 == 4 && once21 == true){
      once21 = false;
      object_data << box.size_x << ", " << box.size_y << ", " << box.size_z << ", " << box.com_x << ", " << box.com_y << ", " << box.com_z << ", " << box.mass << ", " << box.mu << ", " << box.mu2 << ", ";
      object_data << object_pos_q.front()[0] << ", " << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << ", " << ee2_pos_q.front()[0] << ", " << ee2_pos_q.front()[1] << ", " << ee2_pos_q.front()[2] << ", " << ee2_vel_q.front()[0] << ", " << ee2_vel_q.front()[1] << ", " << ee2_vel_q.front()[2] << ", ";
    }

    if (mode1 == 2 && object_pos[1] < -0.75 && once22 == true){
      once22 = false;
      object_data << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << " stopped" << "\n";
      once11 = true;
      once12 = true;
    }

    if (mode1 == 3 && once22 == true){
      once22 = false;
      object_data << predict_pos_q.front()[0] << ", " << predict_pos_q.front()[1] << ", " << predict_pos_q.front()[2] << " free" << "\n";
      once11 = true;
      once12 = true;
    }
    
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
