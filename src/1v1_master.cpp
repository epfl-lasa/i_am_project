//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer), Daan Stokbroekx
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "../include/1v1_master.h"

using namespace std;


double des_speed;
double theta;


// Stuff to measure
geometry_msgs::Pose object_pose, iiwa_base_pose, ee_pose;
geometry_msgs::Twist object_twist, ee_twist;

Eigen::Vector3d object_pos, object_pos_init, iiwa_base_pos, ee_pos, new_object_pos_init;
Eigen::Vector3d object_vel, ee_vel;
//Eigen::Vector3d hit_force = Eigen::Vector3d::Zero();

Eigen::Vector3d object_rpy;
double object_th, object_th_mod;
Eigen::Vector4d th_quat;

std::vector<double> iiwa_joint_angles, iiwa_joint_vel;


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
  object_th = object_rpy[2];                                                                                             //only the z-axis
  object_th_mod = std::fmod(object_th+M_PI+M_PI/4,M_PI/2)-M_PI/4;                                                            //get relative angle of box face facing the arm
}

void iiwaSimCallback(const gazebo_msgs::LinkStates link_states){
  int iiwa_ee_index = getIndex(link_states.name, "iiwa1::iiwa1_link_7");
  int iiwa_base_index = getIndex(link_states.name, "iiwa1::iiwa1_link_0");

  ee_pose = link_states.pose[iiwa_ee_index];
  ee_pos << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z;
  
  iiwa_base_pose = link_states.pose[iiwa_base_index];
  iiwa_base_pos << iiwa_base_pose.position.x,iiwa_base_pose.position.y,iiwa_base_pose.position.z;

}

//Optitrack
void objectCallback(const geometry_msgs::Pose object_pose){
  object_pos << object_pose.position.x, object_pose.position.y, object_pose.position.z;

  object_rpy = quatToRPY({object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z}); //get orientation in rpy
  object_th = object_rpy[2];                                                                                             //only the z-axis
  object_th_mod = std::fmod(object_th+M_PI+M_PI/4,M_PI/2)-M_PI/4;                                                            //get relative angle of box face facing the arm
}

void iiwaBaseCallback(const geometry_msgs::Pose base_pose){
  iiwa_base_pos << base_pose.position.x, base_pose.position.y, base_pose.position.z;
}

void iiwaEEPoseCallback(const geometry_msgs::Pose ee_pose_msg){
  ee_pose.position.x = ee_pose_msg.position.x;
  ee_pose.position.y = ee_pose_msg.position.y;
  ee_pose.position.z = ee_pose_msg.position.z;
  ee_pose.orientation.w = ee_pose_msg.orientation.w;
  ee_pose.orientation.x = ee_pose_msg.orientation.x;
  ee_pose.orientation.y = ee_pose_msg.orientation.y;
  ee_pose.orientation.z = ee_pose_msg.orientation.z;
  ee_pos << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z;
}

//Passive_track (iiwa_toolkit)
void iiwaEETwistCallback(const geometry_msgs::Twist ee_twist_msg){
  ee_twist.linear.x = ee_twist_msg.linear.x;
  ee_twist.linear.y = ee_twist_msg.linear.y;
  ee_twist.linear.z = ee_twist_msg.linear.z;
  ee_twist.angular.x = ee_twist_msg.angular.x;
  ee_twist.angular.y = ee_twist_msg.angular.y;
  ee_twist.angular.z = ee_twist_msg.angular.z;
  ee_vel << ee_twist.linear.x, ee_twist.linear.y, ee_twist.linear.z;
}

void iiwaJointCallback(sensor_msgs::JointState joint_states){
  iiwa_joint_angles = joint_states.position;
  iiwa_joint_vel = joint_states.velocity;
}


void initCallback(const geometry_msgs::Pose object_pose_init){
  new_object_pos_init << object_pose_init.position.x, object_pose_init.position.y, object_pose_init.position.z;
}


int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "AH_master");
  ros::NodeHandle nh;
  ros::Rate rate(100);


  //Get environment setting from param
  bool object_real;
  bool iiwa_real;
  bool hollow;
  if(!nh.getParam("object_real", object_real)){ROS_ERROR("Param object_real not found");}
  if(!nh.getParam("iiwa_real", iiwa_real)){ROS_ERROR("Param iiwa_real not found");}
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
  
  ros::Subscriber iiwa_base_subs;
  ros::Subscriber iiwa_ee_subs;
  ros::Subscriber iiwa_subs;
  if (iiwa_real){
    iiwa_base_subs = nh.subscribe("/simo_track/robot_left/pose", 10, iiwaBaseCallback);
    iiwa_ee_subs = nh.subscribe("/simo_track/robot_left/ee_pose", 10, iiwaEEPoseCallback);
    }
    else{
    iiwa_subs = nh.subscribe("/gazebo/link_states", 10, iiwaSimCallback);
  }

  ros::service::waitForService("iiwa_jacobian_server");
  get_jacobian_client = nh.serviceClient<iiwa_tools::GetJacobian::Request>("iiwa_jacobian_server");

  ros::Subscriber iiwa_ee_twist_subs = nh.subscribe("iiwa1/ee_twist", 100, iiwaEETwistCallback);          //from passive_control and iiwa_ros
  ros::Subscriber iiwa_joint_subs = nh.subscribe("/iiwa1/joint_states", 100, iiwaJointCallback);
  

  //Publishers for position and velocity commands for IIWA end-effectors
  ros::Publisher pub_vel_quat = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/vel_quat", 1);
  ros::Publisher pub_pos_quat = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/pos_quat", 1);
  

  //Information exchange for initializing trials
  ros::Publisher update_pub = nh.advertise<std_msgs::Bool>("trials/update", 100);
  ros::Subscriber init_subs = nh.subscribe("trials/init", 100, initCallback);


  
  double ee_offset_h;
  double ee_offset_v;
  if(!nh.getParam("ee_offset/h", ee_offset_h)){ROS_ERROR("Param ee_offset/h not found");}
  if(!nh.getParam("ee_offset/v", ee_offset_v)){ROS_ERROR("Param ee_offset/v not found");}
  Eigen::Vector2d ee_offset = {ee_offset_h, ee_offset_v};


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
    geometry_msgs::Pose pose;
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
    nh.getParam("box/initial_pos/x", box.pose.position.x);
    nh.getParam("box/initial_pos/y", box.pose.position.y);
    nh.getParam("box/initial_pos/z", box.pose.position.z);
  box.izz = 1.0/12* box.mass* (pow(box.size_x,2) + pow(box.size_y,2));
  
  
  //Set hitting speed and direction. Once we change attractor to something more related to the game, we no longer need this
    std::cout << "Enter the desired speed of hitting (between 0 and 1)" << std::endl;
    std::cin >> des_speed;

    while(des_speed < 0 || des_speed > 3){
      std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
      std::cin >> des_speed;
    }

    std::cout << "Enter the desired direction of hitting (between -pi/2 and pi/2)" << std::endl;
    std::cin >> theta;

    while(theta < -M_PI_2 || theta > M_PI_2){
      std::cout <<"Invalid entry! Please enter a valid value: " << std::endl;
      std::cin >> theta;
  }


  int mode = 4;
  int prev_mode = mode;

  Eigen::Vector3d ee_pos_init = ee_pos;
  object_pos_init << box.pose.position.x, box.pose.position.y, box.pose.position.z;

  std_msgs::Bool update;
  bool infeasible = false;
  bool reset = false;
  bool once_reset = true;
  std::queue<double> object_pos_norm_q;

  int trial = 1;
  
  //Storing data to get the right values in time. The data we want is actually from right before events that we can recognize (e.g. speed right before hitting)
    std::queue<Eigen::Vector3d> object_pos_q;
    std::queue<double> object_theta_q;
    std::queue<geometry_msgs::Pose> ee_pose_q;
    std::queue<geometry_msgs::Twist> ee_twist_q;
    std::queue<std::vector<double>> iiwa_joint_angles_q;
    std::queue<std::vector<double>> iiwa_joint_vel_q;

    bool once1 = true;
    bool once2 = true;
    bool tracking = false;
    int oneinten;
    std::stringstream data_path;
    data_path << ros::package::getPath("i_am_project") << "/data/hitting/object_data.csv";
    std::ofstream object_data;
    std::stringstream regression_path;
    regression_path << ros::package::getPath("i_am_project") << "/data/hitting/regression_data.csv";
    std::ofstream regression_data;
  bool store_data = 0;

  iiwa_tools::IiwaTools iiwatools;
  iiwa_tools::RobotState robot_state;
  robot_state.position.resize(7);
  robot_state.velocity.resize(7);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd(6, 7);


  while(ros::ok()){
    Eigen::Vector3d unitY = {0.0, 1.0, 0.0};
    Eigen::Vector3d aim = object_pos_init + unitY;
    Eigen::Vector3d d_center = aim - object_pos_init;
    Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};

    // Select correct operating mode for both arms based on conditions on (predicted) object position and velocity and ee position
    mode = modeSelektor(object_pos, object_vel, ee_pos, ee_vel, object_pos_init, aim, ee_offset, prev_mode);
    switch (mode) {
      case 1: //track
        pub_pos_quat.publish(track(object_pos, aim, ee_offset, iiwa_base_pos));
        ee_pos_init = ee_pos; //we can go from track to hit. For hit, we need initial
        break;
      case 2: //hit
        pub_vel_quat.publish(hitDS(des_speed, object_pos, aim, ee_pos, ee_pos_init));
        break;
      case 3: //post hit
        pub_pos_quat.publish(postHit(object_pos_init, aim, iiwa_base_pos));
        break;
      case 4: //rest
        pub_pos_quat.publish(rest(object_pos_init, aim, ee_offset, iiwa_base_pos));
        ee_pos_init = ee_pos; //we can also go from rest to hit..
        break;
    }
    prev_mode = mode;




    object_data.open(data_path.str(), std::ofstream::out | std::ofstream::app);
    if(!object_data.is_open()){
      std::cerr << "Error opening output file.\n";
      std::cout << "Current path is " << std::experimental::filesystem::current_path() << '\n';
    }
    regression_data.open(regression_path.str(), std::ofstream::out | std::ofstream::app);
    if(!regression_data.is_open()){
      std::cerr << "Error opening output file.\n";
      std::cout << "Current path is " << std::experimental::filesystem::current_path() << '\n';
    }


    object_pos_norm_q.push(object_pos.norm());
    while (object_pos_norm_q.size() > 500){object_pos_norm_q.pop();}

    if (object_pos_norm_q.front() - object_pos_norm_q.back() < 0.005 && object_pos_norm_q.size() > 490){
      infeasible = true;
    } else {infeasible = false;}

    if (((object_pos-object_pos_init).dot(d_center) > 0.1 && object_vel.norm()<0.01 || infeasible == true) && (ee_pos-(object_pos_init - ee_offset[0]*d_center/d_center.norm() + v_offset)).norm() < 0.05){
      reset = true;
    } else {reset = false;}

    if (reset == true && once_reset == true) {
      once_reset = false;

      std::queue<double> empty;
      std::swap(object_pos_norm_q, empty);

      update.data = true;
      object_pos_init = new_object_pos_init;
      //Set new pose of box
        gazebo_msgs::SetModelState setmodelstate;
        geometry_msgs::Pose new_box_pose;
        new_box_pose.position.x = object_pos_init[0];
        new_box_pose.position.y = object_pos_init[1];
        new_box_pose.position.z = object_pos_init[2];
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

        if (hollow == true){
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
        }

      ROS_INFO("Resetting object pose");
    }

    update_pub.publish(update);

    if ((object_pos-object_pos_init).norm() < 0.01){ 
      update.data = false;
      once_reset = true;
    }



    object_pos_q.push(object_pos);
    object_theta_q.push(object_th);
    ee_pose_q.push(ee_pose);
    ee_twist_q.push(ee_twist);
    iiwa_joint_angles_q.push(iiwa_joint_angles);
    iiwa_joint_vel_q.push(iiwa_joint_vel);

    while (object_pos_q.size() > 3){object_pos_q.pop();}
    while (object_theta_q.size() > 3){object_theta_q.pop();}
    while (ee_pose_q.size() > 3){ee_pose_q.pop();}
    while (ee_twist_q.size() > 3){ee_twist_q.pop();}
    while (iiwa_joint_angles_q.size() > 3){iiwa_joint_angles_q.pop();}
    while (iiwa_joint_vel_q.size() > 3){iiwa_joint_vel_q.pop();}

    

    if (mode == 3 && once1 == true){  //store data right before hit (this is once, directly after hit)
      once1 = false;
      once2 = true;
      object_data << "trial_no.       ," << trial << "\n";
      object_data << "box_properties  ," << box.size_x << ", " << box.size_y << ", " << box.size_z << ", " << box.com_x << ", " << box.com_y << ", " << box.com_z << ", " << box.mass << ", " << box.mu << ", " << box.mu2 << "\n";
      object_data << "pre_hit_joints  ," << iiwa_joint_angles_q.front()[0] << ", " << iiwa_joint_angles_q.front()[1] << ", " << iiwa_joint_angles_q.front()[2] << ", " << iiwa_joint_angles_q.front()[3] << ", " << iiwa_joint_angles_q.front()[4] << ", " << iiwa_joint_angles_q.front()[5] << ", " << iiwa_joint_angles_q.front()[6] << "\n";
      object_data << "post_hit_joints ," << iiwa_joint_angles[0] << ", " << iiwa_joint_angles[1] << ", " << iiwa_joint_angles[2] << ", " << iiwa_joint_angles[3] << ", " << iiwa_joint_angles[4] << ", " << iiwa_joint_angles[5] << ", " << iiwa_joint_angles[6] << "\n";
      object_data << "pre_hit_ee      ," << ee_pose_q.front().position.x << ", " << ee_pose_q.front().position.y << ", " << ee_pose_q.front().position.z << ", " << ee_pose_q.front().orientation.x << ", " << ee_pose_q.front().orientation.y << ", " << ee_pose_q.front().orientation.z << ", " << ee_pose_q.front().orientation.w << ", " << ee_twist_q.front().linear.x << ", " << ee_twist_q.front().linear.y << ", " << ee_twist_q.front().linear.z << ", " << ee_twist_q.front().angular.x << ", " << ee_twist_q.front().angular.y << ", " << ee_twist_q.front().angular.z << "\n";
      object_data << "post_hit_ee     ," << ee_pose.position.x << ", " << ee_pose.position.y << ", " << ee_pose.position.z << ", " << ee_pose.orientation.x << ", " << ee_pose.orientation.y << ", " << ee_pose.orientation.z << ", " << ee_pose.orientation.w << ", " << ee_twist.linear.x << ", " << ee_twist.linear.y << ", " << ee_twist.linear.z << ", " << ee_twist.angular.x << ", " << ee_twist.angular.y << ", " << ee_twist.angular.z << "\n";
      object_data << "pre_hit_object  ," << object_pos_q.front()[0] << ", " << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << "\n";



      for (int i=0; i<7; i++){
        robot_state.position[i] = iiwa_joint_angles_q.front()[i];
        robot_state.velocity[i] = iiwa_joint_vel_q.front()[i];
      }
      jacobian = get_jacobian_client.call(jacobian_request)
      //jacobian = iiwatools.jacobian(robot_state);
      //std::cout << jacobian << std::endl;
      regression_data << trial << ", " << "ee_inertia, " << ee_twist_q.front().linear.y << ", " << ee_twist.linear.y << ", " << object_pos_q.front()[1] << ", ";

      trial++;
    }
    if (reset == true && once2 == true){  //or if it will be hit back just before it came to a halt
      once2 = false;
      once1 = true;
      object_data << "end_object      ," << object_pos_q.front()[0] << ", " << object_pos_q.front()[1] << ", " << object_pos_q.front()[2] << "\n\n\n";

      regression_data << object_pos_q.front()[1] << "\n";
    }

    

    /*
    if (tracking == true && oneinten >= 10){
      oneinten = 0;
      object_data << current_time.toSec() << ", " << object_pos[0] << ", " << object_pos[1] << ", " << object_pos[2] << ", " << object_th << ", " << object_twist.angular.z << "\n";
      if (object_vel.norm()<0.05){
        tracking = false;
        if (once11 == false){
          once12 = false;
          tracking = false;
          object_data << "end, free" << "\n\n\n";
          once21 = true;
          once22 = true;
        }
        if (once21 == false){
          once22 = false;
          tracking = false;
          object_data << "end, free" << "\n\n\n";
          once11 = true;
          once12 = true;
        }
      }
    }
    oneinten++;
    
    */
    object_data.close();
    regression_data.close();
    
    
    // Some infos
      std::stringstream ss1;

      ss1 << "mode: " << mode;

      ROS_INFO("%s",ss1.str().c_str());
      

      //ss3 << "object    : " << object_pos[0] << " " << object_pos[1] << " " << object_pos[2];
      //ss4 << "iiwa1_base: " << predict_pos[1];
      //ss5 << "ETA       : " << ETA*5.0;
      //ss6 << "estima vel: " << object_vel[1];
      //ss8 << "final th  : " << predict_th;
      //ss9 << "current th: " << object_th;

      //ROS_INFO("%s",ss3.str().c_str());
      //ROS_INFO("%s",ss4.str().c_str());
      //ROS_INFO("%s",ss5.str().c_str());
      //ROS_INFO("%s",ss6.str().c_str());
      //ROS_INFO("%s",ss7.str().c_str());
      //ROS_INFO("%s",ss8.str().c_str());
    //ROS_INFO("%s",ss9.str().c_str());




    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}

