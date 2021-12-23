//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Harshit Khurana (maintainer), Daan Stokbroekx
//|    email:   harshit.khurana@epfl.ch
//|    website: lasa.epfl.ch

#include "../include/1v1_master.h"

using namespace std;


double des_speed;
double theta;

int key_ctrl = 0;

double min_y, max_y;

Eigen::Vector3d iiwa_flip = {0.0, 1.0, 0.0};
Eigen::Matrix3d R_Opti; 

// Stuff to measure
geometry_msgs::Pose object_pose, iiwa1_base_pose, iiwa2_base_pose, ee1_pose, ee2_pose;
geometry_msgs::Twist object_twist, ee1_twist, ee2_twist;

Eigen::Vector3d object_pos, iiwa1_base_pos, iiwa2_base_pos, ee1_pos, ee2_pos;
Eigen::Vector3d ee1_vel, ee2_vel;
//Eigen::Vector3d hit_force = Eigen::Vector3d::Zero();

Eigen::Vector3d object_rpy;
double object_th, object_th_mod;
Eigen::Vector4d th_quat;

std::vector<double> iiwa1_joint_angles, iiwa2_joint_angles;


// Stuff to estimate
Eigen::Vector3d object_vel, predict_pos;
double ETA;
double stdev;

double predict_th;


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

  object_rpy = quatToRPY({object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z}); //get orientation in rpy
  object_th = object_rpy[2];                                                                                             //only the z-axis
  object_th_mod = std::fmod(object_th+M_PI+M_PI/4,M_PI/2)-M_PI/4;                                                            //get relative angle of box face facing the arm
}

void iiwaSimCallback(const gazebo_msgs::LinkStates link_states){
  int iiwa1_ee_index = getIndex(link_states.name, "iiwa1::iiwa1_link_7");
  int iiwa2_ee_index = getIndex(link_states.name, "iiwa2::iiwa2_link_7");
  int iiwa1_base_index = getIndex(link_states.name, "iiwa1::iiwa1_link_0");
  int iiwa2_base_index = getIndex(link_states.name, "iiwa2::iiwa2_link_0");

  ee1_pose = link_states.pose[iiwa1_ee_index];
  ee2_pose = link_states.pose[iiwa2_ee_index];
  ee1_pos << ee1_pose.position.x, ee1_pose.position.y, ee1_pose.position.z;
  ee2_pos << ee2_pose.position.x, ee2_pose.position.y, ee2_pose.position.z;
  
  iiwa1_base_pose = link_states.pose[iiwa1_base_index];
  iiwa2_base_pose = link_states.pose[iiwa2_base_index];
  iiwa1_base_pos << iiwa1_base_pose.position.x,iiwa1_base_pose.position.y,iiwa1_base_pose.position.z;
  iiwa2_base_pos << iiwa2_base_pose.position.x,iiwa2_base_pose.position.y,iiwa2_base_pose.position.z;

  min_y = iiwa2_base_pos[1] - 0.5;
  max_y = iiwa2_base_pos[1] - 0.1;
}

//Optitrack
void objectCallback(const geometry_msgs::Pose object_pose){
  object_pos << object_pose.position.x, object_pose.position.y, object_pose.position.z;
  object_pos = R_Opti*object_pos;

  object_rpy = quatToRPY({object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z}); //get orientation in rpy
  object_th = object_rpy[2];                                                                                             //only the z-axis
  object_th_mod = std::fmod(object_th+M_PI+M_PI/4,M_PI/2)-M_PI/4;                                                            //get relative angle of box face facing the arm
}

void iiwa1BaseCallback(const geometry_msgs::Pose base_pose){
  iiwa1_base_pos << base_pose.position.x, base_pose.position.y, base_pose.position.z;
  iiwa1_base_pos = R_Opti*iiwa1_base_pos;
}

void iiwa2BaseCallback(const geometry_msgs::Pose base_pose){
  iiwa2_base_pos << base_pose.position.x, base_pose.position.y, base_pose.position.z;
  iiwa2_base_pos = R_Opti*iiwa2_base_pos;
  min_y = iiwa2_base_pos[1] - 0.5;
  max_y = iiwa2_base_pos[1] - 0.1;
}

void iiwa1EEPoseCallback(const geometry_msgs::Pose ee_pose){
  ee1_pose.position.x = ee_pose.position.x;
  ee1_pose.position.y = ee_pose.position.y;
  ee1_pose.position.z = ee_pose.position.z;
  ee1_pose.orientation.w = ee_pose.orientation.w;
  ee1_pose.orientation.x = ee_pose.orientation.x;
  ee1_pose.orientation.y = ee_pose.orientation.y;
  ee1_pose.orientation.z = ee_pose.orientation.z;
  ee1_pos << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z;
  ee1_pos = R_Opti*ee1_pos;
}

void iiwa2EEPoseCallback(const geometry_msgs::Pose ee_pose){
  ee2_pose.position.x = ee_pose.position.x;
  ee2_pose.position.y = ee_pose.position.y;
  ee2_pose.position.z = ee_pose.position.z;
  ee2_pose.orientation.w = ee_pose.orientation.w;
  ee2_pose.orientation.x = ee_pose.orientation.x;
  ee2_pose.orientation.y = ee_pose.orientation.y;
  ee2_pose.orientation.z = ee_pose.orientation.z;
  ee2_pos << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z;
  ee2_pos = R_Opti*ee2_pos;
}

//Passive_track (iiwa_toolkit)
void iiwa1EETwistCallback(const geometry_msgs::Twist ee_twist){
  ee1_twist.linear.x = ee_twist.linear.x;
  ee1_twist.linear.y = ee_twist.linear.y;
  ee1_twist.linear.z = ee_twist.linear.z;
  ee1_twist.angular.x = ee_twist.angular.x;
  ee1_twist.angular.y = ee_twist.angular.y;
  ee1_twist.angular.z = ee_twist.angular.z;
  ee1_vel << ee1_twist.linear.x, ee1_twist.linear.y, ee1_twist.linear.z;
}

void iiwa2EETwistCallback(const geometry_msgs::Twist ee_twist){
  ee2_twist.linear.x = ee_twist.linear.x;
  ee2_twist.linear.y = ee_twist.linear.y;
  ee2_twist.linear.z = ee_twist.linear.z;
  ee2_twist.angular.x = ee_twist.angular.x;
  ee2_twist.angular.y = ee_twist.angular.y;
  ee2_twist.angular.z = ee_twist.angular.z;
  ee2_vel << ee2_twist.linear.x, ee2_twist.linear.y, ee2_twist.linear.z;
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
  object_vel << estimation.data[5], estimation.data[6], estimation.data[7];
  //predict_th = estimation.data[8];
}

//manual control
void modeCallback(std_msgs::Int16 msg){
  //if (msg.data <= 5){manual_mode1 = msg.data;}
  //if (msg.data >= 6){manual_mode2 = msg.data-5;} 
  //if (msg.data == 0){manual_mode2 = 5;}
  key_ctrl = msg.data;
}



int main (int argc, char** argv){

	//ROS Initialization
  ros::init(argc, argv, "AH_master");
  ros::NodeHandle nh;
  ros::Rate rate(100);


  //Get environment setting from param
  bool object_real;
  bool iiwa_real;
  bool manual_mode;
  bool hollow;
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
  
  ros::Subscriber iiwa1_base_subs;
  ros::Subscriber iiwa2_base_subs;
  ros::Subscriber iiwa1_ee_subs;
  ros::Subscriber iiwa2_ee_subs;
  ros::Subscriber iiwa_subs;
  if (iiwa_real){
    iiwa1_base_subs = nh.subscribe("/simo_track/robot_left/pose", 10, iiwa1BaseCallback);
    iiwa2_base_subs = nh.subscribe("/simo_track/robot_right/pose", 10, iiwa2BaseCallback);
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

  //Subscriber to key control
  ros::Subscriber mode_sub = nh.subscribe("mode",10,modeCallback);


  //Publishers for position and velocity commands for IIWA end-effectors
  ros::Publisher pub_vel_quat1 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/vel_quat", 1);
  ros::Publisher pub_pos_quat1 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa1/pos_quat", 1);
  ros::Publisher pub_vel_quat2 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/vel_quat", 1);
  ros::Publisher pub_pos_quat2 = nh.advertise<geometry_msgs::Pose>("/passive_control/iiwa2/pos_quat", 1);
  

  


  //Center points of desired workspaces (used to aim for and tell what orientation)
  std::vector<double> center1vec;
  std::vector<double> center2vec;
  Eigen::Vector3d center1;
  Eigen::Vector3d center2;
  if(!nh.getParam("center1", center1vec)){ROS_ERROR("Param center1 not found");}
  if(!nh.getParam("center2", center2vec)){ROS_ERROR("Param center2 not found");}
  center1 << center1vec[0], center1vec[1], center1vec[2];
  center2 << center2vec[0], center2vec[1], center2vec[2];
  
  double ee_offset_h;
  double ee_offset_v;
  if(!nh.getParam("ee_offset/h", ee_offset_h)){ROS_ERROR("Param ee_offset/h not found");}
  if(!nh.getParam("ee_offset/v", ee_offset_v)){ROS_ERROR("Param ee_offset/v not found");}
  Eigen::Vector2d ee_offset = {ee_offset_h, ee_offset_v};

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


  R_Opti << -1.0, 0.0, 0.0, 0.0,-1.0, 0.0, 0.0, 0.0, 1.0;
  //Eigen::Vector3d des_pos1, des_pos2, rand_pos;
  //double randx;
  //double randy;


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
    //Determine desired direction of hit
    //randx = ((double)(rand() % 2500-1250))/10000;
    //randy = ((double)(rand() % 2500-1250))/10000;
    //rand_pos = {randx, randy, 0.0};
    //double theta_des1 = -std::atan2((des_pos_set[0]-predict_pos[0]),(des_pos_set[1]-predict_pos[1])); //can also use cur_pos
    //double theta_des2 = -std::atan2((des_pos_set[0]-predict_pos[0]),(des_pos_set[1]+predict_pos[1]));


    // Select correct operating mode for both arms based on conditions on (predicted) object position and velocity and ee position
    if (manual_mode){
      mode1 = maniModeSelektor(object_pos, object_vel, ee1_pos, center1, center2, ee_offset, hittable_params, prev_mode1, key_ctrl, 1);
    }
    else{
      mode1 = modeSelektor(object_pos, object_vel, predict_pos, ETA, ee1_pos, center1, center2, ee_offset, hittable_params, prev_mode1);
    }
    switch (mode1) {
      case 1: //track
        pub_pos_quat1.publish(track(predict_pos, center2, ee_offset, iiwa1_base_pos));
        ee1_pos_init = ee1_pos; //we can go from track to hit. For hit, we need initial
        break;
      case 2: //stop
        pub_pos_quat1.publish(block(object_pos, predict_pos, center1, center2, object_th_mod, iiwa1_base_pos));
        break;
      case 3: //hit
        pub_vel_quat1.publish(hitDS(des_speed, object_pos, center2, ee1_pos, ee1_pos_init));
        object_pos_init1 = object_pos; //need this for post-hit to guide the arm right after hit
        break;
      case 4: //post hit
        pub_pos_quat1.publish(postHit(object_pos_init1, center2, iiwa1_base_pos));
        break;
      case 5: //rest
        pub_pos_quat1.publish(rest(center1, center2, ee_offset, iiwa1_base_pos));
        ee1_pos_init = ee1_pos; //we can also go from rest to hit..
        break;
    }
    prev_mode1 = mode1;

    if (manual_mode){
      mode2 = maniModeSelektor(object_pos, object_vel, ee2_pos, center2, center1, ee_offset, hittable_params, prev_mode2, key_ctrl, 2);
    }
    else{
      mode2 = modeSelektor(object_pos, object_vel, predict_pos, ETA, ee2_pos, center2, center1, ee_offset, hittable_params, prev_mode2);
    }
    switch (mode2) {
      case 1: //track
        pub_pos_quat2.publish(track(predict_pos, center1, ee_offset, iiwa2_base_pos));
        ee2_pos_init = ee2_pos;
        break;
      case 2: //stop
        pub_pos_quat2.publish(block(object_pos, predict_pos, center2, center1, object_th_mod, iiwa2_base_pos));
        break;
      case 3: //hit
        pub_vel_quat2.publish(hitDS(des_speed, object_pos, center1, ee2_pos, ee2_pos_init));
        object_pos_init2 = object_pos;
        break;
      case 4: //post hit
        pub_pos_quat2.publish(postHit(object_pos_init2, center1, iiwa2_base_pos));
        break;
      case 5: //rest
        pub_pos_quat2.publish(rest(center2, center1, ee_offset, iiwa2_base_pos));
        ee2_pos_init = ee2_pos;
        break;
    }
    prev_mode2 = mode2;
    




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
      gazebo_msgs::SetModelState setmodelstate;
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
      tracking = false;
      if (once11 == false){
        once12 = false;
        tracking = false;
        object_data << "end, reset" << "\n\n\n";
        once21 = true;
        once22 = true;
      }
      if (once21 == false){
        once22 = false;
        tracking = false;
        object_data << "end, reset" << "\n\n\n";
        once11 = true;
        once12 = true;
      }
    }


    


    ros::Time current_time = ros::Time::now();
    //Store the actual data for three time steps. This turns out to be the right time. In other words, post-hit is initiated three time steps after the step right before impact, which is the step we want data from.
    object_pos_q.push(object_pos);
    predict_pos_q.push(predict_pos);
    object_theta_q.push(object_th);
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
    if (mode2 == 2 && object_pos[1] > min_y + 0.25 && once12 == true){  //store predicted pos if it will be stopped
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
    if (mode1 == 2 && object_pos[1] < -(min_y+0.25) && once22 == true){
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

    object_data.close();
    
    
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
      std::stringstream ss6;
      std::stringstream ss7;
      std::stringstream ss8;
      std::stringstream ss9;
      std::stringstream ss10;
      std::stringstream ss11;
      

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



/*std_msgs::Float32 posfloat;
    posfloat.data = (float) ee1_pos[1];
    std_msgs::Float32 velfloat;
    velfloat.data = (float) ee1_vel[1];
    pub_ee1_pos.publish(posfloat);
    pub_ee1_vel.publish(velfloat);*/