#include "../include/hit_DS.h"


geometry_msgs::Pose hitDS(double des_speed, double theta, Eigen::Vector3d object_pos, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_pos_init, Eigen::Vector3d iiwa_flip, const int iiwa_no){
  geometry_msgs::Pose vel_quat;
  Eigen::Vector3d object_offset = {0.0, 0.0, 0.025};

  int iiwa_sel = 3-2*iiwa_no;
  Eigen::Vector3d flip_vec = Eigen::Vector3d::Ones() + iiwa_flip*(iiwa_sel-1);
  Eigen::Matrix3d sel_mat;
  sel_mat << flip_vec[0], 0.0,         0.0,
             0.0,         flip_vec[1], 0.0,
             0.0,         0.0,         flip_vec[2];

  //instead of flipping direction of attractor and such, make use of the fact that iiwa2 is exactly the same as iiwa1 but rotated around the worlds z-axis with pi
  //if then the only parameters that are defined w.r.t. the world frame are rotated also, the whole situation becomes identical
  //(passive_track gives cmds w.r.t. respective iiwa frame)
  //Eigen::Vector3d object_pos2  = sel_mat*(object_pos+object_offset); 
  //Eigen::Vector3d ee_pos2      = sel_mat*ee_pos;
  //Eigen::Vector3d ee_pos_init2 = sel_mat*ee_pos_init;


  

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
  
  attractor_main = object_pos + 1*rot_mat*unit_x*flip_vec[1];
  attractor_aux = (4.0/3.0)*object_pos - (1.0/3.0)*attractor_main;
  
  

  Eigen::Vector4d des_quat = Eigen::Vector4d::Zero();
  Eigen::Vector3d ee_direction_x;
  Eigen::Vector3d ee_direction_y;
  Eigen::Vector3d ee_direction_z;
  Eigen::Matrix3d rot_ee;
  Eigen::Quaterniond rot_quat;
  
  ee_direction_z = attractor_main - object_pos; 
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
  double alpha = calculate_alpha(ee_pos, ee_pos_init, object_pos, attractor_main);
  
  des_vel = alpha*nominal_aux(rot_mat, gain_aux, ee_pos, attractor_aux) + (1 - alpha)*nominal_main(rot_mat, gain_main, ee_pos, attractor_main)
            + modulated_DS(attractor_main, object_pos, ee_pos, modulated_sigma);

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