#include "../include/hit_DS.h"


geometry_msgs::Pose hitDS(double des_speed, Eigen::Vector3d object_pos, Eigen::Vector3d center2, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_pos_init){
  geometry_msgs::Pose vel_quat;
  Eigen::Vector3d object_offset = {0.0, 0.0, 0.025};
  object_pos += object_offset;
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


  double theta = -std::atan2(center2[0]-object_pos[0],center2[1]-object_pos[1]); //angle between object and target

  Eigen::Matrix3d rot_mat;
  rot_mat << cos(theta), -sin(theta), 0, 
             sin(theta), cos(theta), 0,
             0, 0, 1;
  

  
  Eigen::Vector3d unit_x = {0.0, 1.0, 0.0};

  Eigen::Vector3d attractor_main;
  Eigen::Vector3d attractor_aux;
  
  attractor_main = object_pos + 1*rot_mat*unit_x;
  attractor_aux = (4.0/3.0)*object_pos - (1.0/3.0)*attractor_main;
  


  Eigen::Vector3d vel_world = {0.0, 0.0, 0.0};
  double alpha = calculate_alpha(ee_pos, ee_pos_init, object_pos, attractor_main);
  
  vel_world = alpha*nominal_aux(rot_mat, gain_aux, ee_pos, attractor_aux) + (1 - alpha)*nominal_main(rot_mat, gain_main, ee_pos, attractor_main)
            + modulated_DS(attractor_main, object_pos, ee_pos, modulated_sigma);

  vel_world = vel_world*des_speed / vel_world.norm();


  Eigen::Vector4d quat_world = pointsToQuat(object_pos, attractor_main);


  //Transform vel and quat from world to iiwa frames
  Eigen::Vector3d vel_iiwa = vel_world;
  Eigen::Vector4d quat_iiwa = quat_world;

  vel_quat.position.x = vel_iiwa[0];
  vel_quat.position.y = vel_iiwa[1];
  vel_quat.position.z = vel_iiwa[2];
  vel_quat.orientation.w = quat_iiwa[0];
  vel_quat.orientation.x = quat_iiwa[1];
  vel_quat.orientation.y = quat_iiwa[2];
  vel_quat.orientation.z = quat_iiwa[3];

  return vel_quat;
}