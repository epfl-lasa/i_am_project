#include "../include/block.h"

geometry_msgs::Pose block(Eigen::Vector3d object_pos, Eigen::Vector3d predict_pos, Eigen::Vector3d center1, Eigen::Vector3d center2, double object_th_mod, Eigen::Vector3d iiwa_base_pos){
  geometry_msgs::Pose pose;

  //The ratio between {the projections of {{the iiwa_base} and {the predicted position} wrt to the current position} on the vector through the centerpoints} is equal to 
  //the ratio between {the intersection point of {the trajectory} and {the line crossing the iiwa_base perpendicular to the vector through centers} wrt the current object_pos} and {the predicted position wrt to the current position of the object}
  //This intersection point is precisely where the EE should stop the object and is therefore calculated as follows:
  Eigen::Vector3d d_center = center2 - center1;
  Eigen::Vector3d traject = predict_pos - object_pos;
  double traject_on_d_center = traject.dot(d_center);
  double iiwa_offset_on_d_center = (iiwa_base_pos-object_pos).dot(d_center) + 0.1;
  Eigen::Vector3d pos_world = object_pos + iiwa_offset_on_d_center/traject_on_d_center*traject;

  
  Eigen::Vector4d rest_quat = pointsToQuat(center1, center2);
  Eigen::Vector4d quat_world = quatProd(rpyToQuat(0.0,0.0,object_th_mod), rest_quat);


  //Transform pos and quat from world to iiwa frame
  Eigen::Vector3d pos_iiwa = pos_world - iiwa_base_pos;
  Eigen::Vector4d quat_iiwa = quat_world;

  pose.position.x = pos_iiwa[0];
  pose.position.y = pos_iiwa[1];
  pose.position.z = pos_iiwa[2];
  pose.orientation.w = quat_iiwa[0];
  pose.orientation.x = quat_iiwa[1];
  pose.orientation.y = quat_iiwa[2];
  pose.orientation.z = quat_iiwa[3];
  return pose;
}