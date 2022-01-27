#include "../include/modeselektor.h"

int modeSelektor(Eigen::Vector3d object_pos, Eigen::Vector3d object_vel, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_vel, Eigen::Vector3d object_pos_init, Eigen::Vector3d aim, Eigen::Vector2d ee_offset, const int prev_mode){
  int mode = prev_mode;           // if none of the conditions are met, mode remains the same

  Eigen::Vector3d d_center = aim - object_pos_init;
  Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};
  



  bool moving;
  if (object_vel.norm() > 0.1){moving = true;}
  else {moving = false;}

  bool moved;
  if ((object_pos-object_pos_init).norm()>0.01){moved = true;}
  else {moved = false;}


  bool ee_ready;
  if ((ee_pos-(object_pos - ee_offset[0]*d_center/d_center.norm() + v_offset)).norm() < 0.05 && ee_vel.norm() < 0.05){ee_ready = true;}
  else {ee_ready = false;}








  switch (prev_mode){
    case 1:   //track
      if (ee_ready && !moved) {mode = 2;}             //if object will be in feasible position and stops in 0.5s and ee is in correct position, go to hit
      break;

    case 2:   //hit
      if (moving) {mode = 3;}                                   //if object starts moving because it is hit, go to post hit and initialize kalman                                          
      break;

    case 3:   //post hit
      if ((object_pos-object_pos_init).dot(d_center) > 0.1) {mode = 4;}                            //if object has left the range of arm, go to rest
      break;

    case 4:   //rest
      if (ee_ready && !moved) {mode = 2;}               //same as when being in tracking mode, since mode is initialized in rest
      if (!ee_ready && !moved) {mode = 1;}                //if object is going to be hittable but ee is not in the right position, lets track! 
      break;
  }
  return mode;
}
