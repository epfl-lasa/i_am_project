#include "../include/modeselektor.h"

int modeSelektor(Eigen::Vector3d object_pos, Eigen::Vector3d object_pos_init, Eigen::Vector3d object_vel, Eigen::Vector3d predict_pos, double ETA, Eigen::Vector3d ee_pos, Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector4d hittable_params, const int prev_mode){
  int mode = prev_mode;           // if none of the conditions are met, mode remains the same

  Eigen::Vector3d d_center = center2 - center1;
  Eigen::Vector3d d_points = center2 - object_pos;
  Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};
  
  bool cur_hittable, cur_too_far, pred_hittable, pred_too_far;
  std::tie(cur_hittable, cur_too_far) = hittable(object_pos, center1, center2, hittable_params);

  std::tie(pred_hittable, pred_too_far) = hittable(predict_pos, center1, center2, hittable_params);

  bool too_far;
  if (!pred_hittable && pred_too_far){too_far = true;}
  else {too_far = false;}

  bool too_close;
  if (!pred_hittable && !pred_too_far){too_close = true;}
  else {too_close = false;}

  bool moving;
  if (object_vel.norm() > 0.1){moving = true;}
  else {moving = false;}

  bool towards;
  if (object_vel.dot(d_center) < -0.05){towards = true;} //
  else {towards = false;}

  bool ee_ready;
  if ((ee_pos-(predict_pos - ee_offset[0]*d_points/d_points.norm() + v_offset)).norm() < 0.05){ee_ready = true;}
  else {ee_ready = false;}

  bool ee_hit;
  if (ee_pos.dot(d_center) > object_pos_init.dot(d_center)-0.13){ee_hit = true;}
  else {ee_hit = false;}

  
  

  //std::cout << "cur_hittable: " << cur_hittable << "\n";
  //std::cout << "too_far:      " << too_far << "\n";
  //std::cout << "too_close:    " << too_close << "\n";
  //std::cout << "towards:      " << towards << "\n";
  //std::cout << "ee_ready:     " << ee_ready << "\n";




  switch (prev_mode){
    case 1:   //track
      if (too_far && ETA < 3) {mode = 2;}                                         //if object will go too far, try to stop it
      if (pred_hittable && ETA < 0.3 && ee_ready) {mode = 3;}             //if object will be in feasible position and stops in 0.5s and ee is in correct position, go to hit
      if (too_close && ETA < 3) {mode = 5;}                                       //if object will not make it into reach, give up and go to rest
      //if (!cur_hittable) {mode = 5;}
      break;

    case 2:   //stop
      if (!towards || pred_hittable) {mode = 1;}                            //if the object no longer moves towards, it has been stopped succesfully so go to track for correct ee
    
    case 3:   //hit
      if (!towards && moving) {mode = 4;}                           //|| ee_hit == true       //if object starts moving because it is hit, go to post hit and initialize kalman                                          
      break;

    case 4:   //post hit
      if (!cur_hittable || towards || !moving) {mode = 5;}                            //if object has left the range of arm, go to rest
      break;

    case 5:   //rest
      if (too_far && ETA < 3) {mode = 2;}                                           //if object will go too far, try to stop it
      if (pred_hittable && ETA < 0.3 && ee_ready) {mode = 3;}               //same as when being in tracking mode, since mode is initialized in rest
      if (pred_hittable && ETA < 3 && !ee_ready) {mode = 1;}                //if object is going to be hittable but ee is not in the right position, lets track! 
      break;
  }
  return mode;
}

int maniModeSelektor(Eigen::Vector3d object_pos, Eigen::Vector3d object_pos_init, Eigen::Vector3d object_vel, Eigen::Vector3d ee_pos, Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::Vector2d ee_offset, Eigen::Vector4d hittable_params, const int prev_mode, const int key_ctrl, const int iiwa_no){
  int mode = prev_mode;           // if none of the conditions are met, mode remains the same
  
  Eigen::Vector3d d_center = center2 - center1;
  Eigen::Vector3d d_points = center2 - object_pos;
  Eigen::Vector3d v_offset = {0.0, 0.0, ee_offset[1]};
  
  bool cur_hittable, cur_too_far;
  std::tie(cur_hittable, cur_too_far) = hittable(object_pos, center1, center2, hittable_params);


  bool moving;
  if (object_vel.norm() > 0.07){moving = true;}
  else {moving = false;}

  bool towards;
  if (object_vel.dot(d_center) < -0.05){towards = true;}
  else {towards = false;}

  bool ee_ready;
  if ((ee_pos-(object_pos - ee_offset[0]*d_points/d_points.norm() + v_offset)).norm() < 0.02){ee_ready = true;}
  else {ee_ready = false;}

  bool ee_hit;
  if (ee_pos.dot(d_center) > object_pos_init.dot(d_center)){ee_hit = true;}
  else {ee_hit = false;}

  if (iiwa_no == 1){
    std::cout << (ee_pos-(object_pos - ee_offset[0]*d_points/d_points.norm() + v_offset)).norm() << std::endl;
  }

  switch (prev_mode){
    case 1: //track
      if (cur_hittable && ee_ready && key_ctrl == iiwa_no) {mode = 3;}
      if (!cur_hittable) {mode = 5;}
      if (key_ctrl == 3) {mode = 5;}
      break;
      
    case 3:   //hit
      if (!towards && moving || ee_hit == true) {mode = 4;}                                   //if object starts moving because it is hit, go to post hit and initialize kalman                                          
      if (key_ctrl == 3) {mode = 5;}
      break;

    case 4:   //post hit
      if (!cur_hittable || towards) {mode = 5;}                            //if object has left the range of arm, go to rest
      if (key_ctrl == 3) {mode = 5;}
      break;

    case 5:   //rest
      if (cur_hittable && ee_ready && key_ctrl == iiwa_no) {mode = 3;}   //same as when being in tracking mode, since mode is initialized in rest
      if (cur_hittable && !ee_ready && key_ctrl == iiwa_no) {mode = 1;}
      break;
  }
  return mode;
}
