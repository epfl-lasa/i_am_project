#include "../include/kalman.h"



std::tuple<Eigen::MatrixXd, Eigen::VectorXd> kalmanFilter(Eigen::MatrixXd P_prior, Eigen::VectorXd s_prior, Eigen::VectorXd u, Eigen::VectorXd y, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd W, Eigen::MatrixXd V){
  //Covariance update
  Eigen::MatrixXd P_pred;
  Eigen::MatrixXd P_post;

  P_pred = A*P_prior*A.transpose() + W;                                             //propagate information from previous state to current through system
  P_post = (P_pred.inverse() + C.transpose()*V.inverse()*C).inverse();                    //correct current state
  //P_post = P_pred - P_pred*C.transpose()*(C*P_pred*C.transpose()+V).inverse()*C*P_pred;


  //State update
  Eigen::VectorXd s_pred;
  Eigen::VectorXd s_post;

  //Eigen::Matrix<double, 6,3> L;
  //L = P_post*C.transpose()*(C*P_post*C.transpose()+V).inverse();
  s_pred = A*s_prior + B*u;                                                                  //propagate information from previous state to current through system
  s_post = s_pred + P_post*C.transpose()*V.inverse()*(y - C*s_pred);                 //correct current state given information of current measurement
  //s_post = s_pred + L*(object_vel - C*s_pred);    //correction

  return std::make_tuple(P_post,s_post);
};



std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::Vector3d, double> predictPos(Eigen::MatrixXd P_obj_lin, Eigen::VectorXd s_obj_lin, Eigen::Vector3d hit_force, Eigen::Vector3d object_pos, double dt, double mass){
  //Kalman stuff first
  Eigen::MatrixXd syst(9,9);
  Eigen::MatrixXd B(9,3);
  Eigen::MatrixXd C(3,9);
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Z3 = Eigen::Matrix3d::Zero();
  syst << I3, dt*I3,  0.5*dt*dt*I3, 
          Z3,    I3,         dt*I3, 
          Z3,    Z3,            I3; //
  B << dt*dt/(2*mass)*I3, dt/mass*I3, Z3;
  C << I3, Z3, Z3;

  Eigen::Matrix<double, 9,9> W;
  //Eigen::Matrix<double, 9,9> Wa;
  //Wa << Z3, Z3, Z3, Z3, 0.2*I3, Z3, Z3, Z3, I3;
  //W << syst*Wa*syst.transpose();
  W << 0.01*I3, Z3, Z3, Z3, 0.1*I3, Z3, Z3, Z3, 1*I3;
  /*W << 0.25*dt*dt*dt*dt*I3, 0.5*dt*dt*dt*I3, 0.5*dt*dt*I3, 
           0.5*dt*dt*dt*I3,        dt*dt*I3,        dt*I3, 
              0.5*dt*dt*I3,           dt*I3,         1*I3;*/
  Eigen::Matrix<double, 3,3> V = 0.1*Eigen::Matrix<double, 3,3>::Identity();

  std::tie(P_obj_lin, s_obj_lin) = kalmanFilter(P_obj_lin, s_obj_lin, hit_force, object_pos, syst, B, C, W, V); //estimate current covar and state given priors and measurement

  //Prediction next given estimated states
  Eigen::Vector3d vel_est;
  Eigen::Vector3d acc_est;
  double ETA;

  vel_est << s_obj_lin[3], s_obj_lin[4], s_obj_lin[5];
  acc_est << s_obj_lin[6], s_obj_lin[7], s_obj_lin[8];
  ETA = vel_est.norm()/acc_est.norm();
  

  Eigen::Vector3d predict_pos;
  for (int i = 0; i < 3; i++) {
    predict_pos[i] = object_pos[i] - 0.5*vel_est[i]*vel_est[i]/acc_est[i];
  }


  return std::make_tuple(P_obj_lin, s_obj_lin, predict_pos, ETA);
}


/*
std::tuple<Eigen::MatrixXd, Eigen::VectorXd, double> predictTh(Eigen::MatrixXd P_obj_th, Eigen::VectorXd s_obj_th, double object_th, double dt, double inertia){
  //Kalman stuff first
  Eigen::Matrix3d syst;
  Eigen::MatrixXd C(2,3);
  syst << 1.0, dt, 0.0, 0.0, 1.0, 1.0*dt/inertia, 0.0, 0.0, 1.0;
  C << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Eigen::Matrix3d W;
  W << 5.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix2d V = Eigen::Matrix2d::Identity();

  Eigen::Vector2d object_th_vec;
  object_th_vec << object_th, 0.0;
  std::tie(P_obj_th, s_obj_th) = kalmanFilter(P_obj_th,s_obj_th,object_th_vec,syst,C,W,V); //estimate current covar and state given priors and measurement

  //Prediction next given estimated states
  double th_dot_est;
  double M_est;

  th_dot_est = s_obj_th[1];
  M_est = s_obj_th[2];
  

  double predict_th;
  for (int i = 0; i < 3; i++) {
    predict_th = object_th - 0.5*inertia*th_dot_est*th_dot_est/M_est;
  }


  return std::make_tuple(P_obj_th, s_obj_th, predict_th);
}*/