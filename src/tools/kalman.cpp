#include "../include/kalman.h"



std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> kalmanFilter(Eigen::Matrix<double,6,6> P_prior, Eigen::Matrix<double,6,1> s_prior, Eigen::Vector3d object_vel){
    double dt = 1.0/100;

    Eigen::MatrixXd syst(6,6);
    Eigen::MatrixXd C(3,6);
    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Z3 = Eigen::Matrix3d::Zero();
    syst << I3, I3*dt/0.3, Z3, I3;
    C << I3, Z3;

    Eigen::Matrix<double, 6,6> W;
    W << 0.2*I3, Z3, Z3, 1*I3;
    Eigen::Matrix3d V = 1*Eigen::Matrix3d::Identity();



    //Covariance update
    Eigen::Matrix<double, 6,6> P_pred;
    Eigen::Matrix<double, 6,6> P_post;

    P_pred = syst*P_prior*syst.transpose() + W;                                             //propagate information from previous state to current through system
    P_post = (P_pred.inverse() + C.transpose()*V.inverse()*C).inverse();                    //correct current state
    //P_post = P_pred - P_pred*C.transpose()*(C*P_pred*C.transpose()+V).inverse()*C*P_pred;


    //State update
    Eigen::Matrix<double, 6,1> s_pred;
    Eigen::Matrix<double, 6,1> s_post;
    
    //Eigen::Matrix<double, 6,3> L;
    //L = P_post*C.transpose()*(C*P_post*C.transpose()+V).inverse();
    s_pred = syst*s_prior;                                                                  //propagate information from previous state to current through system
    s_post = s_pred + P_post*C.transpose()*V.inverse()*(object_vel - C*s_pred);             //correct current state given information of current measurement
    //s_post = s_pred + L*(object_vel - C*s_pred);    //correction

    return std::make_tuple(P_post,s_post);
};



std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Vector3d, double> predictPos(Eigen::Matrix<double,6,6> P, Eigen::Matrix<double,6,1> state, Eigen::Vector3d object_pos, Eigen::Vector3d object_vel, double mass){
  std::tie(P,state) = kalmanFilter(P,state,object_vel); //estimate current covar and state given priors and measurement

  Eigen::Vector3d vel_est;
  Eigen::Vector3d f_est;
  double ETA;

  vel_est << state[0], state[1], state[2];
  f_est << state[3], state[4], state[5];
  ETA = mass*vel_est.norm()/f_est.norm();
  

  Eigen::Vector3d predict_pos;
  for (int i = 0; i < 3; i++) {
    predict_pos[i] = object_pos[i] - 0.5*mass*vel_est[i]*vel_est[i]/f_est[i];
  }


  return std::make_tuple(P, state, predict_pos, ETA);
}