#include "../include/kalman.h"
double dt = 1.0/100;

Eigen::MatrixXd syst(6,6);
Eigen::MatrixXd C(3,6);
Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
Eigen::Matrix3d Z3 = Eigen::Matrix3d::Zero();


Eigen::Matrix3d V = 1*Eigen::Matrix3d::Identity();




Eigen::MatrixXd covarUpdate(Eigen::Matrix<double,6,6> P_prior){
    Eigen::Matrix<double, 6,6> P_pred;
    Eigen::Matrix<double, 6,6> P_post;

    syst << I3, I3*dt/0.3, Z3, I3;
    C << I3, Z3;
    
    Eigen::Matrix<double, 6,6> W;
    W << 0.2*I3, Z3, Z3, 1*I3;


    P_pred = syst*P_prior*syst.transpose() + W;
    P_post = (P_pred.inverse() + C.transpose()*V.inverse()*C).inverse();
    //P_post = P_pred - P_pred*C.transpose()*(C*P_pred*C.transpose()+V).inverse()*C*P_pred;

    return P_post;
};





Eigen::MatrixXd stateUpdate(Eigen::Vector3d object_vel, Eigen::Matrix<double,6,1> s_prior, Eigen::Matrix<double,6,6> P_post){
    Eigen::Matrix<double, 6,3> L;
    Eigen::Matrix<double, 6,1> s_pred;
    Eigen::Matrix<double, 6,1> s_post;

    syst << I3, I3*dt/0.3, Z3, I3;
    C << I3, Z3;
    
    
    //L = P_post*C.transpose()*(C*P_post*C.transpose()+V).inverse();

    s_pred = syst*s_prior;                             //prediction
    s_post = s_pred + P_post*C.transpose()*V.inverse()*(object_vel - C*s_pred);
    //s_post = s_pred + L*(object_vel - C*s_pred);    //correction

    return s_post;
};