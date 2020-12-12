#include "EKF.h"
#include <iostream>

namespace simple_ekf{

Ekf::Ekf(){}

Ekf::Ekf(CarState state):state_(state){}

CarState Ekf::getState(){
    return state_;
}

void Ekf::Propagate(const Eigen::Matrix<double,6,1>& m, double dt){

    // Get IMU Measurements
    Eigen::Vector3d w = m.head(3) - state_.getGyroscopeBias();     // Angular Velocity
    Eigen::Vector3d a = m.tail(3) - state_.getAccelerometerBias(); // Linear Acceleration

    // Extract State
    Eigen::Matrix3d R = state_.getRotation();
    Eigen::Vector3d v = state_.getVelocity();
    Eigen::Vector3d p = state_.getPosition();
    Eigen::Vector3d g = state_.getGravity();
    
    // Strapdown IMU Motion Model
    Eigen::Vector3d phi = w * dt; 
    Eigen::Matrix3d R_pred = R * Exp_SO3(phi);
    Eigen::Vector3d v_pred = v + (R*a + g)*dt;
    Eigen::Vector3d p_pred = p + v*dt + 0.5*(R*a + g)*dt*dt;

    // Set new state 
    state_.setRotation(R_pred);
    state_.setVelocity(v_pred);
    state_.setPosition(p_pred);

    // Propegate P 
    Eigen::MatrixXd X = state_.getX();
    Eigen::MatrixXd P = state_.getP();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(15, 15);
    
    A.block<3,3>(0,9) = -R;
    A.block<3,3>(3,0) = skew(R*a);
    A.block<3,3>(3,12) = R;
    A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(15,15);
    // Q.block<3,3>(0,0) = R * state_.getGyroscopeCov() * R.transpose(); 
    // Q.block<3,3>(3,3) = R * state_.getAccelerometerCov() * R.transpose();

    Q.block<3,3>(0,0) = state_.getGyroscopeCov(); 
    Q.block<3,3>(3,3) = state_.getAccelerometerCov();

    Q.block<3,3>(9,9) = state_.getGyroscopeBiasCov();
    Q.block<3,3>(12,12) = state_.getAccelerometerBiasCov();
    
    // Discretization
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15,15);
    Eigen::MatrixXd Phi = I + A*dt;
    Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Q*dt;
    state_.setP(P_pred);
    
    // std::cout<<  P_pred << std::endl << std::endl;
    // std::cout<<  A << std::endl;
    return;
}

void Ekf::Correct(const Eigen::MatrixXd H, const Eigen::VectorXd Z, const Eigen::MatrixXd N){
    
    // Compute Kalman Gain
    Eigen::MatrixXd P = state_.getP();
    Eigen::MatrixXd PHT = P * H.transpose();
    Eigen::MatrixXd S = H * PHT + N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Compute correction terms
    Eigen::VectorXd delta = K*Z;
    
    // Update state
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity() - skew(delta.segment(0,3));
    Eigen::Matrix3d R_hat = T.transpose() * state_.getRotation();
    Eigen::Vector3d v_hat = state_.getVelocity() - delta.segment(3, 3);
    Eigen::Vector3d p_hat = state_.getPosition() - delta.segment(6, 3);
    Eigen::Vector3d g_bias_hat = state_.getGyroscopeBias() + delta.segment(9,3);
    Eigen::Vector3d a_bias_hat = state_.getAccelerometerBias() + delta.segment(12,3);

    state_.setRotation(R_hat);
    state_.setVelocity(v_hat);
    state_.setPosition(p_hat); 
    state_.setGyroscopeBias(g_bias_hat);
    state_.setAccelerometerBias(a_bias_hat);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(15,15) - K*H;
    Eigen::MatrixXd P_new = IKH * P;
    state_.setP(P_new); 

}

}

