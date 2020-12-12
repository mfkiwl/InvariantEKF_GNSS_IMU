#include "CarState.h"


namespace simple_ekf {

CarState::CarState(){
    P_ = 0.1* Eigen::MatrixXd::Identity(15, 15);
    X_ = Eigen::MatrixXd::Zero(15, 1);
}

void CarState::setRotation(const Eigen::Matrix3d& R){
    R_ = R;
}
void CarState::setVelocity(const Eigen::Vector3d& v){
    v_ = v;
}
void CarState::setPosition(const Eigen::Vector3d& p){
    p_ = p;
}
void CarState::setGyroscopeBias(const Eigen::Vector3d& bg){
    bg_ = bg;
}
void CarState::setAccelerometerBias(const Eigen::Vector3d& ba){
    ba_ = ba;
}
void CarState::setGravity(const Eigen::Vector3d& g){
    g_ = g;
}

void CarState::setGyroscopeBiasNoise(const Eigen::Vector3d& std){
    Qbg_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); 
}
void CarState::setAccelerometerBiasNoise(const Eigen::Vector3d& std){
    Qba_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); 
}
void CarState::setGyroscopeNoise(const Eigen::Vector3d& std){
    Qg_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); 
}
void CarState::setAccelerometerNoise(const Eigen::Vector3d& std){
    Qa_ << std(0)*std(0),0,0, 0,std(1)*std(1),0, 0,0,std(2)*std(2); 
}

void CarState::setP(const Eigen::Matrix<double, 15, 15>& P){
    P_ = P;
}

void CarState::setX(const Eigen::Matrix<double, 15, 1>& X){
    X_ = X;
}


const Eigen::Matrix3d CarState::getRotation(){
    return R_;
}
const Eigen::Vector3d CarState::getVelocity(){
    return v_;
}
const Eigen::Vector3d CarState::getPosition(){
    return p_;
}
const Eigen::Vector3d CarState::getGyroscopeBias(){
    return bg_;
}
const Eigen::Vector3d CarState::getAccelerometerBias(){
    return ba_;
}
const Eigen::Vector3d CarState::getGravity(){
    return g_;
}

const Eigen::Matrix3d CarState::getGyroscopeCov(){
    return Qg_;
}
const Eigen::Matrix3d CarState::getAccelerometerCov(){
    return Qa_;
}
const Eigen::Matrix3d CarState::getGyroscopeBiasCov(){
    return Qbg_;
}
const Eigen::Matrix3d CarState::getAccelerometerBiasCov(){
    return Qba_;
}
const Eigen::MatrixXd CarState::getX(){
    return X_;
}
const Eigen::MatrixXd CarState::getP(){
    return P_;
}

}