#ifndef __CAR_STATE_H_
#define __CAR_STATE_H_

#include <Eigen/Dense>

namespace simple_ekf {

class CarState{
    public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CarState();

    void setX(const Eigen::MatrixXd& X);

    // void setP(const Eigen::MatrixXd& P);
    void setRotation(const Eigen::Matrix3d& R);
    void setVelocity(const Eigen::Vector3d& v);
    void setPosition(const Eigen::Vector3d& p);
    void setGyroscopeBias(const Eigen::Vector3d& bg);
    void setAccelerometerBias(const Eigen::Vector3d& ba);
    void setGyroscopeBiasNoise(const Eigen::Vector3d& n);
    void setAccelerometerBiasNoise(const Eigen::Vector3d& n);
    void setGyroscopeNoise(const Eigen::Vector3d& n);
    void setAccelerometerNoise(const Eigen::Vector3d& n);    
    void setGravity(const Eigen::Vector3d& g);
    void setP(const Eigen::Matrix<double, 15, 15>& P);
    void setX(const Eigen::Matrix<double, 15, 1>& X);


    const Eigen::Matrix3d getRotation();
    const Eigen::Vector3d getVelocity();
    const Eigen::Vector3d getPosition();
    const Eigen::Vector3d getGyroscopeBias();
    const Eigen::Vector3d getAccelerometerBias();
    const Eigen::Vector3d getGravity();

    const Eigen::Matrix3d getGyroscopeCov();
    const Eigen::Matrix3d getAccelerometerCov();
    const Eigen::Matrix3d getGyroscopeBiasCov();
    const Eigen::Matrix3d getAccelerometerBiasCov();

    const Eigen::MatrixXd getX();
    const Eigen::MatrixXd getP();

    private:

    Eigen::Matrix3d R_;
    Eigen::Vector3d v_;
    Eigen::Vector3d p_;
    Eigen::Vector3d ba_;
    Eigen::Vector3d bg_;
    Eigen::Vector3d g_;

    Eigen::Matrix3d Qa_;
    Eigen::Matrix3d Qg_;
    Eigen::Matrix3d Qba_;
    Eigen::Matrix3d Qbg_;

    Eigen::Matrix<double, 15, 1> X_;
    Eigen::Matrix<double, 15, 15> P_;
};

} //end namespace simple_ekf


#endif