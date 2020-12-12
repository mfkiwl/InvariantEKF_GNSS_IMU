#ifndef __EKF_H_
#define __EKF_H_
#include <Eigen/Dense>
#include "CarState.h"
#include "LieGroup.h"

namespace simple_ekf{

class Ekf{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Ekf();
    Ekf(CarState state);
    CarState getState();
    
    void Propagate(const Eigen::Matrix<double,6,1>& m, double dt);
    void Correct(const Eigen::MatrixXd H, const Eigen::VectorXd Z, const Eigen::MatrixXd N);

    private:
    CarState state_;

};


} //end namespace simple_ekf

#endif