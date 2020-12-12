
#ifndef INEKF_H
#define INEKF_H 
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>

#include <algorithm>
#include "RobotState.h"
#include "NoiseParams.h"
#include "LieGroup.h"

namespace inekf {
class Observation {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, Eigen::MatrixXd& N, Eigen::MatrixXd& PI);
        bool empty();

        Eigen::VectorXd Y;
        Eigen::VectorXd b;
        Eigen::MatrixXd H;
        Eigen::MatrixXd N;
        Eigen::MatrixXd PI;

        friend std::ostream& operator<<(std::ostream& os, const Observation& o);  
};


class InEKF {
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        InEKF();
        InEKF(NoiseParams params);
        InEKF(RobotState state);
        InEKF(RobotState state, NoiseParams params);

        RobotState getState();
        NoiseParams getNoiseParams();
        void setState(RobotState state);
        void setNoiseParams(NoiseParams params);

        void RightPropagate(const Eigen::Matrix<double,6,1>& m, double dt);
        void LeftPropagate(const Eigen::Matrix<double,6,1>&m, double dt);
        void Correct(const Observation& obs);
        void RightCorrect(const Eigen::MatrixXd H, const Eigen::VectorXd Z, const Eigen::MatrixXd N);
        void LeftCorrect(const Eigen::MatrixXd H, const Eigen::VectorXd Z, const Eigen::MatrixXd N);
        void LeftToRight(void);
        void RightToLeft(void);

    private:
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_; // Gravity
};

} // end inekf namespace
#endif 
