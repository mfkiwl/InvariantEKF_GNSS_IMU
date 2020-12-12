#include "LieGroup.h"

namespace simple_ekf {

using namespace std;

const double TOLERANCE = 1e-10;

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    // Convert vector to skew-symmetric matrix
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    M << 0, -v[2], v[1],
         v[2], 0, -v[0], 
        -v[1], v[0], 0;
        return M;
}

Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w) {
    // Computes the vectorized exponential map for SO(3)
    Eigen::Matrix3d A = skew(w);
    double theta = w.norm();
    if (theta < TOLERANCE) {
        return Eigen::Matrix3d::Identity();
    } 
    Eigen::Matrix3d R =  Eigen::Matrix3d::Identity() + (sin(theta)/theta)*A + ((1-cos(theta))/(theta*theta))*A*A;
    return R;
}


} // end simple_ekf namespace