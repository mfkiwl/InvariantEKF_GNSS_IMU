#ifndef _LIE_GROUP_H
#define _LIE_GROUP_H 
#include <Eigen/Dense>

namespace simple_ekf {

extern const double TOLERANCE;

Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);

} // end simple_ekf namespace
#endif 
