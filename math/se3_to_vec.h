#ifndef SE3_TO_VEC_H
#define SE3_TO_VEC_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix<double, 6, 1> se3ToVec(const Eigen::Matrix4d &se3mat);
}

#endif // SE3_TO_VEC_H
