#ifndef VEC_TO_SE3_H
#define VEC_TO_SE3_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix4d VecTose3(const Eigen::Matrix<double, 6, 1> &V);
}

#endif // VEC_TO_SE3_H
