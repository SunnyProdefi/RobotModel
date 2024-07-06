#ifndef VEC_TO_SO3_H
#define VEC_TO_SO3_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg);
}

#endif // VEC_TO_SO3_H
