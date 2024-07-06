#ifndef SO3_TO_VEC_H
#define SO3_TO_VEC_H

#include <Eigen/Dense>

namespace math {
Eigen::Vector3d so3ToVec(const Eigen::Matrix3d &so3mat);
}

#endif // SO3_TO_VEC_H
