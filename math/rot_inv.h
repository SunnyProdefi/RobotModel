#ifndef ROT_INV_H
#define ROT_INV_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix3d RotInv(const Eigen::Matrix3d &R);
}

#endif // ROT_INV_H
