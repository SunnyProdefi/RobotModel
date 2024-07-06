#ifndef ADJOINT_H
#define ADJOINT_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix<double, 6, 6> Adjoint(const Eigen::Matrix4d &T);
}

#endif // ADJOINT_H
