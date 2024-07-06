#ifndef MATRIX_EXP3_H
#define MATRIX_EXP3_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d &so3mat);
}

#endif // MATRIX_EXP3_H
