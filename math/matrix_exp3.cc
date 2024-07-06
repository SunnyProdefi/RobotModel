#include "matrix_exp3.h"
#include "axis_ang3.h"
#include "near_zero.h"
#include "so3_to_vec.h"
#include <cmath>
#include <iostream>

namespace math {
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d &so3mat) {
  auto omgtheta = so3ToVec(so3mat);

  if (NearZero(omgtheta.norm())) { // 使用 NearZero 函数的思想
    return Eigen::Matrix3d::Identity();
  } else {
    auto result = AxisAng3(omgtheta);
    auto theta = result.second;
    auto omgmat = so3mat / theta;
    return Eigen::Matrix3d::Identity() + sin(theta) * omgmat +
           (1 - cos(theta)) * omgmat * omgmat;
  }
}
} // namespace math
