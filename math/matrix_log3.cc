#include "matrix_log3.h"
#include "near_zero.h"
#include "vec_to_so3.h"
#include <cmath>

namespace math {
Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d &R) {
  double acosInput = (R.trace() - 1) / 2.0;
  Eigen::Matrix3d so3mat;

  if (acosInput >= 1) {
    // The rotation is 0 degrees
    so3mat.setZero();
  } else if (acosInput <= -1) {
    // The rotation is 180 degrees
    Eigen::Vector3d omg;
    if (!NearZero(1 + R(2, 2))) {
      omg = (1 / sqrt(2 * (1 + R(2, 2)))) *
            Eigen::Vector3d(R(0, 2), 1 + R(2, 2), R(1, 2));
    } else if (!NearZero(1 + R(1, 1))) {
      omg = (1 / sqrt(2 * (1 + R(1, 1)))) *
            Eigen::Vector3d(1 + R(1, 1), R(0, 1), R(2, 1));
    } else {
      omg = (1 / sqrt(2 * (1 + R(0, 0)))) *
            Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(0, 0));
    }
    so3mat = VecToso3(M_PI * omg);
  } else {
    double theta = acos(acosInput);
    so3mat = theta * (1 / (2 * sin(theta))) * (R - R.transpose());
  }

  return so3mat;
}
} // namespace math
