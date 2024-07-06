#include "so3_to_vec.h"

namespace math {
Eigen::Vector3d so3ToVec(const Eigen::Matrix3d &so3mat) {
  Eigen::Vector3d omg;
  omg << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
  return omg;
}
} // namespace math
