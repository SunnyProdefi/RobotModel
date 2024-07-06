#include "se3_to_vec.h"

namespace math {
Eigen::Matrix<double, 6, 1> se3ToVec(const Eigen::Matrix4d &se3mat) {
  Eigen::Matrix<double, 6, 1> V;
  // Extracting the angular velocity vector from the skew-symmetric part
  V(0) = se3mat(2, 1); // omega_x
  V(1) = se3mat(0, 2); // omega_y
  V(2) = se3mat(1, 0); // omega_z
  // Extracting the linear velocity vector from the last column
  V(3) = se3mat(0, 3); // v_x
  V(4) = se3mat(1, 3); // v_y
  V(5) = se3mat(2, 3); // v_z
  return V;
}
} // namespace math
