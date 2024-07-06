#include "vec_to_se3.h"
#include "vec_to_so3.h" // Assuming the existence of this header file for VecToso3 function

namespace math {
Eigen::Matrix4d VecTose3(const Eigen::Matrix<double, 6, 1> &V) {
  Eigen::Vector3d omega = V.head<3>(); // Extracts the angular velocity part
  Eigen::Vector3d v = V.tail<3>();     // Extracts the linear velocity part
  Eigen::Matrix3d omega_mat = VecToso3(omega);
  Eigen::Matrix4d se3mat;

  se3mat.block<3, 3>(0, 0) = omega_mat;
  se3mat.block<3, 1>(0, 3) = v;
  se3mat.row(3) << 0, 0, 0, 0;

  return se3mat;
}
} // namespace math
