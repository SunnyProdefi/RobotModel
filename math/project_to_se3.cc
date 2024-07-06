#include "project_to_se3.h"
#include "project_to_so3.h" // Ensure this function is defined to project a 3x3 matrix to SO(3)

namespace math {
Eigen::Matrix4d ProjectToSE3(const Eigen::Matrix4d &mat) {
  Eigen::Matrix3d R = mat.block<3, 3>(0, 0);
  Eigen::Vector3d p = mat.block<3, 1>(0, 3);

  // Project the top-left 3x3 part to SO(3)
  Eigen::Matrix3d projectedR = ProjectToSO3(R);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = projectedR;
  T.block<3, 1>(0, 3) = p; // Keep the translation component as is

  return T;
}
} // namespace math
