#include "project_to_so3.h"

namespace math {
Eigen::Matrix3d ProjectToSO3(const Eigen::Matrix3d &mat) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeFullU |
                                                 Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

  // Ensure a right-handed coordinate system
  if (R.determinant() < 0) {
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    R = U * Eigen::Vector3d(1, 1, -1).asDiagonal() * V.transpose();
  }

  return R;
}
} // namespace math
