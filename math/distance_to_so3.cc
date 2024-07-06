#include "distance_to_so3.h"

namespace math {
double DistanceToSO3(const Eigen::Matrix3d &mat) {
  double det = mat.determinant();
  if (det > 0) {
    Eigen::Matrix3d error = mat.transpose() * mat - Eigen::Matrix3d::Identity();
    return error.norm(); // Frobenius norm by default in Eigen
  } else {
    return 1e+9; // A large number
  }
}
} // namespace math
