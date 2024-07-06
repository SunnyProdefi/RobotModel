#include "distance_to_se3.h"
#include "trans_to_rp.h"
#include <cmath> // For fabs

namespace math {
double DistanceToSE3(const Eigen::Matrix4d &mat) {
  auto [R, p] = TransToRp(mat);

  if (R.determinant() <= 0) {
    return 1e+9; // A large number to indicate invalid rotation matrix
  } else {
    Eigen::Matrix4d modified = Eigen::Matrix4d::Zero();
    modified.block<3, 3>(0, 0) = R.transpose() * R;
    modified.block<1, 4>(3, 0) = mat.block<1, 4>(3, 0);

    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    return (modified - I)
        .norm(); // Frobenius norm of the difference from identity
  }
}
} // namespace math
