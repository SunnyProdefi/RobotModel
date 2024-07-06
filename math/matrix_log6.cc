#include "matrix_log6.h"
#include "matrix_log3.h"
#include "trans_to_rp.h"
#include <cmath>

namespace math {
Eigen::Matrix4d MatrixLog6(const Eigen::Matrix4d &T) {
  auto [R, p] = TransToRp(T);
  Eigen::Matrix3d omgmat = MatrixLog3(R);
  double theta = acos((R.trace() - 1) / 2);

  Eigen::Matrix4d expmat = Eigen::Matrix4d::Zero();

  if (theta == 0) { // This checks if the rotation part is identity
    expmat.block<3, 1>(0, 3) = p;
  } else if (std::fabs(theta) < 1e-6) { // Near zero, indicating a small angle
    expmat.block<3, 3>(0, 0) = omgmat;
    expmat.block<3, 1>(0, 3) = p;
  } else {
    Eigen::Matrix3d A =
        Eigen::Matrix3d::Identity() - omgmat / 2 +
        (1 / theta - 1 / (2 * tan(theta / 2))) * omgmat * omgmat / theta;
    expmat.block<3, 3>(0, 0) = omgmat;
    expmat.block<3, 1>(0, 3) = A * p;
  }

  return expmat;
}
} // namespace math
