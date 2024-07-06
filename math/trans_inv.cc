#include "trans_inv.h"

namespace math {
Eigen::Matrix4d TransInv(const Eigen::Matrix4d &T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d p = T.block<3, 1>(0, 3);
  Eigen::Matrix3d Rt = R.transpose();
  Eigen::Vector3d t = -(Rt * p);

  Eigen::Matrix4d invT;
  invT << Rt(0, 0), Rt(0, 1), Rt(0, 2), t(0), Rt(1, 0), Rt(1, 1), Rt(1, 2),
      t(1), Rt(2, 0), Rt(2, 1), Rt(2, 2), t(2), 0, 0, 0, 1;
  return invT;
}
} // namespace math
