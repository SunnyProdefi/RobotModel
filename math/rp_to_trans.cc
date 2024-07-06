#include "rp_to_trans.h"

namespace math {
Eigen::Matrix4d RpToTrans(const Eigen::Matrix3d &R, const Eigen::Vector3d &p) {
  Eigen::Matrix4d T;
  T << R(0, 0), R(0, 1), R(0, 2), p(0), R(1, 0), R(1, 1), R(1, 2), p(1),
      R(2, 0), R(2, 1), R(2, 2), p(2), 0, 0, 0, 1;
  return T;
}
} // namespace math
