#include "adjoint.h"
#include "trans_to_rp.h"
#include "vec_to_so3.h"

namespace math {
Eigen::Matrix<double, 6, 6> Adjoint(const Eigen::Matrix4d &T) {
  auto [R, p] = TransToRp(T);
  Eigen::Matrix<double, 6, 6> AdT;
  Eigen::Matrix3d p_hat = VecToso3(p);

  AdT.block<3, 3>(0, 0) = R;
  AdT.block<3, 3>(0, 3).setZero();
  AdT.block<3, 3>(3, 0) = p_hat * R;
  AdT.block<3, 3>(3, 3) = R;

  return AdT;
}
} // namespace math
