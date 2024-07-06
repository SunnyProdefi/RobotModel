#include "rot_inv.h"

namespace math {
Eigen::Matrix3d RotInv(const Eigen::Matrix3d &R) { return R.transpose(); }
} // namespace math
