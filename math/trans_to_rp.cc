#include "trans_to_rp.h"

namespace math {
std::pair<Eigen::Matrix3d, Eigen::Vector3d>
TransToRp(const Eigen::Matrix4d &T) {
  Eigen::Matrix3d R = T.block<3, 3>(
      0, 0); // Extract the top-left 3x3 block for the rotation matrix
  Eigen::Vector3d p = T.block<3, 1>(
      0, 3); // Extract the top-right 3x1 block for the position vector
  return {R, p};
}
} // namespace math
