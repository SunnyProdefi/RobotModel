#include "vec_to_so3.h"

namespace math {
Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg) {
  Eigen::Matrix3d so3mat;
  so3mat << 0, -omg(2), omg(1), omg(2), 0, -omg(0), -omg(1), omg(0), 0;
  return so3mat;
}
} // namespace math
