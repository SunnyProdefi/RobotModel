#include "axis_ang3.h"

namespace math {
std::pair<Eigen::Vector3d, double> AxisAng3(const Eigen::Vector3d &expc3) {
  double theta = expc3.norm();
  Eigen::Vector3d omghat = expc3 / theta;
  return std::make_pair(omghat, theta);
}
} // namespace math
