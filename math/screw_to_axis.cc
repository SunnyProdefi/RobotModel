#include "screw_to_axis.h"

namespace math {
Eigen::Matrix<double, 6, 1> ScrewToAxis(const Eigen::Vector3d &q,
                                        const Eigen::Vector3d &s, double h) {
  Eigen::Matrix<double, 6, 1> S;
  Eigen::Vector3d m =
      q.cross(s) + h * s; // Compute the moment (cross product) plus pitch term
  S << s, m;
  return S;
}
} // namespace math
