#include "axis_ang6.h"
#include "near_zero.h"
namespace math {

std::pair<Eigen::Matrix<double, 6, 1>, double>
AxisAng6(const Eigen::Matrix<double, 6, 1> &expc6) {
  double theta = expc6.head<3>().norm(); // Norm of angular part
  if (NearZero(theta)) {
    theta = expc6.tail<3>()
                .norm(); // Norm of linear part if angular part is near zero
  }

  Eigen::Matrix<double, 6, 1> S = expc6 / theta; // Normalize the 6-vector
  return {S, theta};
}
} // namespace math
