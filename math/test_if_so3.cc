#include "test_if_so3.h"
#include "distance_to_so3.h"

namespace math {
bool TestIfSO3(const Eigen::Matrix3d &mat) {
  double distance = DistanceToSO3(mat);
  return distance < 1e-3;
}
} // namespace math
