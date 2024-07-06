#include "test_if_se3.h"
#include "distance_to_se3.h"

namespace math {
bool TestIfSE3(const Eigen::Matrix4d &mat) {
  double threshold = 1e-3;              // Threshold for distance
  double distance = DistanceToSE3(mat); // Calculate the distance to SE(3)
  return distance <
         threshold; // Return true if the distance is less than the threshold
}
} // namespace math
