#include "normalize.h"
#include <cmath>

namespace math {
Eigen::VectorXd Normalize(const Eigen::VectorXd &V) { return V / V.norm(); }
} // namespace math
