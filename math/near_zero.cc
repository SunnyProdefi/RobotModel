// near_zero.cc
#include "near_zero.h"
#include <cmath>

namespace math {
bool NearZero(double near) { return std::abs(near) < 1e-6; }
} // namespace math
