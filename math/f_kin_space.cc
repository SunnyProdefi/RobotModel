#include "f_kin_space.h"
#include "matrix_exp6.h"
#include "vec_to_se3.h"

namespace math {
Eigen::Matrix4d FKinSpace(const Eigen::Matrix4d &M,
                          const std::vector<Eigen::Matrix<double, 6, 1>> &Slist,
                          const std::vector<double> &thetalist) {
  Eigen::Matrix4d T = M;
  // Apply transformations in reverse order
  for (int i = thetalist.size() - 1; i >= 0; --i) {
    auto se3 = VecTose3(Slist[i] * thetalist[i]);
    T = MatrixExp6(se3) * T;
  }
  return T;
}
} // namespace math
