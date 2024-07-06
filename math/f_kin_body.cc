#include "f_kin_body.h"
#include "matrix_exp6.h"
#include "vec_to_se3.h"

namespace math {
Eigen::Matrix4d FKinBody(const Eigen::Matrix4d &M,
                         const std::vector<Eigen::Matrix<double, 6, 1>> &Blist,
                         const std::vector<double> &thetalist) {
  Eigen::Matrix4d T = M;
  for (size_t i = 0; i < thetalist.size(); ++i) {
    Eigen::Matrix<double, 6, 1> se3 = Blist[i] * thetalist[i];
    T *= MatrixExp6(VecTose3(se3));
  }
  return T;
}
} // namespace math
