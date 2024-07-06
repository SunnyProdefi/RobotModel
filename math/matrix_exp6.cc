#include "matrix_exp6.h"
#include "matrix_exp3.h" // Assuming this function is correctly implemented
#include "so3_to_vec.h"  // Convert so(3) to a vector
#include "vec_to_so3.h"  // Convert a vector to so(3)

namespace math {
Eigen::Matrix4d MatrixExp6(const Eigen::Matrix4d &se3mat) {
  Eigen::Matrix3d so3mat = se3mat.block<3, 3>(0, 0);
  Eigen::Vector3d v = se3mat.block<3, 1>(0, 3);
  Eigen::Vector3d omgtheta = so3ToVec(so3mat);

  double theta = omgtheta.norm();

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  if (theta < 1e-6) { // Near zero, indicating no rotation, direct translation
    T.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    T.block<3, 1>(0, 3) = v;
  } else {
    Eigen::Matrix3d R = MatrixExp3(so3mat);
    Eigen::Matrix3d omgmat = VecToso3(omgtheta / theta);

    Eigen::Matrix3d V = Eigen::Matrix3d::Identity() * theta +
                        (1 - cos(theta)) * omgmat +
                        (theta - sin(theta)) * omgmat * omgmat;

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = V * (v / theta);
  }

  return T;
}
} // namespace math
