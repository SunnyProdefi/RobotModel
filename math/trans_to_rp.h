#ifndef TRANS_TO_RP_H
#define TRANS_TO_RP_H

#include <Eigen/Dense>
#include <utility> // For std::pair

namespace math {
std::pair<Eigen::Matrix3d, Eigen::Vector3d> TransToRp(const Eigen::Matrix4d &T);
}

#endif // TRANS_TO_RP_H
