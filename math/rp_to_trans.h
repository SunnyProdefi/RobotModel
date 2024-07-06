#ifndef RP_TO_TRANS_H
#define RP_TO_TRANS_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix4d RpToTrans(const Eigen::Matrix3d &R, const Eigen::Vector3d &p);
}

#endif // RP_TO_TRANS_H
