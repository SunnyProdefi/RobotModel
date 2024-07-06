#ifndef TRANS_INV_H
#define TRANS_INV_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix4d TransInv(const Eigen::Matrix4d &T);
}

#endif // TRANS_INV_H
