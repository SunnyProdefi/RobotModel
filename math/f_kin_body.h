#ifndef F_KIN_BODY_H
#define F_KIN_BODY_H

#include <Eigen/Dense>
#include <vector>

namespace math {
Eigen::Matrix4d FKinBody(const Eigen::Matrix4d &M,
                         const std::vector<Eigen::Matrix<double, 6, 1>> &Blist,
                         const std::vector<double> &thetalist);
}

#endif // F_KIN_BODY_H
