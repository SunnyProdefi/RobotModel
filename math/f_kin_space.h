#ifndef F_KIN_SPACE_H
#define F_KIN_SPACE_H

#include <Eigen/Dense>
#include <vector>

namespace math {
Eigen::Matrix4d FKinSpace(const Eigen::Matrix4d &M,
                          const std::vector<Eigen::Matrix<double, 6, 1>> &Slist,
                          const std::vector<double> &thetalist);
}

#endif // F_KIN_SPACE_H
