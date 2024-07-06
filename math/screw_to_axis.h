#ifndef SCREW_TO_AXIS_H
#define SCREW_TO_AXIS_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix<double, 6, 1> ScrewToAxis(const Eigen::Vector3d &q,
                                        const Eigen::Vector3d &s, double h);
}

#endif // SCREW_TO_AXIS_H
