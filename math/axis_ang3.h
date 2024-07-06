#ifndef AXIS_ANG3_H
#define AXIS_ANG3_H

#include <Eigen/Dense>
#include <utility> // for std::pair

namespace math {
std::pair<Eigen::Vector3d, double> AxisAng3(const Eigen::Vector3d &expc3);
}

#endif // AXIS_ANG3_H
