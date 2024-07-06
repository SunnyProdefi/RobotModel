#ifndef AXIS_ANG6_H
#define AXIS_ANG6_H

#include <Eigen/Dense>
#include <utility> // For std::pair

namespace math {
std::pair<Eigen::Matrix<double, 6, 1>, double>
AxisAng6(const Eigen::Matrix<double, 6, 1> &expc6);
}

#endif // AXIS_ANG6_H
