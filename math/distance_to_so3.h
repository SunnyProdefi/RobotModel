#ifndef DISTANCE_TO_SO3_H
#define DISTANCE_TO_SO3_H

#include <Eigen/Dense>

namespace math {
double DistanceToSO3(const Eigen::Matrix3d &mat);
}

#endif // DISTANCE_TO_SO3_H
