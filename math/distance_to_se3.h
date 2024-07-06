#ifndef DISTANCE_TO_SE3_H
#define DISTANCE_TO_SE3_H

#include <Eigen/Dense>

namespace math {
double DistanceToSE3(const Eigen::Matrix4d &mat);
}

#endif // DISTANCE_TO_SE3_H
