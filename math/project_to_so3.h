#ifndef PROJECT_TO_SO3_H
#define PROJECT_TO_SO3_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix3d ProjectToSO3(const Eigen::Matrix3d &mat);
}

#endif // PROJECT_TO_SO3_H
