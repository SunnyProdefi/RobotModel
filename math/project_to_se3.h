#ifndef PROJECT_TO_SE3_H
#define PROJECT_TO_SE3_H

#include <Eigen/Dense>

namespace math {
Eigen::Matrix4d ProjectToSE3(const Eigen::Matrix4d &mat);
}

#endif // PROJECT_TO_SE3_H
