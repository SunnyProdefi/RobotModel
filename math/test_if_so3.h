#ifndef TEST_IF_SO3_H
#define TEST_IF_SO3_H

#include <Eigen/Dense>

namespace math {
bool TestIfSO3(const Eigen::Matrix3d &mat);
}

#endif // TEST_IF_SO3_H
