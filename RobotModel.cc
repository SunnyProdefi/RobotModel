#include "Kinematics/Kinematics.h"
#include <Eigen/Dense>
#include <cmath> // For M_PI and trigonometric functions
#include <iostream>

int main() {
  // UR5 DH parameters
  std::vector<std::vector<double>> dhParams = {
      {0, M_PI / 2, 0.089159, 0}, {-0.42500, 0, 0, 0},
      {-0.39225, 0, 0, 0},        {0, M_PI / 2, 0.10915, 0},
      {0, -M_PI / 2, 0.09465, 0}, {0, 0, 0.0823, 0}};

  // Joint angles (for example, all set to 1 radian)
  std::vector<double> jointAngles = {1, 0.5, 1, 1, 1, 1};

  // Create a DHKinematics object with the DH parameters
  DHKinematics ur5(dhParams);

  // Compute the forward kinematics
  Eigen::Matrix4d T = ur5.forwardKinematics(jointAngles);

  // Output the transformation matrix
  std::cout << "Transformation matrix T:" << std::endl << T << std::endl;
  return 0;
}
