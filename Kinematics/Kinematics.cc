#include "Kinematics.h"
#include <Eigen/Geometry> // Include this for advanced transformations if needed
#include <cmath>

// Implementation of DHKinematics constructor
DHKinematics::DHKinematics(const std::vector<std::vector<double>> &dhParams)
    : dhParameters(dhParams) {}

// Implementation of DHKinematics forwardKinematics
Eigen::Matrix4d
DHKinematics::forwardKinematics(const std::vector<double> &jointAngles) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < dhParameters.size(); ++i) {
    double a = dhParameters[i][0];
    double alpha = dhParameters[i][1];
    double d = dhParameters[i][2];
    double theta = dhParameters[i][3] + jointAngles[i];

    Eigen::Matrix4d A_i;
    A_i << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
        a * cos(theta), sin(theta), cos(theta) * cos(alpha),
        -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;
    T = T * A_i;
  }
  return T;
}

// Implementation of ScrewKinematics constructor
ScrewKinematics::ScrewKinematics(
    const std::vector<Eigen::Matrix<double, 6, 1>> &axes,
    const std::vector<double> &pitch)
    : screwAxes(axes), h(pitch) {}

// Implementation of ScrewKinematics forwardKinematics
Eigen::Matrix4d
ScrewKinematics::forwardKinematics(const std::vector<double> &jointAngles) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < screwAxes.size(); ++i) {
    double theta = jointAngles[i];
    // This is a placeholder for the actual exponential map calculation
    Eigen::Matrix4d matrixExp =
        Eigen::Matrix4d::Identity(); // Placeholder: actually should calculate
                                     // the exponential map of the screw axis
    T = T * matrixExp;
  }
  return T;
}
