#include "kinematics.h"
#include "matrix_exp6.h"
#include "vec_to_se3.h"
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
    const Eigen::Matrix4d &M)
    : screwAxes(axes), initialTransform(M) {}

// Implementation of ScrewKinematics forwardKinematics
Eigen::Matrix4d
ScrewKinematics::forwardKinematics(const std::vector<double> &jointAngles) {
  Eigen::Matrix4d T = initialTransform;
  for (size_t i = 0; i < screwAxes.size(); ++i) {
    double theta = jointAngles[i];
    // Calculate the se(3) representation of the screw axis scaled by the joint
    // angle
    Eigen::Matrix<double, 6, 1> se3 = screwAxes[i] * theta;
    // Compute the matrix exponential of the se(3) matrix
    Eigen::Matrix4d matrixExp = math::MatrixExp6(math::VecTose3(se3));
    T = T * matrixExp;
  }
  return T;
}
