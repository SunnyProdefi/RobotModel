#include "Kinematics/Kinematics.h"
#include <Eigen/Dense>
#include <cmath> // For M_PI
#include <iostream>
#include <memory> // For std::unique_ptr

int main() {
  // Define the Denavit-Hartenberg parameters for the UR5 robot arm
  std::vector<std::vector<double>> dhParameters = {
      {0, M_PI / 2, 0.089159, 0}, {-0.42500, 0, 0, 0},
      {-0.39225, 0, 0, 0},        {0, M_PI / 2, 0.10915, 0},
      {0, -M_PI / 2, 0.09465, 0}, {0, 0, 0.0823, 0}};

  // Initialize joint angles to zero radians for this example
  std::vector<double> jointAngles = {0, 0, 0, 0, 0, 0};

  // Create a unique pointer to a DHKinematics object with the given parameters
  std::unique_ptr<DHKinematics> ur5RobotArm =
      std::make_unique<DHKinematics>(dhParameters);

  // Compute the forward kinematics for the current joint angles
  Eigen::Matrix4d transformationMatrix =
      ur5RobotArm->forwardKinematics(jointAngles);

  // Output the 4x4 transformation matrix
  std::cout << "Transformation matrix T:\n"
            << transformationMatrix << std::endl;

  Eigen::Matrix3d RotationMatrix = transformationMatrix.block<3, 3>(0, 0);
  // Convert the rotation part of the transformation matrix to aquaternion
  Eigen::Quaterniond quaternion(RotationMatrix);
  std::cout << "Quaternion(xyzw): " << quaternion.x() << " " << quaternion.y()
            << " " << quaternion.z() << " " << quaternion.w() << std::endl;

  // Convert the quaternion to Euler angles in Roll-Pitch-Yaw order
  Eigen::Vector3d rollPitchYaw = RotationMatrix.eulerAngles(0, 1, 2);
  std::cout << "Euler angles (RPY): " << rollPitchYaw(0) << " "
            << rollPitchYaw(1) << " " << rollPitchYaw(2) << std::endl;

  return 0;
}
