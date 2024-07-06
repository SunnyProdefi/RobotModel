#include "kinematics/kinematics.h"
#include "math/math.h"
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

  //
  std::vector<Eigen::Matrix<double, 6, 1>> axes = {
      (Eigen::Matrix<double, 6, 1>() << 0, 0, -1, 2, 0, 0).finished(),
      (Eigen::Matrix<double, 6, 1>() << 0, 0, 0, 0, 1, 0).finished(),
      (Eigen::Matrix<double, 6, 1>() << 0, 0, 1, 0, 0, 0.1).finished()};
  Eigen::Matrix4d initMat;
  initMat << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1;

  std::unique_ptr<ScrewKinematics> ur5RobotArmScrew =
      std::make_unique<ScrewKinematics>(axes, initMat);
  std::vector<double> jointAngles1 = {M_PI / 2, 3, M_PI};
  Eigen::Matrix4d transformationMatrixScrew =
      ur5RobotArmScrew->forwardKinematics(jointAngles1);
  // Output the 4x4 transformation matrix
  std::cout << "Transformation matrix T:\n"
            << transformationMatrixScrew << std::endl;
  // math test
  // Test the NearZero function
  std::cout << "NearZero(1e-7): " << math::NearZero(1e-7) << std::endl;

  // Test the Normalize function
  Eigen::VectorXd V(3);
  V << 1, 2, 3;
  Eigen::VectorXd norm_v = math::Normalize(V);
  std::cout << "Normalized vector:\n" << norm_v << std::endl;

  // Test the RotInv function
  Eigen::Matrix3d R;
  R << 0, 0, 1, 1, 0, 0, 0, 1, 0;
  Eigen::Matrix3d invR = math::RotInv(R);
  std::cout << "Inverse (transpose) of the rotation matrix:\n"
            << invR << std::endl;

  // Test the VecToso3 function
  Eigen::Vector3d omg;
  omg << 1, 2, 3;
  Eigen::Matrix3d so3mat = math::VecToso3(omg);
  std::cout << "Skew symmetric matrix in so(3):\n" << so3mat << std::endl;

  // Test the so3ToVec function
  Eigen::Matrix3d so3mat_1;
  so3mat_1 << 0, -3, 2, 3, 0, -1, -2, 1, 0;
  Eigen::Vector3d omg_1 = math::so3ToVec(so3mat_1);
  std::cout << "Corresponding 3-vector (angular velocity):\n"
            << omg_1 << std::endl;

  // Test the AxisAng3 function
  Eigen::Vector3d expc3;
  expc3 << 1, 2, 3;
  auto result = math::AxisAng3(expc3);
  Eigen::Vector3d omghat = result.first;
  double theta = result.second;
  std::cout << "Unit rotation axis (omghat):\n" << omghat << std::endl;
  std::cout << "Rotation angle (theta):\n" << theta << std::endl;

  // Test the MatrixExp3 function
  Eigen::Matrix3d so3mat_2;
  so3mat_2 << 0, -3, 2, 3, 0, -1, -2, 1, 0;
  Eigen::Matrix3d R_1 = math::MatrixExp3(so3mat_2);
  std::cout << "R_1 =\n" << R_1 << std::endl;

  // Test the MatrixLog3 function
  Eigen::Matrix3d R_2;
  R_2 << 0, 0, 1, 1, 0, 0, 0, 1, 0;
  Eigen::Matrix3d so3mat_3 = math::MatrixLog3(R);
  std::cout << "so3mat_3 =\n" << so3mat_3 << std::endl;

  // Test the DistanceToSO3 function
  Eigen::Matrix3d mat;
  mat << 1.0, 0.0, 0.0, 0.0, 0.1, -0.95, 0.0, 1.0, 0.1;
  double d = math::DistanceToSO3(mat);
  std::cout << "Distance to SO(3): " << d << std::endl;

  // Test the TestIfSO3 function
  Eigen::Matrix3d mat_1;
  mat_1 << 1.0, 0.0, 0.0, 0.0, 0.1, -0.95, 0.0, 1.0, 0.1;
  bool judge = math::TestIfSO3(mat_1);
  std::cout << "Is the matrix close to SO(3)?: " << (judge ? "Yes" : "No")
            << std::endl;

  // Test the ProjectToSO3 function
  Eigen::Matrix3d mat_2;
  mat_2 << 0.675, 0.150, 0.720, 0.370, 0.771, -0.511, -0.630, 0.619, 0.472;
  Eigen::Matrix3d R_3 = math::ProjectToSO3(mat_2);
  std::cout << "Projected matrix R_3 to SO(3):\n" << R_3 << std::endl;

  // Test the RpToTrans function
  Eigen::Matrix3d R_4;
  R_4 << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  Eigen::Vector3d p(1, 2, 5);
  Eigen::Matrix4d T = math::RpToTrans(R_4, p);
  std::cout << "Homogeneous transformation matrix T:\n" << T << std::endl;

  // Test the TransToRp function
  Eigen::Matrix4d T_1;
  T_1 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1;
  auto [R_5, p_1] = math::TransToRp(T_1);
  std::cout << "Rotation matrix R_5:\n" << R_5 << std::endl;
  std::cout << "Position vector p_1:\n"
            << p_1.transpose()
            << std::endl; // Transpose for better readability in output

  // Test the TransInv function
  Eigen::Matrix4d T_2;
  T_2 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1;
  Eigen::Matrix4d invT = math::TransInv(T_2);
  std::cout << "Inverse Transformation Matrix T_2:\n" << invT << std::endl;

  // Test the VecTose3 function
  Eigen::Matrix<double, 6, 1> V_1;
  V_1 << 1, 2, 3, 4, 5, 6; // Spatial velocity vector
  Eigen::Matrix4d se3mat = math::VecTose3(V_1);
  std::cout << "se(3) matrix:\n" << se3mat << std::endl;

  // Test the se3ToVec function
  Eigen::Matrix4d se3mat_1;
  se3mat_1 << 0, -3, 2, 4, 3, 0, -1, 5, -2, 1, 0, 6, 0, 0, 0, 0;

  Eigen::Matrix<double, 6, 1> V_2 = math::se3ToVec(se3mat_1);
  std::cout << "Corresponding 6-vector (spatial velocity):\n"
            << V_2.transpose() << std::endl;

  // Test the Adjoint function
  Eigen::Matrix4d T_3;
  T_3 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1;
  Eigen::Matrix<double, 6, 6> AdT = math::Adjoint(T_3);
  std::cout << "Adjoint representation [AdT]:\n" << AdT << std::endl;

  // Test the ScrewToAxis function
  Eigen::Vector3d q(3, 0, 0);
  Eigen::Vector3d s(0, 0, 1);
  double h = 2;
  Eigen::Matrix<double, 6, 1> S = math::ScrewToAxis(q, s, h);
  std::cout << "Screw axis S:\n" << S.transpose() << std::endl;

  // Test the AxisAng6 function
  Eigen::Matrix<double, 6, 1> expc6;
  expc6 << 1, 0, 0, 1, 2,
      3; // Example exponential coordinates for rigid-body motion
  auto [S_1, theta_1] = math::AxisAng6(expc6);
  std::cout << "Normalized screw axis S:\n" << S_1.transpose() << std::endl;
  std::cout << "Distance traveled along/about S (theta):\n"
            << theta_1 << std::endl;

  // Test the MatrixExp6 function
  Eigen::Matrix4d se3mat_2;
  se3mat_2 << 0, 0, 0, 0, 0, 0, -1.5708, 2.3562, 0, 1.5708, 0, 2.3562, 0, 0, 0,
      0;
  Eigen::Matrix4d T_4 = math::MatrixExp6(se3mat_2);
  std::cout << "Transformation Matrix T_4:\n" << T_4 << std::endl;

  // Test the MatrixLog6 function
  Eigen::Matrix4d T_5;
  T_5 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1;
  Eigen::Matrix4d expmat = math::MatrixLog6(T_5);
  std::cout << "se(3) matrix (exponential coordinates):\n"
            << expmat << std::endl;

  // Test the DistanceToSE3 function
  Eigen::Matrix4d mat_3;
  mat_3 << 1.0, 0.0, 0.0, 1.2, 0.0, 0.1, -0.95, 1.5, 0.0, 1.0, 0.1, -0.9, 0.0,
      0.0, 0.1, 0.98;
  double d_1 = math::DistanceToSE3(mat_3);
  std::cout << "Distance to SE(3): " << d_1 << std::endl;

  // Test the TestIfSE3 function
  Eigen::Matrix4d mat_4;
  mat_4 << 1.0, 0.0, 0.0, 1.2, 0.0, 0.1, -0.95, 1.5, 0.0, 1.0, 0.1, -0.9, 0.0,
      0.0, 0.1, 0.98;
  bool judge_1 = math::TestIfSE3(mat_4);
  std::cout << "Is the matrix close to SE(3)?: " << (judge_1 ? "Yes" : "No")
            << std::endl;

  // Test the ProjectToSE3 function
  Eigen::Matrix4d mat_5;
  mat_5 << 0.675, 0.150, 0.720, 1.2, 0.370, 0.771, -0.511, 5.4, -0.630, 0.619,
      0.472, 3.6, 0.003, 0.002, 0.010, 0.9;
  Eigen::Matrix4d T_6 = math::ProjectToSE3(mat_5);
  std::cout << "Projected matrix to SE(3):\n" << T_6 << std::endl;

  // Test the FKinBody function
  Eigen::Matrix4d M;
  M << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1;
  std::vector<Eigen::Matrix<double, 6, 1>> Blist = {
      (Eigen::Matrix<double, 6, 1>() << 0, 0, -1, 2, 0, 0).finished(),
      (Eigen::Matrix<double, 6, 1>() << 0, 0, 0, 0, 1, 0).finished(),
      (Eigen::Matrix<double, 6, 1>() << 0, 0, 1, 0, 0, 0.1).finished()};
  std::vector<double> thetalist = {M_PI / 2, 3, M_PI};
  Eigen::Matrix4d T_7 = math::FKinBody(M, Blist, thetalist);
  std::cout << "Transformation Matrix T_7:\n" << T_7 << std::endl;

  // Test the FKinSpace function
  Eigen::Matrix4d M_1;
  M_1 << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1;
  std::vector<Eigen::Matrix<double, 6, 1>> Slist = {
      (Eigen::Matrix<double, 6, 1>() << 0, 0, 1, 4, 0, 0).finished(),
      (Eigen::Matrix<double, 6, 1>() << 0, 0, 0, 0, 1, 0).finished(),
      (Eigen::Matrix<double, 6, 1>() << 0, 0, -1, -6, 0, -0.1).finished()};
  std::vector<double> thetalist_1 = {M_PI / 2, 3, M_PI};
  Eigen::Matrix4d T_8 = math::FKinSpace(M_1, Slist, thetalist_1);
  std::cout << "Transformation Matrix T_8:\n" << T_8 << std::endl;
  return 0;
}
