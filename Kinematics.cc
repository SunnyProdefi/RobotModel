#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

class Kinematics {
public:
  virtual Eigen::Matrix4d
  forwardKinematics(const std::vector<double> &jointAngles) = 0;
  virtual ~Kinematics() {}
};

class DHKinematics : public Kinematics {
private:
  std::vector<std::vector<double>> dhParameters;

public:
  DHKinematics(const std::vector<std::vector<double>> &dhParams)
      : dhParameters(dhParams) {}

  Eigen::Matrix4d
  forwardKinematics(const std::vector<double> &jointAngles) override {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < dhParameters.size(); ++i) {
      double a = dhParameters[i][0];
      double alpha = dhParameters[i][1];
      double d = dhParameters[i][2];
      double theta = dhParameters[i][3] + jointAngles[i];

      Eigen::Matrix4d A_i;
      A_i << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
          a * cos(theta), sin(theta), cos(theta) * cos(alpha),
          -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha),
          d, 0, 0, 0, 1;
      T = T * A_i;
    }
    return T;
  }
};

class ScrewKinematics : public Kinematics {
private:
  std::vector<Eigen::Vector6d> screwAxes;
  std::vector<double> h;

public:
  ScrewKinematics(const std::vector<Eigen::Vector6d> &axes,
                  const std::vector<double> &pitch)
      : screwAxes(axes), h(pitch) {}

  Eigen::Matrix4d
  forwardKinematics(const std::vector<double> &jointAngles) override {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < screwAxes.size(); ++i) {
      double theta = jointAngles[i];
      // This is a placeholder for the actual exponential map calculation
      Eigen::Matrix4d matrixExp =
          Eigen::Matrix4d::Identity(); // 实际上应该计算螺旋轴的指数映射
      T = T * matrixExp;
    }
    return T;
  }
};
