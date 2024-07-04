#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
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
  DHKinematics(const std::vector<std::vector<double>> &dhParams);
  Eigen::Matrix4d
  forwardKinematics(const std::vector<double> &jointAngles) override;
};

class ScrewKinematics : public Kinematics {
private:
  std::vector<Eigen::Matrix<double, 6, 1>>
      screwAxes; // Use Eigen::Matrix for a fixed-size 6D vector
  std::vector<double> h;

public:
  ScrewKinematics(const std::vector<Eigen::Matrix<double, 6, 1>> &axes,
                  const std::vector<double> &pitch);
  Eigen::Matrix4d
  forwardKinematics(const std::vector<double> &jointAngles) override;
};

#endif // KINEMATICS_H
