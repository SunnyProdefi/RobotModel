cmake_minimum_required(VERSION 3.29)
Project(RobotModel VERSION 1.0 LANGUAGES CXX) 

#设置C++标准
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(kinematics)
add_subdirectory(math)

add_executable(RobotModel RobotModel.cc)
target_link_libraries(RobotModel Eigen3::Eigen kinematics math)