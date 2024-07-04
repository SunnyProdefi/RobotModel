cmake_minimum_required(VERSION 3.29)
Project(RobotModel VERSION 1.0 LANGUAGES CXX) 

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(RobotModel RobotModel.cpp)
target_link_libraries(RobotModel Eigen3::Eigen)