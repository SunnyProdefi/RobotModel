add_library(Kinematics SHARED Kinematics.cc)
target_include_directories(Kinematics PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(Kinematics Eigen3::Eigen)
