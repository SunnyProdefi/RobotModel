include_directories(${PROJECT_SOURCE_DIR}/math)
add_library(kinematics SHARED kinematics.cc)
target_include_directories(kinematics PRIVATE)
target_link_libraries(kinematics)
