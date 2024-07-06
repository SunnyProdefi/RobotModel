add_library(math SHARED 
    near_zero.cc 
    normalize.cc
    rot_inv.cc
    vec_to_so3.cc
    so3_to_vec.cc
    axis_ang3.cc
    matrix_exp3.cc
    matrix_log3.cc
    distance_to_so3.cc
    test_if_so3.cc
    project_to_so3.cc
    rp_to_trans.cc
    trans_to_rp.cc
    trans_inv.cc
    vec_to_se3.cc
    se3_to_vec.cc
    adjoint.cc
    screw_to_axis.cc
    axis_ang6.cc
    matrix_exp6.cc
    matrix_log6.cc
    distance_to_se3.cc
    test_if_se3.cc
    project_to_se3.cc
    f_kin_body.cc
    f_kin_space.cc
)
target_include_directories(math PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(math Eigen3::Eigen)