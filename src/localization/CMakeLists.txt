
include_directories(
        ../utilities
        ../laser_odometry
)
add_library(LOCALIZATION STATIC localization.cpp UKF.cpp)

target_include_directories(LOCALIZATION PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(LOCALIZATION PUBLIC Eigen3::Eigen Ceres::ceres glog::glog tsl::robin_map yaml-cpp OpenMP::OpenMP_CXX small_gicp::small_gicp)

if (WITH_VIZ3D)
    target_compile_definitions(LOCALIZATION PUBLIC CT_ICP_WITH_VIZ=1)
    target_link_libraries(LOCALIZATION PUBLIC viz3d)
endif ()