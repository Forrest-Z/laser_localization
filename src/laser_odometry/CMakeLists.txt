file(GLOB CT_ICP_SRC "ct_icp/*")

include_directories(
        ../utilities
)

add_library(CT_ICP STATIC ct_odometry.cpp ${CT_ICP_SRC})

target_include_directories(CT_ICP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CT_ICP PUBLIC util Eigen3::Eigen Ceres::ceres glog::glog tsl::robin_map yaml-cpp OpenMP::OpenMP_CXX)

if (WITH_VIZ3D)
    target_compile_definitions(CT_ICP PUBLIC CT_ICP_WITH_VIZ=1)
    target_link_libraries(CT_ICP PUBLIC viz3d)
endif ()
