file(GLOB utilities "*.cpp")

add_library(util STATIC ${utilities})

target_include_directories(util PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(util PUBLIC Eigen3::Eigen Ceres::ceres glog::glog tsl::robin_map yaml-cpp OpenMP::OpenMP_CXX)

if (WITH_VIZ3D)
    target_compile_definitions(util PUBLIC CT_ICP_WITH_VIZ=1)
    target_link_libraries(util PUBLIC viz3d)
endif ()