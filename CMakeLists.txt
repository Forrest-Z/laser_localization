cmake_minimum_required(VERSION 3.14)
project(laser_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(WITH_VIZ3D "Search and install viz3d (disabled by default)" OFF)
option(WITH_PYTHON_BINDING "Install Python bindings" OFF)

if (NOT RUNTIME_DESTINATION)
    set(RUNTIME_DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
endif()
set(CMAKE_INSTALL_PREFIX ${RUNTIME_DESTINATION})
set(laser_localization_RUNTIME_DESTINATION ${RUNTIME_DESTINATION})

include(cmake/external.cmake)

find_package(OpenMP)
if(NOT TARGET OpenMP::OpenMP_CXX)
    message(INFO "Could not find OpenMP")
endif()

add_subdirectory(src/utilities)
add_subdirectory(src/laser_odometry)
add_subdirectory(test)

#cmake_minimum_required(VERSION 3.5)
#project(laser_localization)
#
## Default to C99
#if(NOT CMAKE_C_STANDARD)
#    set(CMAKE_C_STANDARD 99)
#endif()
#
## Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#    set(CMAKE_CXX_STANDARD 17)
#endif()
#
##set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
#
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
#add_definitions(-DCT_ICP_WITH_VIZ)
#
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#endif()
#add_definitions(-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
#set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif()
#
#add_subdirectory(third_part/robin-map-1.3.0)
#
## find dependencies
#find_package(catkin REQUIRED COMPONENTS
#        roscpp
#        visualization_msgs
#        tf2_ros
#        tf2
#        nav_msgs
#        sensor_msgs
#        pcl_conversions
#        std_srvs
#)
#
#
##find_package(teaserpp REQUIRED)
#find_package(Boost 1.58 REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common features kdtree visualization)
#find_package(OpenCV REQUIRED)
##find_package(colormap INTERFACE)
#
#set(Ceres_DIR "/home/vio/libs/ceres/1.14/lib/cmake/Ceres/")
#find_package(Ceres REQUIRED)
#
#include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv320/include)
#
## 包含头文件目录
#include_directories(${PCL_INCLUDE_DIRS})
## 设置依赖库链接目录
#link_directories(${PCL_LIBRARY_DIRS})
## 添加预处理器和编译器标记
#add_definitions(${PCL_DEFINITIONS})
#
#
#include_directories(
#        include
#        include/global_localization
#        include/laser_odometry
#        /usr/local/include
#        ${CERES_INCLUDE_DIRS}
#        ${catkin_INCLUDE_DIRS}
#        third_part/tclap-1.2.5/include
#        third_part/viz3d/src
#        third_part/viz3d/cmake-build-Release/external/install/Release/glad/include
#        third_part/viz3d/cmake-build-Release/external/install/Release/glfw/include
#        third_part/viz3d/cmake-build-Release/external/install/Release/glog/include
#        third_part/imgui/build/dist/include
#)
#
#catkin_package(
#        CATKIN_DEPENDS roscpp visualization_msgs tf2_ros tf2 nav_msgs sensor_msgs pcl_conversions std_srvs actionlib
#)
#
#
##add_library(
##        localization_lib
##        src/localization/localization.cpp
##        src/error_estimater/error_estimater.cpp
##        src/localization/UKF.cpp
##        src/global_localization/bbs_localization.cpp
##        src/global_localization/global_localization.cpp
##        src/laser_odometry/laser_odometry.cpp
##        src/error_estimater/voxelset.cpp
##        src/localization/tools.cpp
##        src/utilities/file.cpp
##        src/utilities/plyFile.cpp
##        src/utilities/persoTimer.cpp
##        src/utilities/datasetReader.cpp
##        src/utilities/io.cpp
##        src/utilities/evaluate_slam.cpp
##        src/laser_odometry/ct_odometry.cpp
##        src/laser_odometry/ct_icp/ct_icp.cpp
##        src/laser_odometry/ct_icp/cost_functions.cpp
##)
##
##target_link_libraries(
##        localization_lib
##        ${PCL_LIBRARIES}
##        ${EIGEN3_LIBS}
##        ${OpenCV_LIBS}
##        ${catkin_LIBRARIES}
##        tsl::robin_map
##        glog
##        yaml-cpp
##        ${CERES_LIBRARIES}
##        /home/vio/Code/RobotSystem/human/src/localization/laser_localization/third_part/viz3d/cmake-build-Release/src/libviz3d.a
##        /home/vio/Code/RobotSystem/human/src/localization/laser_localization/third_part/imgui/build/dist/lib/libimgui.a
##)
##
##
##add_executable(laser_localization src/main.cpp)
##target_link_libraries(laser_localization
##        ${PCL_LIBRARIES}
##        ${EIGEN3_LIBS}
##        ${OpenCV_LIBS}
##        localization_lib
##        ${catkin_LIBRARIES}
##        glog
##        yaml-cpp
##)
##install(TARGETS
##        laser_localization
##        #        DESTINATION lib/${PROJECT_NAME}
##        #        LIBRARY DESTINATION lib/${PROJECT_NAME}
##        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##)
#### test
#
##add_executable(test_ct_odom_kitti test/test_ct_odom_kitti.cpp)
##target_link_libraries(test_ct_odom_kitti
##        ${PCL_LIBRARIES}
##        ${EIGEN3_LIBS}
##        localization_lib
##)
#
#add_executable(test_ct_odom_kitti
#        src/utilities/file.cpp
#        src/utilities/plyFile.cpp
#        src/utilities/persoTimer.cpp
#        src/utilities/datasetReader.cpp
#        src/utilities/io.cpp
#        src/utilities/evaluate_slam.cpp
#        src/laser_odometry/ct_odometry.cpp
#        src/laser_odometry/ct_icp/ct_icp.cpp
#        src/laser_odometry/ct_icp/cost_functions.cpp
#        test/test_ct_odom_kitti.cpp)
#
#target_link_libraries(test_ct_odom_kitti
#        ${EIGEN3_LIBS}
#        glog
#        yaml-cpp
#        tsl::robin_map
#        ${CERES_LIBRARIES}
#        /home/vio/Code/RobotSystem/human/src/localization/laser_localization/third_part/viz3d/cmake-build-Release/src/libviz3d.a
#        /home/vio/Code/RobotSystem/human/src/localization/laser_localization/third_part/imgui/build/dist/lib/libimgui.a
#)
#
##add_executable(test_global_localization test/test_global_localization.cpp)
##target_link_libraries(test_global_localization
##        ${PCL_LIBRARIES}
##        ${EIGEN3_LIBS}
##        localization_lib
##        ${OpenCV_LIBS}
##)
