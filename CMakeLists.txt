cmake_minimum_required(VERSION 3.5)
project(laser_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
        visualization_msgs
        tf2_ros
        tf2
        nav_msgs
        sensor_msgs
        pcl_conversions
        std_srvs
)

#find_package(teaserpp REQUIRED)
find_package(Boost 1.58 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features kdtree visualization)
find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv320/include)

# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
# 设置依赖库链接目录
link_directories(${PCL_LIBRARY_DIRS})
# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS})


include_directories(
        include
        include/global_localization
        include/laser_odometry
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS roscpp visualization_msgs tf2_ros tf2 nav_msgs sensor_msgs pcl_conversions std_srvs actionlib
)


add_library(
        localization_lib
        src/localization/localization.cpp
        src/error_estimater/error_estimater.cpp
        src/localization/UKF.cpp
        src/global_localization/bbs_localization.cpp
        src/global_localization/global_localization.cpp
        src/laser_odometry/laser_odometry.cpp
        src/error_estimater/voxelset.cpp
        src/localization/tools.cpp
)

target_link_libraries(
        localization_lib
        ${PCL_LIBRARIES}
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
)


add_executable(laser_localization src/main.cpp)
target_link_libraries(laser_localization
        ${PCL_LIBRARIES}
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        localization_lib
        ${catkin_LIBRARIES}
)
install(TARGETS
        laser_localization
        #        DESTINATION lib/${PROJECT_NAME}
        #        LIBRARY DESTINATION lib/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
### test

add_executable(test_laser_odom test/test_laser_odom.cpp)
target_link_libraries(test_laser_odom
        ${PCL_LIBRARIES}
        ${EIGEN3_LIBS}
        localization_lib
)

add_executable(test_global_localization test/test_global_localization.cpp)
target_link_libraries(test_global_localization
        ${PCL_LIBRARIES}
        ${EIGEN3_LIBS}
        localization_lib
        ${OpenCV_LIBS}
)
