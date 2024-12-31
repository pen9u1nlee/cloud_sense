cmake_minimum_required(VERSION 3.10)
project(CloudEdgeTransfer)

# 添加可执行文件
add_executable(cloud cloud.cpp)
add_executable(edge edge.cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_PREFIX_PATH home/jetson/rtabmap/build ${CMAKE_PREFIX_PATH})
set(rtabmap_DIR /home/jetson/rtabmap/build)
include_directories(/home/jetson/cloud_edge_test/Keyframe/)
include_directories(/usr/local/include/rtabmap-0.21/)
include_directories(/opt/ros/noetic/include/)
include_directories(/home/jetson/opencv4/include/opencv4/)

add_definitions(-DEIGEN_DONT_INLINE)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# 设置Boost库的路径
find_package(Boost REQUIRED COMPONENTS serialization)
target_link_libraries(cloud Boost::serialization)
target_link_libraries(edge Boost::serialization)


find_package(OpenCV REQUIRED)
target_link_libraries(cloud ${OpenCV_LIBS})
target_link_libraries(edge ${OpenCV_LIBS})

# 设置RTAB-Map库的路径
find_package(RTABMap 0.20.18 REQUIRED)
# find_package(rtabmap REQUIRED)

#pthread
find_package(Threads REQUIRED)  # 找到并链接线程库

# 为每个目标链接 pthread 库
target_link_libraries(cloud ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(edge ${CMAKE_THREAD_LIBS_INIT})



# 链接Boost和RTAB-Map库
#target_link_libraries(cloud Boost::archive rtabmap::rtabmap)
#target_link_libraries(edge Boost::archive rtabmap::rtabmap)


