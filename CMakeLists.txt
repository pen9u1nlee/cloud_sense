cmake_minimum_required(VERSION 3.0.2)
project(image_subscriber)

# 查找 catkin 宏和库
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  rtabmap_ros
  rtabmap_conversions
)

find_package(g2o REQUIRED)
# 添加 g2o 头文件路径
include_directories(/home/jetson/g2o/g2o/include)
link_directories(/home/jetson/g2o/g2o/lib)


# 设置Boost库的路径
find_package(Boost REQUIRED COMPONENTS serialization)

# 查找OpenCV和RTAB-Map
find_package(OpenCV REQUIRED)
find_package(rtabmap REQUIRED)  # 查找 RTAB-Map 库
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含头文件
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
  /home/jetson/orbbec_ws/src/image_subscriber/include/image_subscriber  # 添加 httplib.h 的路径
  /home/jetson/rtabmap/corelib/include/rtabmap/core  # 添加 RTAB-Map 头文件目录
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${rtabmap_INCLUDE_DIRS}  # 这里是自动生成的 RTAB-Map 头文件目录
  ${CSLAM_COMMON_INTERFACES_PATH}/include  # 添加 cslam_common_interfaces 头文件路径
)

# 查找并链接线程库
find_package(Threads REQUIRED)  # 找到并链接线程库

# 声明 ROS 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_subscriber
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport rtabmap_ros rtabmap_conversions
)

# 创建可执行文件
add_executable(image_subscriber src/image_subscriber.cpp)  # 创建 image_subscriber 可执行文件
add_executable(pgo src/pgo.cpp) 

# 为每个目标链接相应的库
target_link_libraries(image_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${RTABMAP_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  Boost::serialization
)

target_link_libraries(pgo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rtabmap_core
)


# 安装目标
install(TARGETS image_subscriber
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

