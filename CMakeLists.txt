cmake_minimum_required(VERSION 3.0.2)
project(cloud_sense)

# 查找 catkin 宏和库
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  rtabmap_ros
  rtabmap_conversions
  std_msgs
  message_generation
)

add_message_files(
  FILES
  KeyFrameRGB.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

# 设置Boost库的路径
find_package(Boost REQUIRED COMPONENTS serialization)

# 查找OpenCV和RTAB-Map
find_package(OpenCV REQUIRED)
find_package(rtabmap REQUIRED)  # 查找 RTAB-Map 库
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# 查找并设置 HTTP 库 (httplib)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
  ${PROJECT_SOURCE_DIR}/include/cloud_sense  # 添加 httplib.h 的路径
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
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge rtabmap_ros rtabmap_conversions std_msgs message_runtime image_transport
) 

# 创建可执行文件
add_executable(cloud_node src/cloud.cpp)  # 创建 cloud_node 可执行文件
add_executable(image_subscriber src/image_subscriber.cpp)  # 创建 image_subscriber 可执行文件

# 为每个目标链接相应的库
target_link_libraries(cloud_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${RTABMAP_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  Boost::serialization
)

target_link_libraries(image_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${RTABMAP_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  Boost::serialization
)

# 安装目标
install(TARGETS cloud_node image_subscriber
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



