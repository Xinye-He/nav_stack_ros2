cmake_minimum_required(VERSION 3.8)
project(robosense_driver)

# 编译告警（屏蔽常见但无害的告警）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
    -Wno-missing-field-initializers
  )
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(stack_msgs REQUIRED)

# 头文件路径（rs_driver 为头文件库，加入其源码路径即可）
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_driver/src
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# 查找 libpcap（rs_driver 的 pcap 输入实现需要；即便在线模式也建议链接）
find_library(PCAP_LIB pcap PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/local/lib)
if(NOT PCAP_LIB)
  message(FATAL_ERROR "libpcap not found. Install with: sudo apt-get install -y libpcap-dev")
else()
  message(STATUS "Found libpcap: ${PCAP_LIB}")
endif()

# 可执行：雷达点云发布（rsview）
add_executable(rsview src/rsview.cpp)
ament_target_dependencies(rsview
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
)
target_link_libraries(rsview
  pthread
  ${PCAP_LIB}
)

# 可执行：圆捆草垛估计（bale_estimator，在 base_link 下做 TF 转换）
add_executable(bale_estimator src/bale_estimator.cpp)
ament_target_dependencies(bale_estimator
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_link_libraries(bale_estimator
  ${PCL_LIBRARIES}
)

add_executable(bale_detector src/bale_detector.cpp)
ament_target_dependencies(bale_detector
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  stack_msgs
)
target_link_libraries(bale_detector
  ${PCL_LIBRARIES}
)

# 安装
install(TARGETS
  rsview
  bale_estimator
  bale_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
