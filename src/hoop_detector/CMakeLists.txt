cmake_minimum_required(VERSION 3.8)
project(hoop_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

# 额外的包含目录（OpenCV 已经在 find_package 中给出）
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 1. 算法共享库
add_library(hoop_detector_algorithm 
  src/hoop_detector.cpp
)

set_target_properties(hoop_detector_algorithm PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 头文件目录
target_include_directories(hoop_detector_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}   # 修改为当前目录
)

target_link_libraries(hoop_detector_algorithm
  ${OpenCV_LIBS}
)

ament_target_dependencies(hoop_detector_algorithm
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

# 2. 组件库
add_library(hoop_detector_component SHARED
  src/hoop_detector_node.cpp
)

ament_target_dependencies(hoop_detector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

target_link_libraries(hoop_detector_component
  hoop_detector_algorithm
)

# 注册组件
rclcpp_components_register_nodes(hoop_detector_component
  "hoop_detector::HoopDetectorNode"
)

# 安装目标
install(TARGETS
  hoop_detector_algorithm
  hoop_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装launch文件  hik的不用单独执行 所以不用添加install （在intall/../hik_camera 中就少了launch）
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 导出
ament_export_include_directories(include)
ament_export_libraries(hoop_detector_algorithm hoop_detector_component)
# ament_export_dependencies(rclcpp rclcpp_components sensor_msgs geometry_msgs cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs visualization_msgs OpenCV)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs std_msgs geometry_msgs cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs visualization_msgs OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
