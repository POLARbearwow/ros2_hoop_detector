cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)

# --- 海康相机 SDK ---
# 假设 SDK 安装在 /opt/MVS 目录
include_directories(SYSTEM /opt/MVS/include)
link_directories(/opt/MVS/lib/64)
# ---------------------

# 1. 创建一个基础库，用于封装海康SDK的逻辑
# 这个库不包含ROS节点，只是纯粹的C++相机驱动代码
add_library(hik_camera_driver src/hik_camera.cpp)
target_include_directories(hik_camera_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hik_camera_driver PUBLIC ${OpenCV_LIBS} PRIVATE MvCameraControl)


# 2. 将你的ROS节点编译成一个组件库 (SHARED LIBRARY)
# 注意：我们将源文件改成了 hik_camera_node.cpp
add_library(hik_camera_component SHARED
  src/hik_camera_node.cpp
)
set_target_properties(hik_camera_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 为组件添加所有必需的依赖 (这是您缺失的关键步骤！)
ament_target_dependencies(hik_camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
)

# 将组件链接到我们上面创建的相机驱动库
target_link_libraries(hik_camera_component hik_camera_driver)


# 3. 注册组件插件
# 这是让 `ros2 component load` 能够找到你的组件的关键步骤。
rclcpp_components_register_nodes(hik_camera_component
  "hik_camera::HikCameraNode"
)


# 4. 安装目标
# 我们现在只安装库和组件，不再有独立的_node可执行文件
install(TARGETS
  hik_camera_driver
  hik_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 5. 导出依赖，以便其他包可以使用此包
ament_export_include_directories(include)
ament_export_libraries(hik_camera_driver hik_camera_component)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs cv_bridge OpenCV)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs cv_bridge image_transport OpenCV)

ament_package()