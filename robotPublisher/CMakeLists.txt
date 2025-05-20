cmake_minimum_required(VERSION 3.8)
project(robotstatepub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_definitions(-DADAM_LITE)

# 核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# 消息生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Imu.msg"
  "msg/JointStateCmd.msg"
  "msg/RobotState.msg"
  DEPENDENCIES 
  geometry_msgs
  builtin_interfaces
)
# 获取 ROS 2 类型支持
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 头文件包含
include_directories(
  include
  ../robot_interface/include()
  ${Eigen3_INCLUDE_DIRS}
)

# 添加共享库路径
link_directories(${PROJECT_SOURCE_DIR}/lib)

# 定义可执行文件
add_executable(${PROJECT_NAME}_node "src/pndbotics_publisher.cpp")

# 添加依赖
ament_target_dependencies(${PROJECT_NAME}_node rclcpp geometry_msgs)

# 设置 include 目录
target_include_directories(${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


# 关联共享库（显式指定绝对路径）
target_link_libraries(${PROJECT_NAME}_node
  ${typesupport_target}
  Eigen3::Eigen
  ${PROJECT_SOURCE_DIR}/../robot_interface/lib/libRobotInterface.so
)

# 安装配置
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(FILES ../robot_interface/lib/libRobotInterface.so DESTINATION lib)  # 安装到 lib
install(DIRECTORY ../robot_interface/include/ DESTINATION include)

# 导出配置
ament_export_dependencies(rclcpp geometry_msgs rosidl_default_runtime rosidl_default_generators)
ament_export_libraries(libRobotInterface.so)

ament_package()