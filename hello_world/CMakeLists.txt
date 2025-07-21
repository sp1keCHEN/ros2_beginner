cmake_minimum_required(VERSION 3.5)
project(hello_world)

# 设置编译标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

# 编译器警告（可选）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成自定义消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose.msg"
  "msg/RobotStatus.msg"
  "srv/MoveRobot.srv"
  DEPENDENCIES geometry_msgs
)

# 添加可执行文件
add_executable(node_01 src/node_01.cpp)
add_executable(node_03 src/node_03.cpp)
add_executable(publisher src/publisher.cpp)
add_executable(subscriber src/subscriber.cpp)

# 设置依赖
ament_target_dependencies(node_01 rclcpp)
ament_target_dependencies(node_03 rclcpp)
ament_target_dependencies(publisher rclcpp example_interfaces)
ament_target_dependencies(subscriber rclcpp example_interfaces)

# 安装目标
install(TARGETS
  node_01
  node_03
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()