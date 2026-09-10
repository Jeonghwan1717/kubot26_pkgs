cmake_minimum_required(VERSION 3.8)
project(kubot26_pkgs)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KubotControlMsgs.msg"
  DEPENDENCIES std_msgs
)

add_library(kubot_control_node SHARED
  src/kubot26_plugin.cpp
  src/kudos/CKubot.cpp
)

target_include_directories(kubot_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

ament_target_dependencies(kubot_control_node
  rclcpp
  sensor_msgs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(kubot_control_node
  Eigen3::Eigen
  ${GAZEBO_LIBRARIES}
  ${cpp_typesupport_target}
  ${gazebo_ros_LIBRARIES}
)

install(TARGETS
  kubot_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config DESTINATION share/${PROJECT_NAME})

ament_package()
