CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(ros2_vicon)
SET(CMAKE_BUILD_TYPE Release)

# DEPENDENCIES
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(ament_cmake_python REQUIRED)
FIND_PACKAGE(builtin_interfaces REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(rclcpp_components REQUIRED)
FIND_PACKAGE(std_msgs REQUIRED)
FIND_PACKAGE(geometry_msgs REQUIRED)

# VICON SDK
INCLUDE_DIRECTORIES(vicon_datastream_sdk/Release/Linux64)
LINK_DIRECTORIES(vicon_datastream_sdk/Release/Linux64)

# EXPORT DEPENDENCIES
AMENT_EXPORT_DEPENDENCIES(
	ament_cmake
	rclcpp
	rclcpp_components
	std_msgs
	geometry_msgs
)

# VICON NODE
ADD_EXECUTABLE(vicon_node vicon_node.cpp)
AMENT_TARGET_DEPENDENCIES(
  vicon_node
  rclcpp
  std_msgs
  geometry_msgs
)
TARGET_LINK_LIBRARIES(vicon_node ViconDataStreamSDK_CPP)
INSTALL(TARGETS vicon_node DESTINATION lib/${PROJECT_NAME})

# FORM ROS2 Package
AMENT_PACKAGE()
