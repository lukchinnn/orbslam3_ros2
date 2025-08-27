cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros)

set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++17 for ROS 2 Jazzy
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
find_package(ORB_SLAM3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include dirs
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  include
)

# ======= Sources =======
# set(COMMON_SRC
#   src/monocular/common.cpp
# )

# Monocular
add_executable(ros_mono
  src/monocular/ros2_mono.cpp
  src/monocular/common.cpp
  # src/monocular/monocular-slam-node.cpp
  ${COMMON_SRC}
)
ament_target_dependencies(ros_mono
  rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs std_srvs cv_bridge message_filters tf2 tf2_ros ORB_SLAM3 Pangolin OpenCV image_transport visualization_msgs
  std_srvs rosidl_default_runtime)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ros_mono "${cpp_typesupport_target}")
# Stereo
# add_executable(ros_stereo
#   src/stereo/ros_stereo.cpp
#   src/stereo/stereo-slam-node.cpp
#   ${COMMON_SRC}
# )
# ament_target_dependencies(ros_stereo
#   rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs std_srvs
#   cv_bridge message_filters tf2 tf2_ros ORB_SLAM3 Pangolin OpenCV
# )

# RGB-D
# add_executable(ros_rgbd
#   src/rgbd/ros_rgbd.cpp
#   src/rgbd/rgbd-slam-node.cpp
#   ${COMMON_SRC}
# )
# ament_target_dependencies(ros_rgbd
#   rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs std_srvs
#   cv_bridge message_filters tf2 tf2_ros ORB_SLAM3 Pangolin OpenCV
# )

# Stereo-Inertial
# add_executable(ros_stereo_inertial
#   src/stereo-inertial/ros_stereo_inertial.cpp
#   src/stereo-inertial/stereo-inertial-node.cpp
#   ${COMMON_SRC}
# )
# ament_target_dependencies(ros_stereo_inertial
#   rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs std_srvs
#   cv_bridge message_filters tf2 tf2_ros ORB_SLAM3 Pangolin OpenCV
# )

# ======= Install =======
install(TARGETS
  ros_mono
  # ros_stereo
  # ros_rgbd
  # ros_stereo_inertial
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/config files
install(DIRECTORY launch config Vocabulary
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
