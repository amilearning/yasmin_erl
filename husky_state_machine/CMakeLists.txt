cmake_minimum_required(VERSION 3.5)
project(husky_state_machine)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined." )
endif()
if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  message(STATUS "Build for ROS 2 Foxy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DFOXY")
elseif("$ENV{ROS_DISTRO}" STREQUAL "galactic")
  message(STATUS "Build for ROS 2 Galactic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGALACTIC")
elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(STATUS "Build for ROS 2 Humble")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHUMBLE")
elseif("$ENV{ROS_DISTRO}" STREQUAL "iron")
  message(STATUS "Build for ROS 2 Iron")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIRON")
elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  message(STATUS "Build for ROS 2 Jazzy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DJAZZY")
elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  message(STATUS "Build for ROS 2 Rolling")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROLLING")
else()
  message(FATAL_ERROR "Unsupported ROS Distribution: " "$ENV{ROS_DISTRO}")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(franka_msgs REQUIRED)


find_package(nav_msgs REQUIRED)
find_package(example_interfaces REQUIRED)


# Python
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  husky_state_machine/temp_monitoring.py    
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
