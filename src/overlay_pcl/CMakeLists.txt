cmake_minimum_required(VERSION 3.8)
project(overlay_pcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include ${PCL_CONVERSIONS_INCLUDE_DIRS})

add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
  PCL
  pcl_conversions
)

install(TARGETS
  image_subscriber
  DESTINATION lib/${PROJECT_NAME})


ament_package()
