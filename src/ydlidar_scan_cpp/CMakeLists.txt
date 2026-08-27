cmake_minimum_required(VERSION 3.8)
project(ydlidar_scan_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ydlidar_sdk REQUIRED)

# ydlidar_sdkConfig.cmake 只給裸函式庫名，沒有 imported target 也沒有 library dir。
get_filename_component(YDLIDAR_SDK_ROOT "${ydlidar_sdk_DIR}/../.." ABSOLUTE)
find_library(YDLIDAR_SDK_LIB
  NAMES ydlidar_sdk
  HINTS ${YDLIDAR_SDK_ROOT} ${YDLIDAR_SDK_LIBRARY_DIRS}
  REQUIRED
)

add_executable(ydlidar_scan_node src/ydlidar_scan_node.cpp)
ament_target_dependencies(ydlidar_scan_node rclcpp sensor_msgs std_srvs)
target_include_directories(ydlidar_scan_node PRIVATE ${YDLIDAR_SDK_INCLUDE_DIRS})
target_link_libraries(ydlidar_scan_node ${YDLIDAR_SDK_LIB} pthread rt)

install(TARGETS ydlidar_scan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
