cmake_minimum_required(VERSION 3.8)
project(jetson_rgbd_udp_sender)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ZLIB REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(
  GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

set(
  ZDEPTH_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Zdepth"
  CACHE PATH
  "Path to the pinned catid/Zdepth source tree"
)

if(NOT EXISTS "${ZDEPTH_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "Zdepth source not found at ${ZDEPTH_SOURCE_DIR}. "
    "Restore the vendored third_party/Zdepth directory from this repository.")
endif()

add_subdirectory(
  "${ZDEPTH_SOURCE_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/zdepth"
  EXCLUDE_FROM_ALL
)

add_library(rgbd_sender_core
  src/calibration.cpp
  src/h265_encoder.cpp
  src/image_input.cpp
  src/sender_node.cpp
  src/udp_protocol.cpp
  src/udp_sender.cpp
  src/zdepth_encoder.cpp
)

target_include_directories(rgbd_sender_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(rgbd_sender_core
  rclcpp
  sensor_msgs
  message_filters
)

target_link_libraries(rgbd_sender_core
  PkgConfig::GSTREAMER
  ZLIB::ZLIB
  nlohmann_json::nlohmann_json
  Threads::Threads
  zdepth::zdepth
)

add_executable(rgbd_udp_sender src/main.cpp)
target_link_libraries(rgbd_udp_sender rgbd_sender_core)
ament_target_dependencies(rgbd_udp_sender rclcpp)

install(
  TARGETS rgbd_sender_core rgbd_udp_sender
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/check_jetson_deps.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs message_filters)

ament_package()
