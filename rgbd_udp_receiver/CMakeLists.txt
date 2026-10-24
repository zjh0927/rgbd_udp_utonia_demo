cmake_minimum_required(VERSION 3.8)
project(rgbd_udp_receiver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 Humble on Ubuntu 22.04 is built for Python 3.10. Find it before
# ament_cmake so an active Conda Python cannot be cached by ament.
find_package(Python3 3.10 EXACT REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11 REQUIRED)

set(ZDEPTH_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Zdepth"
  CACHE PATH "Path to the pinned catid/Zdepth source tree")
if(NOT EXISTS "${ZDEPTH_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "Zdepth source not found at ${ZDEPTH_SOURCE_DIR}. "
    "Restore the vendored third_party/Zdepth directory from this repository.")
endif()

add_subdirectory("${ZDEPTH_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/zdepth" EXCLUDE_FROM_ALL)

add_library(rgbd_receiver_zdepth STATIC src/zdepth_decoder.cpp)
target_include_directories(rgbd_receiver_zdepth PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(rgbd_receiver_zdepth PUBLIC zdepth::zdepth)
set_target_properties(rgbd_receiver_zdepth PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(_zdepth MODULE src/zdepth_pybind.cpp)
target_link_libraries(_zdepth PRIVATE rgbd_receiver_zdepth Python3::NumPy)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS _zdepth LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
install(PROGRAMS scripts/rgbd_udp_receiver DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/check_receiver_deps.sh
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
