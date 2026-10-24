cmake_minimum_required(VERSION 3.8)
project(utonia_pca_inference)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

install(PROGRAMS
  scripts/utonia_pca_node
  scripts/check_utonia_deps.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
