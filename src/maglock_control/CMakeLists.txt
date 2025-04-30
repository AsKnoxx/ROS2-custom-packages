cmake_minimum_required(VERSION 3.8)
project(maglock_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_library(JETSON_GPIO_LIBRARY NAMES JetsonGPIO)

add_executable(maglock_controller src/maglock_controller.cpp)
ament_target_dependencies(maglock_controller rclcpp std_msgs)

target_link_libraries(maglock_controller ${JETSON_GPIO_LIBRARY})

install(TARGETS maglock_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
