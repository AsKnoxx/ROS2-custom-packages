cmake_minimum_required(VERSION 3.8)
project(maglock_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

# Find Jetson GPIO library
find_library(JETSON_GPIO_LIBRARY NAMES JetsonGPIO REQUIRED)

# Include header files
include_directories(include)

# Build the maglock_control_server node
add_executable(maglock_control_server
  src/maglock_control_server.cpp
  src/gpio_control.cpp
)

ament_target_dependencies(maglock_control_server
  rclcpp
  example_interfaces
)

target_link_libraries(maglock_control_server
  ${JETSON_GPIO_LIBRARY}
)

# Install the node
install(TARGETS maglock_control_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
