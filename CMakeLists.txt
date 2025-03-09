cmake_minimum_required(VERSION 3.8)
project(mlx90640_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)

# External Project - libi2c
include(ExternalProject)
ExternalProject_Add(libi2c
    GIT_REPOSITORY https://github.com/amaork/libi2c.git
    UPDATE_DISCONNECTED true
    BUILD_IN_SOURCE true
    CONFIGURE_COMMAND ""
    BUILD_COMMAND make
    INSTALL_COMMAND ""
)
ExternalProject_Get_property(libi2c SOURCE_DIR)
set(LIBI2C_INCLUDE_DIR "${SOURCE_DIR}/include")
set(LIBI2C_LIB_PATH "${SOURCE_DIR}/libi2c.so")

# Include directories
include_directories(
  include/${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/wiring_for_ros/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Seeed_Arduino_MLX9064x/src
  ${LIBI2C_INCLUDE_DIR}  # Ensure libi2c headers are included
  ${rclcpp_INCLUDE_DIRS} # Ensure ROS 2 headers are included
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${rcutils_INCLUDE_DIRS}
)

# Create the main executable
add_executable(mlx90640_ros 
  src/mlx90640_ros.cpp
  src/wiring_for_ros/SPI.cpp
  src/wiring_for_ros/Wire.cpp
  src/wiring_for_ros/Arduino.cpp
  src/Seeed_Arduino_MLX9064x/src/MLX90640_API.cpp
  src/Seeed_Arduino_MLX9064x/src/MLX9064X_I2C_Driver.cpp
)

# Ensure libi2c builds before mlx90640_ros
add_dependencies(mlx90640_ros libi2c)

# Link libraries
target_link_libraries(mlx90640_ros ${LIBI2C_LIB_PATH} gpiod rclcpp::rclcpp rcutils::rcutils)

# Set C++ standard
set_target_properties(mlx90640_ros PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)

# Install executable
install(TARGETS mlx90640_ros DESTINATION lib/${PROJECT_NAME})

# Enable tests if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
