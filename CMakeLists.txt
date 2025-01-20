cmake_minimum_required(VERSION 3.5)
project(rosify)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Include rclpy as a dependency

# Install Python script as an executable
install(
  PROGRAMS
    src/turtleJoy.py  # Path to your Python script
  DESTINATION lib/${PROJECT_NAME}  # Destination directory for executables
)

# Install the `launch` directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark the package for ament
ament_package()
