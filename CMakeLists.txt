cmake_minimum_required(VERSION 3.8)
project(kratos_isha)

# Enable better warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find core dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python scripts
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install each Python script as an executable
install(PROGRAMS
  scripts/q1_publisher.py
  scripts/q1_subscriber.py
  scripts/q2_signal_1.py
  scripts/q2_signal_2.py
  scripts/q3_rover_status_publisher.py
  scripts/q4_clock_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Optional: disable these if no copyright/license yet
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
