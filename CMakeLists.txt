cmake_minimum_required(VERSION 3.5)
project(gnss_imu_esekf)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp)

add_executable(sensor_sub_node src/sensor_sub_node.cpp)
ament_target_dependencies(sensor_sub_node rclcpp sensor_msgs)

add_executable(esekf_node src/esekf_node.cpp)
ament_target_dependencies(esekf_node rclcpp sensor_msgs nav_msgs tf2_ros)

target_link_libraries(esekf_node Geographic)

install(TARGETS
  test_node
  sensor_sub_node
  esekf_node
  DESTINATION lib/${PROJECT_NAME}
)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
