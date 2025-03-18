cmake_minimum_required(VERSION 3.8)
project(livox_to_laserscan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(livox_to_laserscan src/livox_to_laserscan.cpp)
add_executable(custom_tf_pub src/custom_tf_pub.cpp)
target_include_directories(livox_to_laserscan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(custom_tf_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(livox_to_laserscan PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(custom_tf_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Link the libraries
ament_target_dependencies(livox_to_laserscan
  rclcpp
  livox_ros_driver2
  sensor_msgs
)
ament_target_dependencies(custom_tf_pub
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS livox_to_laserscan
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS custom_tf_pub
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp livox_ros_driver2 sensor_msgs geometry_msgs tf2_ros)
ament_export_include_directories(include)
ament_export_libraries(livox_to_laserscan custom_tf_pub)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
