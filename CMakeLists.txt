cmake_minimum_required(VERSION 3.8)
project(mocap-transforms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable for mocap_tf2_broadcaster
add_executable(mocap_tf2_broadcaster src/mocap_tf2_broadcaster.cpp)
target_link_libraries(mocap_tf2_broadcaster PUBLIC
      ${geometry_msgs_TARGETS}
      ${mocap4r2_msgs_TARGETS}
      ${tf2_msgs_TARGETS}
      rclcpp::rclcpp
      tf2::tf2
      tf2_ros::static_transform_broadcaster_node
      tf2_ros::tf2_ros
    )

# Add executable for mocap_tf2_post_processor
add_executable(mocap_tf2_post_processor src/mocap_tf2_post_processor.cpp)
target_link_libraries(mocap_tf2_post_processor PUBLIC
      ${geometry_msgs_TARGETS}
      rclcpp::rclcpp
      tf2::tf2
      tf2_ros::tf2_ros
      tf2_geometry_msgs::tf2_geometry_msgs
    )

# Install the executable
install(TARGETS
  mocap_tf2_broadcaster
  mocap_tf2_post_processor
  DESTINATION lib/${PROJECT_NAME}
)

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
