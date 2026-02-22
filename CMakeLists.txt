cmake_minimum_required(VERSION 3.8)
project(control_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories( include ${PCL_INCLUDE_DIRS} )

# Reference Manager Node
add_executable(reference_manager_node 
  src/reference_manager_node.cpp
)
ament_target_dependencies(reference_manager_node
  rclcpp geometry_msgs std_srvs
)

# PID Controller Node
add_executable(pid_control_node 
  src/pid_control_node.cpp
)
ament_target_dependencies(pid_control_node
  rclcpp geometry_msgs std_msgs
)

# Motor Command Node
add_executable(motor_command_node 
  src/motor_command_node.cpp
)
ament_target_dependencies(motor_command_node
  rclcpp std_msgs geometry_msgs std_srvs
)

# IMU Quaternion to Euler Node (Used for gazebo simulation)
add_executable(imu_quat_to_euler_node src/imu_quat_to_euler_node.cpp)

ament_target_dependencies(imu_quat_to_euler_node
  rclcpp sensor_msgs std_msgs
)

# LaserScan to PointCloud Node (Used for gazebo simulation)
add_executable(laserscan_to_pointcloud_node 
  src/laser_to_pointcloud.cpp
  )
  ament_target_dependencies(laserscan_to_pointcloud_node
   rclcpp std_msgs sensor_msgs geometry_msgs tf2 pcl_conversions tf2_geometry_msgs
  )

# PCA Node
add_executable(pca_node 
  src/pca_node.cpp
  )
  ament_target_dependencies(pca_node
   rclcpp std_msgs sensor_msgs geometry_msgs tf2 pcl_conversions tf2_geometry_msgs std_srvs  
  )

  # Topic Health Monitor Node
add_executable(topic_health_monitor_node 
  src/topic_health_monitor.cpp
  )
  ament_target_dependencies(topic_health_monitor_node
   rclcpp std_msgs sensor_msgs diagnostic_msgs std_srvs 
  )
  
# Python nodes don't compile, so no add_executable needed
install(PROGRAMS
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(laserscan_to_pointcloud_node
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

# Install executables
install(TARGETS
  reference_manager_node
  pid_control_node
  motor_command_node
  imu_quat_to_euler_node
  laserscan_to_pointcloud_node
  pca_node
  topic_health_monitor_node
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

install(DIRECTORY 
  description
  launch
  bringup 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
