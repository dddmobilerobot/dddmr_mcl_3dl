cmake_minimum_required(VERSION 3.5)
project(mcl_3dl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  message_filters
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  pluginlib
  pcl_conversions
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# sub_maps
add_library(sub_maps SHARED
  src/sub_maps.cpp
)
target_link_libraries(sub_maps
  ${PCL_LIBRARIES}
)
ament_target_dependencies(sub_maps
  ${dependencies}
)

# parameters
add_library(parameters SHARED
  src/parameters.cpp
)
target_link_libraries(parameters
  ${PCL_LIBRARIES}
)
ament_target_dependencies(parameters
  ${dependencies}
)

# lidar_measurement_model_likelihood
add_library(lidar_measurement_model_likelihood SHARED
  src/lidar_measurement_model_likelihood.cpp
)
target_link_libraries(lidar_measurement_model_likelihood
  parameters
  ${PCL_LIBRARIES}
)
ament_target_dependencies(lidar_measurement_model_likelihood
  ${dependencies}
)

#mcl 3dl node
add_executable(mcl_3dl
  src/mcl_3dl.cpp
  src/mcl_3dl_node.cpp
)
target_link_libraries(mcl_3dl 
  parameters
  lidar_measurement_model_likelihood
  sub_maps
)
ament_target_dependencies(mcl_3dl
  ${dependencies}
)

#pcl publisher node
add_executable(pcl_publisher
  src/pcl_publisher.cpp
)
target_link_libraries(pcl_publisher
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pcl_publisher
  ${dependencies}
)

install(TARGETS
  parameters
  lidar_measurement_model_likelihood
  sub_maps
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  mcl_3dl
  pcl_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  "launch"
  "config"
  "rviz"
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
