cmake_minimum_required(VERSION 3.8)
project(point_cloud_op)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message REQUIRED)
find_package(robotiq_hand_e REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3)
find_package(eigen3_cmake_module REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
set(Qt5_DIR /usr/local/Cellar/qt@5/5.15.2_1/lib/cmake/Qt5)

install(
  DIRECTORY include src 
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include ${PCL_INCLUDE_DIRS} /opt/ros/humble/include/pluginlib/pluginlib)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS}) 

add_executable(point_cloud_remove src/Main.cpp src/point_cloud_op/point_cloud_op.cpp) 
target_link_libraries(point_cloud_remove ${PCL_LIBRARIES})
ament_target_dependencies(point_cloud_remove rclcpp rclcpp_components geometry_msgs sensor_msgs pcl_conversions message std_srvs Eigen3 robotiq_hand_e)
ament_export_dependencies(Eigen3 ${MOVEIT_LIB_NAME})


install(TARGETS
  point_cloud_remove
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
