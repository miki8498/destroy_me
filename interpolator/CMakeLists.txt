cmake_minimum_required(VERSION 3.8)
project(interpolator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

install(
  DIRECTORY include src
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

add_executable(interpolator src/ur_c_spline.cpp src/interpolator/Spline.cpp src/interpolator/FIRFilter.cpp) 
ament_target_dependencies(interpolator rclcpp rclcpp_components std_msgs geometry_msgs message Eigen3)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
# uncomment the following section in order to fill in
# further dependencies manually.

# find_package(<dependency> REQUIRED)

install(TARGETS
  interpolator
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
