cmake_minimum_required(VERSION 3.8)
project(path_smoother)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(cubic_spline_1d_node src/cubic_spline_1d_node.cpp src/cubic_spline_1d_component.cpp)
ament_target_dependencies(cubic_spline_1d_node
  rclcpp
  OpenCV
)
target_include_directories(cubic_spline_1d_node PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(cubic_spline_1d_node
  ${EIGEN3_LIBRARIES}
)

# Install the executable
install(TARGETS
  cubic_spline_1d_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Export dependencies
ament_export_dependencies(OpenCV) 

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
