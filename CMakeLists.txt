cmake_minimum_required(VERSION 3.8)
project(path_smoother)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# cubic_spline_1d_node
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

# cubic_spline_2d_node
add_executable(cubic_spline_2d_node src/cubic_spline_2d_node.cpp src/cubic_spline_1d_component.cpp src/cubic_spline_2d_component.cpp)
ament_target_dependencies(cubic_spline_2d_node
  rclcpp
  OpenCV
)
target_include_directories(cubic_spline_2d_node PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(cubic_spline_2d_node
  ${EIGEN3_LIBRARIES}
)

# cubic_spline_node
add_executable(cubic_spline_node src/cubic_spline_node.cpp src/cubic_spline_1d_component.cpp src/cubic_spline_2d_component.cpp)
ament_target_dependencies(cubic_spline_node
  rclcpp
  tf2_geometry_msgs
  tf2_ros
  nav_msgs
  OpenCV
)
target_include_directories(cubic_spline_node PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(cubic_spline_node
  ${EIGEN3_LIBRARIES}
)

# path_publisher
add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher
  rclcpp
  nav_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS
  cubic_spline_1d_node
  cubic_spline_2d_node
  cubic_spline_node
  path_publisher
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Install other files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_dependencies(OpenCV) 

# include ディレクトリをエクスポート
#ament_export_include_directories(include)

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
