cmake_minimum_required(VERSION 3.8)
project(firefly)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-O3")
set(WITH_TBB "ON")
set(WITH_OPENMP "ON")
set(WITH_IPP "ON")
set(BUILD_EXAMPLES "OFF")
set(WITH_NVCUVID "ON")
set(WITH_CUDA "ON")
set(BUILD_DOCS "OFF")
set(BUILD_PERF_TESTS "OFF")
set(BUILD_TESTS "OFF")
set(WITH_CSTRIPES "ON")
set(WITH_OPENCL "ON")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(firefly src/firefly.cpp)
target_include_directories(firefly PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(firefly PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  firefly
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  OpenCV
  cv_bridge
)

install(TARGETS firefly
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files.
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
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
