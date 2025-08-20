cmake_minimum_required(VERSION 3.8)
project(foxglove_compressed_video)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(depthai_ros_driver REQUIRED)
find_package(foxglove_bridge REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GSTAPP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_RTSP_SERVER REQUIRED gstreamer-rtsp-server-1.0)

add_executable(foxglove_compressed_video_stream src/foxglove_compressed_video_stream.cpp)
target_include_directories(foxglove_compressed_video_stream PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(foxglove_compressed_video_stream PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(foxglove_compressed_video_stream
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  foxglove_msgs
  depthai_ros_driver
  foxglove_bridge
)

target_include_directories(foxglove_compressed_video_stream PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GSTAPP_INCLUDE_DIRS}
  ${GST_RTSP_SERVER_INCLUDE_DIRS}
)
target_link_libraries(foxglove_compressed_video_stream
  gstrtspserver-1.0
  ${GST_LIBRARIES}
  ${GSTAPP_LIBRARIES}
  ${GST_RTSP_SERVER_INCLUDE_DIRS}
)

install(TARGETS foxglove_compressed_video_stream
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
