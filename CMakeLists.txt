cmake_minimum_required(VERSION 3.5.0)

project(usb_camera_driver)

# Default to C++17 (required for std::filesystem)
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(OpenCV 4 REQUIRED)

# ─── System: libudev ──────────────────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUDEV REQUIRED libudev)

include_directories(
  include
  ${LIBUDEV_INCLUDE_DIRS}
)

# ═══════════════════════════════════════════════════════════════════════════════
# 1.  usb_camera_driver  (original node — unchanged)
# ═══════════════════════════════════════════════════════════════════════════════
add_library(usb_camera_driver SHARED
  src/usb_camera_driver.cpp)
target_compile_definitions(usb_camera_driver
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(usb_camera_driver
  rclcpp rclcpp_components sensor_msgs std_msgs
  camera_info_manager image_transport camera_calibration_parsers OpenCV)
rclcpp_components_register_nodes(usb_camera_driver "usb_camera_driver::CameraDriver")
set(node_plugins "${node_plugins}usb_camera_driver::CameraDriver;$<TARGET_FILE:usb_camera_driver>\n")

add_executable(usb_camera_driver_node
  src/usb_camera_driver_node.cpp)
target_link_libraries(usb_camera_driver_node usb_camera_driver)
ament_target_dependencies(usb_camera_driver_node rclcpp)

# ═══════════════════════════════════════════════════════════════════════════════
# 2.  usb_device_detector  (shared utility, links libudev)
# ═══════════════════════════════════════════════════════════════════════════════
add_library(usb_device_detector SHARED
  src/usb_device_detector.cpp)
target_include_directories(usb_device_detector
  PUBLIC include ${LIBUDEV_INCLUDE_DIRS})
target_link_libraries(usb_device_detector
  ${LIBUDEV_LIBRARIES})
ament_target_dependencies(usb_device_detector rclcpp)

# ═══════════════════════════════════════════════════════════════════════════════
# 3.  usb_camera_run_driver  (autodetect + image_transport run node)
# ═══════════════════════════════════════════════════════════════════════════════
add_library(usb_camera_run_driver SHARED
  src/usb_camera_run_driver.cpp)
target_compile_definitions(usb_camera_run_driver
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(usb_camera_run_driver
  usb_device_detector)
ament_target_dependencies(usb_camera_run_driver
  rclcpp rclcpp_components sensor_msgs std_msgs
  camera_info_manager image_transport camera_calibration_parsers OpenCV)
rclcpp_components_register_nodes(usb_camera_run_driver
  "usb_camera_driver::CameraRunDriver")

add_executable(usb_camera_run_node
  src/usb_camera_run_node.cpp)
target_link_libraries(usb_camera_run_node
  usb_camera_run_driver)
ament_target_dependencies(usb_camera_run_node rclcpp)

# ═══════════════════════════════════════════════════════════════════════════════
# 4.  usb_camera_calibration_driver  (one-shot calibration node)
# ═══════════════════════════════════════════════════════════════════════════════
add_library(usb_camera_calibration_driver SHARED
  src/usb_camera_calibration_driver.cpp)
target_compile_definitions(usb_camera_calibration_driver
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(usb_camera_calibration_driver
  usb_device_detector)
ament_target_dependencies(usb_camera_calibration_driver
  rclcpp rclcpp_components sensor_msgs
  camera_calibration_parsers OpenCV)
rclcpp_components_register_nodes(usb_camera_calibration_driver
  "usb_camera_driver::CameraCalibrationDriver")

add_executable(usb_camera_calibration_node
  src/usb_camera_calibration_node.cpp)
target_link_libraries(usb_camera_calibration_node
  usb_camera_calibration_driver)
ament_target_dependencies(usb_camera_calibration_node rclcpp)

# ═══════════════════════════════════════════════════════════════════════════════
# Install
# ═══════════════════════════════════════════════════════════════════════════════
install(TARGETS
  usb_camera_driver
  usb_device_detector
  usb_camera_run_driver
  usb_camera_calibration_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  usb_camera_driver_node
  usb_camera_run_node
  usb_camera_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ═══════════════════════════════════════════════════════════════════════════════
# Tests
# ═══════════════════════════════════════════════════════════════════════════════
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
