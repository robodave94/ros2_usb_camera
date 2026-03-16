/**
 * @file usb_camera_calibration_node.cpp
 * @brief Standalone executable for CameraCalibrationDriver.
 */

#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "usb_camera_calibration_driver.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<usb_camera_driver::CameraCalibrationDriver>(options);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
