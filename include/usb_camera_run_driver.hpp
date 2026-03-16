/**
 * @file usb_camera_run_driver.hpp
 * @brief ROS 2 node that finds a USB camera by PID:VID+serial, loads a
 *        calibration file (with format validation), and publishes image_raw
 *        via image_transport (enabling compressed/theora/etc transport plugins).
 */
#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace usb_camera_driver
{

class CameraRunDriver : public rclcpp::Node
{
public:
  explicit CameraRunDriver(const rclcpp::NodeOptions & options);

private:
  // ─── parameters ──────────────────────────────────────────────────────────
  std::string pid_vid_;
  std::string serial_number_;
  std::string camera_name_;
  std::string calibration_file_;
  std::string calibration_file_type_;  // "ros" | "cv"
  int         image_width_;
  int         image_height_;
  double      fps_;
  std::string capture_backend_;  ///< "any" (default) | "v4l2" | "gstreamer"
  std::string pixel_format_;     ///< "" = no override | "MJPG" | "YUYV" | "BGR3" …

  // ─── state ───────────────────────────────────────────────────────────────
  std::string resolved_device_;  ///< /dev/videoN chosen after detection
  cv::VideoCapture cap_;
  cv::Mat frame_;
  bool calibration_loaded_{false};

  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
  std::chrono::steady_clock::time_point last_frame_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
  image_transport::CameraPublisher camera_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ─── internal helpers ────────────────────────────────────────────────────
  bool detectDevice();
  bool loadCalibration();
  void imageCallback();
  std::shared_ptr<sensor_msgs::msg::Image> convertFrameToMessage(cv::Mat & frame);

  /// Sniff first ~30 lines of a file and return detected format: "ros", "cv", or "unknown"
  static std::string detectCalibrationFileFormat(const std::string & filepath);
};

}  // namespace usb_camera_driver
