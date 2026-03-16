/**
 * @file usb_camera_calibration_driver.hpp
 * @brief ROS 2 node for one-shot camera calibration.
 *
 * Detects camera hardware by USB PID:VID (+ optional serial), captures frames
 * directly via OpenCV, collects checkerboard detections, runs cv::calibrateCamera,
 * and saves both an OpenCV FileStorage YAML and a ROS camera_calibration_parsers YAML.
 * The output filenames are derived from the camera's USB serial number.
 */
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"

namespace usb_camera_driver
{

class CameraCalibrationDriver : public rclcpp::Node
{
public:
  explicit CameraCalibrationDriver(const rclcpp::NodeOptions & options);

private:
  // ─── parameters ──────────────────────────────────────────────────────────
  std::string pid_vid_;
  std::string serial_number_;
  std::string calibration_output_dir_;
  int         checkerboard_rows_;
  int         checkerboard_cols_;
  double      square_size_;
  int         num_frames_;
  int         image_width_;
  int         image_height_;
  std::string capture_backend_;  ///< "any" (default) | "v4l2" | "gstreamer"
  std::string pixel_format_;     ///< "" = no override | "MJPG" | "YUYV" | "BGR3" …

  // ─── derived ─────────────────────────────────────────────────────────────
  std::string resolved_device_;
  std::string resolved_serial_;   ///< USB serial (or "vid_pid" fallback for filenames)
  cv::Size    board_size_;

  // ─── calibration state ───────────────────────────────────────────────────
  std::vector<std::vector<cv::Point3f>> obj_points_;
  std::vector<std::vector<cv::Point2f>> img_points_;
  std::vector<cv::Point3f>              obj_template_;
  cv::Size image_size_;
  int  frames_collected_{0};
  bool calibration_done_{false};
  std::mutex mutex_;

  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;

  // ─── helpers ─────────────────────────────────────────────────────────────
  bool detectDevice();
  void calibrationLoop();
  void runCalibration();
  bool saveCalibration(const cv::Mat & camera_matrix,
                       const cv::Mat & dist_coeffs,
                       const cv::Size & image_size,
                       double rms);
};

}  // namespace usb_camera_driver
