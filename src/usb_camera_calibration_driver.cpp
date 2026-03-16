/**
 * @file usb_camera_calibration_driver.cpp
 * @brief One-shot camera calibration node using direct VideoCapture.
 */

#include "usb_camera_calibration_driver.hpp"
#include "serial/serial.h"

#include <filesystem>
#include <sstream>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "camera_calibration_parsers/parse.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace usb_camera_driver
{

CameraCalibrationDriver::CameraCalibrationDriver(const rclcpp::NodeOptions & options)
: Node("usb_camera_calibration_driver", options)
{
  // ── parameters ──
  pid_vid_                = this->declare_parameter<std::string>("pid_vid", "");
  serial_number_          = this->declare_parameter<std::string>("serial_number", "");
  calibration_output_dir_ = this->declare_parameter<std::string>("calibration_output_dir", ".");
  checkerboard_rows_      = this->declare_parameter<int>("checkerboard_rows", 6);
  checkerboard_cols_      = this->declare_parameter<int>("checkerboard_cols", 9);
  square_size_            = this->declare_parameter<double>("square_size", 0.025);
  num_frames_             = this->declare_parameter<int>("num_frames", 20);
  image_width_            = this->declare_parameter<int>("image_width", 1280);
  image_height_           = this->declare_parameter<int>("image_height", 720);
  capture_backend_        = this->declare_parameter<std::string>("capture_backend", "any");
  pixel_format_           = this->declare_parameter<std::string>("pixel_format", "");

  if (pid_vid_.empty()) {
    RCLCPP_FATAL(this->get_logger(),
      "Parameter 'pid_vid' is required. Provide e.g. --ros-args -p pid_vid:=\"046d:085c\"");
    rclcpp::shutdown();
    return;
  }

  if (!detectDevice()) {
    rclcpp::shutdown();
    return;
  }

  // Build board size and object point template
  board_size_ = cv::Size(checkerboard_cols_, checkerboard_rows_);
  for (int r = 0; r < checkerboard_rows_; ++r) {
    for (int c = 0; c < checkerboard_cols_; ++c) {
      obj_template_.push_back(cv::Point3f(
        static_cast<float>(c) * static_cast<float>(square_size_),
        static_cast<float>(r) * static_cast<float>(square_size_),
        0.0f));
    }
  }

  auto resolveBackend = [](const std::string & b) -> int {
    if (b == "v4l2")      return cv::CAP_V4L2;
    if (b == "gstreamer") return cv::CAP_GSTREAMER;
    if (b == "ffmpeg")    return cv::CAP_FFMPEG;
    return cv::CAP_ANY;
  };
  int backend_api = resolveBackend(capture_backend_);
  cap_.open(resolved_device_, backend_api);
  if (!cap_.isOpened()) {
    RCLCPP_FATAL(this->get_logger(),
      "Failed to open device: %s (backend: %s)",
      resolved_device_.c_str(), capture_backend_.c_str());
    rclcpp::shutdown();
    return;
  }
  if (!pixel_format_.empty()) {
    if (pixel_format_.size() != 4) {
      RCLCPP_WARN(this->get_logger(),
        "Parameter 'pixel_format' must be a 4-character FOURCC string (e.g. \"MJPG\"), "
        "got \"%s\" — ignoring.", pixel_format_.c_str());
    } else {
      double fourcc = cv::VideoWriter::fourcc(
        pixel_format_[0], pixel_format_[1], pixel_format_[2], pixel_format_[3]);
      cap_.set(cv::CAP_PROP_FOURCC, fourcc);
      RCLCPP_INFO(this->get_logger(), "Pixel format requested: %s", pixel_format_.c_str());
    }
  }
  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  static_cast<double>(image_width_));
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(image_height_));

  RCLCPP_INFO(this->get_logger(), "=== Camera Calibration Node ===");
  RCLCPP_INFO(this->get_logger(), "  Device:        %s", resolved_device_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Serial:        %s", resolved_serial_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Board:         %dx%d inner corners",
    checkerboard_cols_, checkerboard_rows_);
  RCLCPP_INFO(this->get_logger(), "  Square size:   %.4f m", square_size_);
  RCLCPP_INFO(this->get_logger(), "  Frames needed: %d", num_frames_);
  RCLCPP_INFO(this->get_logger(), "  Output dir:    %s", calibration_output_dir_.c_str());
  RCLCPP_INFO(this->get_logger(), "Show checkerboard in front of camera...");

  // ~33 Hz capture loop
  timer_ = this->create_wall_timer(
    30ms, std::bind(&CameraCalibrationDriver::calibrationLoop, this));
}

// ─── device detection ────────────────────────────────────────────────────────

bool CameraCalibrationDriver::detectDevice()
{
  // Find all /dev/videoN paths matching the VID:PID via serial-ros2
  std::vector<std::string> paths;
  try {
    paths = serial::findVideoDevicesByPIDVID(pid_vid_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "USB enumeration failed: %s", e.what());
    return false;
  }

  // Pair each path with its USB serial number
  struct DevEntry { std::string device; std::string serial; };
  std::vector<DevEntry> matches;
  for (const auto & path : paths) {
    matches.push_back({path, serial::getUSBSerialForVideoDevice(path)});
  }

  if (matches.empty()) {
    RCLCPP_FATAL(this->get_logger(),
      "No video device found for VID:PID = %s", pid_vid_.c_str());
    return false;
  }

  if (matches.size() == 1) {
    if (!serial_number_.empty() && matches[0].serial != serial_number_) {
      RCLCPP_FATAL(this->get_logger(),
        "Found device %s but serial \"%s\" != requested \"%s\"",
        matches[0].device.c_str(),
        matches[0].serial.c_str(),
        serial_number_.c_str());
      return false;
    }
    resolved_device_ = matches[0].device;
    resolved_serial_ = matches[0].serial.empty()
      ? [this]{ std::string f = pid_vid_; auto p = f.find(':'); if (p != std::string::npos) f[p] = '_'; return f; }()
      : matches[0].serial;
    RCLCPP_INFO(this->get_logger(), "Using device: %s (serial: \"%s\")",
      resolved_device_.c_str(), resolved_serial_.c_str());
    return true;
  }

  // Multiple matches
  if (serial_number_.empty()) {
    std::ostringstream oss;
    oss << matches.size() << " devices match VID:PID=" << pid_vid_
        << ", no serial_number set. Found:\n";
    for (auto & m : matches) {
      oss << "  " << m.device << "  serial=\"" << m.serial << "\"\n";
    }
    oss << "Set -p serial_number:=<serial> to pick one.";
    RCLCPP_FATAL(this->get_logger(), "%s", oss.str().c_str());
    return false;
  }

  std::vector<DevEntry> filtered;
  for (auto & m : matches) {
    if (m.serial == serial_number_) filtered.push_back(m);
  }
  if (filtered.empty()) {
    std::ostringstream oss;
    oss << "Serial \"" << serial_number_ << "\" not found. Available:";
    for (auto & m : matches) {
      oss << "\n  \"" << m.serial << "\" → " << m.device;
    }
    RCLCPP_FATAL(this->get_logger(), "%s", oss.str().c_str());
    return false;
  }

  resolved_device_ = filtered[0].device;
  resolved_serial_ = filtered[0].serial.empty()
    ? [this]{ std::string f = pid_vid_; auto p = f.find(':'); if (p != std::string::npos) f[p] = '_'; return f; }()
    : filtered[0].serial;
  RCLCPP_INFO(this->get_logger(), "Using device: %s (serial: \"%s\")",
    resolved_device_.c_str(), resolved_serial_.c_str());
  return true;
}

// ─── capture / detection loop ─────────────────────────────────────────────────

void CameraCalibrationDriver::calibrationLoop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (calibration_done_) return;

  cv::Mat frame;
  cap_ >> frame;
  if (frame.empty()) return;

  image_size_ = frame.size();

  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(
    gray, board_size_, corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

  if (found) {
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

    obj_points_.push_back(obj_template_);
    img_points_.push_back(corners);
    ++frames_collected_;

    RCLCPP_INFO(this->get_logger(), "Checkerboard captured: frame %d / %d",
      frames_collected_, num_frames_);

    if (frames_collected_ >= num_frames_) {
      runCalibration();
    }
  }
}

// ─── calibrate ────────────────────────────────────────────────────────────────

void CameraCalibrationDriver::runCalibration()
{
  RCLCPP_INFO(this->get_logger(),
    "Running cv::calibrateCamera with %d frames...", frames_collected_);

  cv::Mat camera_matrix, dist_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;

  double rms = cv::calibrateCamera(
    obj_points_, img_points_, image_size_,
    camera_matrix, dist_coeffs, rvecs, tvecs);

  RCLCPP_INFO(this->get_logger(), "=== Calibration Complete ===");
  RCLCPP_INFO(this->get_logger(), "RMS reprojection error: %.6f", rms);
  RCLCPP_INFO(this->get_logger(), "fx=%.4f  fy=%.4f  cx=%.4f  cy=%.4f",
    camera_matrix.at<double>(0, 0), camera_matrix.at<double>(1, 1),
    camera_matrix.at<double>(0, 2), camera_matrix.at<double>(1, 2));

  std::ostringstream dist_ss;
  for (int i = 0; i < dist_coeffs.cols * dist_coeffs.rows; ++i) {
    if (i) dist_ss << ", ";
    dist_ss << dist_coeffs.at<double>(i);
  }
  RCLCPP_INFO(this->get_logger(), "Distortion: [%s]", dist_ss.str().c_str());

  if (saveCalibration(camera_matrix, dist_coeffs, image_size_, rms)) {
    RCLCPP_INFO(this->get_logger(), "Calibration saved. Node shutting down.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to save one or more calibration files.");
  }

  calibration_done_ = true;
  timer_->cancel();
  cap_.release();
  rclcpp::shutdown();
}

// ─── save both formats ────────────────────────────────────────────────────────

bool CameraCalibrationDriver::saveCalibration(const cv::Mat & camera_matrix,
                                               const cv::Mat & dist_coeffs,
                                               const cv::Size & image_size,
                                               double rms)
{
  try {
    fs::create_directories(calibration_output_dir_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Cannot create output dir '%s': %s",
      calibration_output_dir_.c_str(), e.what());
    return false;
  }

  std::string stem    = resolved_serial_ + "_calib";
  std::string cv_path  = (fs::path(calibration_output_dir_) / (stem + "_cv.yaml")).string();
  std::string ros_path = (fs::path(calibration_output_dir_) / (stem + "_ros.yaml")).string();

  bool ok = true;

  // ── 1. OpenCV FileStorage YAML ──────────────────────────────────────────
  {
    cv::FileStorage fs_out(cv_path, cv::FileStorage::WRITE);
    if (!fs_out.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open for writing: %s", cv_path.c_str());
      ok = false;
    } else {
      fs_out << "rms_error"     << rms;
      fs_out << "image_width"   << image_size.width;
      fs_out << "image_height"  << image_size.height;
      fs_out << "camera_matrix" << camera_matrix;
      fs_out << "dist_coeffs"   << dist_coeffs;
      fs_out.release();
      RCLCPP_INFO(this->get_logger(), "Saved OpenCV calibration → %s", cv_path.c_str());
    }
  }

  // ── 2. ROS camera_calibration_parsers YAML ──────────────────────────────
  {
    sensor_msgs::msg::CameraInfo ci;
    ci.width  = static_cast<uint32_t>(image_size.width);
    ci.height = static_cast<uint32_t>(image_size.height);
    ci.distortion_model = "plumb_bob";

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        ci.k[r * 3 + c] = camera_matrix.at<double>(r, c);
      }
    }

    for (int i = 0; i < dist_coeffs.cols * dist_coeffs.rows; ++i) {
      ci.d.push_back(dist_coeffs.at<double>(i));
    }

    // R = identity
    ci.r[0] = 1.0; ci.r[4] = 1.0; ci.r[8] = 1.0;

    // P = [K | 0]
    ci.p[0]  = ci.k[0]; ci.p[1]  = ci.k[1]; ci.p[2]  = ci.k[2]; ci.p[3]  = 0.0;
    ci.p[4]  = ci.k[3]; ci.p[5]  = ci.k[4]; ci.p[6]  = ci.k[5]; ci.p[7]  = 0.0;
    ci.p[8]  = ci.k[6]; ci.p[9]  = ci.k[7]; ci.p[10] = ci.k[8]; ci.p[11] = 0.0;

    if (!camera_calibration_parsers::writeCalibration(ros_path, resolved_serial_, ci)) {
      RCLCPP_ERROR(this->get_logger(),
        "camera_calibration_parsers failed to write: %s", ros_path.c_str());
      ok = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Saved ROS calibration    → %s", ros_path.c_str());
    }
  }

  return ok;
}

}  // namespace usb_camera_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera_driver::CameraCalibrationDriver)
