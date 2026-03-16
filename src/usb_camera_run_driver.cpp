/**
 * @file usb_camera_run_driver.cpp
 * @brief CameraRunDriver — detects USB camera by VID:PID+serial, validates calibration
 *        format, and publishes via image_transport (image_raw + all enabled transport plugins).
 */

#include "usb_camera_run_driver.hpp"
#include "usb_device_detector.hpp"

#include <fstream>
#include <sstream>
#include <chrono>
#include <cstring>

#include "camera_calibration_parsers/parse.hpp"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"

using namespace std::chrono_literals;

namespace usb_camera_driver
{

// ─── calibration format sniffer ──────────────────────────────────────────────

std::string CameraRunDriver::detectCalibrationFileFormat(const std::string & filepath)
{
  std::ifstream f(filepath);
  if (!f.is_open()) return "unknown";

  std::string line;
  int count = 0;
  while (std::getline(f, line) && count < 30) {
    ++count;
    // OpenCV FileStorage markers
    if (line.find("%YAML") != std::string::npos) return "cv";
    if (line.find("!!opencv-matrix") != std::string::npos) return "cv";
    // ROS camera_calibration_parsers markers
    if (line.find("image_width") != std::string::npos) return "ros";
    if (line.find("distortion_model") != std::string::npos) return "ros";
    if (line.find("camera_name") != std::string::npos) return "ros";
  }
  return "unknown";
}

// ─── constructor ─────────────────────────────────────────────────────────────

CameraRunDriver::CameraRunDriver(const rclcpp::NodeOptions & options)
: Node("usb_camera_run_driver", options)
{
  // ── declare parameters ──
  pid_vid_               = this->declare_parameter<std::string>("pid_vid", "");
  serial_number_         = this->declare_parameter<std::string>("serial_number", "");
  camera_name_           = this->declare_parameter<std::string>("camera_name", "camera");
  calibration_file_      = this->declare_parameter<std::string>("calibration_file", "");
  calibration_file_type_ = this->declare_parameter<std::string>("calibration_file_type", "ros");
  image_width_           = this->declare_parameter<int>("image_width", 1280);
  image_height_          = this->declare_parameter<int>("image_height", 720);
  fps_                   = this->declare_parameter<double>("fps", 10.0);
  capture_backend_       = this->declare_parameter<std::string>("capture_backend", "any");
  pixel_format_          = this->declare_parameter<std::string>("pixel_format", "");

  // ── validate pid_vid ──
  if (pid_vid_.empty()) {
    RCLCPP_FATAL(this->get_logger(),
      "Parameter 'pid_vid' is required but was not set. "
      "Provide it as e.g. --ros-args -p pid_vid:=\"046d:085c\"");
    rclcpp::shutdown();
    return;
  }

  // ── validate calibration_file_type ──
  if (calibration_file_type_ != "ros" && calibration_file_type_ != "cv") {
    RCLCPP_FATAL(this->get_logger(),
      "Parameter 'calibration_file_type' must be \"ros\" or \"cv\", got: \"%s\"",
      calibration_file_type_.c_str());
    rclcpp::shutdown();
    return;
  }

  // ── USB device detection ──
  if (!detectDevice()) {
    rclcpp::shutdown();
    return;
  }

  // ── calibration ──
  if (!loadCalibration()) {
    rclcpp::shutdown();
    return;
  }

  // ── open capture ──
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
      "Failed to open video device: %s (backend: %s)",
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
  RCLCPP_INFO(this->get_logger(), "Capture backend: %s", capture_backend_.c_str());

  // ── image_transport CameraPublisher ──
  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  last_frame_ = std::chrono::steady_clock::now();

  // 1 ms poll timer — rate throttled in callback by fps_
  timer_ = this->create_wall_timer(1ms, std::bind(&CameraRunDriver::imageCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "CameraRunDriver started — device: %s, resolution: %dx%d @ %.1f fps",
    resolved_device_.c_str(), image_width_, image_height_, fps_);
}

// ─── device detection ────────────────────────────────────────────────────────

bool CameraRunDriver::detectDevice()
{
  uint16_t vid = 0, pid = 0;
  try {
    UsbDeviceDetector::parseVidPid(pid_vid_, vid, pid);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to parse 'pid_vid': %s", e.what());
    return false;
  }

  std::vector<DeviceMatch> matches;
  try {
    matches = UsbDeviceDetector::findVideoDevices(vid, pid);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "USB device enumeration failed: %s", e.what());
    return false;
  }

  if (matches.empty()) {
    RCLCPP_FATAL(this->get_logger(),
      "No video device found matching VID:PID = %s", pid_vid_.c_str());
    return false;
  }

  if (matches.size() == 1) {
    if (!serial_number_.empty() && matches[0].serial_number != serial_number_) {
      RCLCPP_FATAL(this->get_logger(),
        "One device found (%s) but its serial number \"%s\" does not match "
        "the requested serial \"%s\"",
        matches[0].device_node.c_str(),
        matches[0].serial_number.c_str(),
        serial_number_.c_str());
      return false;
    }
    resolved_device_ = matches[0].device_node;
    RCLCPP_INFO(this->get_logger(), "Found device: %s  (serial: \"%s\")",
      resolved_device_.c_str(), matches[0].serial_number.c_str());
    return true;
  }

  // Multiple devices found
  if (serial_number_.empty()) {
    std::ostringstream oss;
    oss << matches.size() << " devices matched VID:PID=" << pid_vid_
        << " but no serial_number was provided. Cannot disambiguate.\n"
        << "Found devices:\n";
    for (auto & m : matches) {
      oss << "  " << m.device_node << "  serial=\"" << m.serial_number << "\"\n";
    }
    oss << "Re-launch with  -p serial_number:=<one of the above>  to select a device.";
    RCLCPP_FATAL(this->get_logger(), "%s", oss.str().c_str());
    return false;
  }

  // Filter by serial
  std::vector<DeviceMatch> filtered;
  for (auto & m : matches) {
    if (m.serial_number == serial_number_) {
      filtered.push_back(m);
    }
  }
  if (filtered.empty()) {
    std::ostringstream oss;
    oss << "Serial number \"" << serial_number_
        << "\" not found among the " << matches.size()
        << " devices matching VID:PID=" << pid_vid_ << ".\nFound serials:";
    for (auto & m : matches) {
      oss << "\n  \"" << m.serial_number << "\" on " << m.device_node;
    }
    RCLCPP_FATAL(this->get_logger(), "%s", oss.str().c_str());
    return false;
  }

  resolved_device_ = filtered[0].device_node;
  RCLCPP_INFO(this->get_logger(), "Found device: %s  (serial: \"%s\")",
    resolved_device_.c_str(), filtered[0].serial_number.c_str());
  return true;
}

// ─── calibration loading ─────────────────────────────────────────────────────

bool CameraRunDriver::loadCalibration()
{
  if (calibration_file_.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "No calibration file provided (parameter 'calibration_file' is empty). "
      "Running without camera calibration — camera_info will contain zeros.");
    camera_info_msg_ = sensor_msgs::msg::CameraInfo{};
    calibration_loaded_ = false;
    return true;  // Not a fatal error
  }

  // Strip optional file:// prefix for local filesystem access
  std::string fpath = calibration_file_;
  if (fpath.rfind("file://", 0) == 0) {
    fpath = fpath.substr(7);
  }

  // Auto-detect format
  std::string detected_fmt = detectCalibrationFileFormat(fpath);

  // Validate against declared type
  if (detected_fmt != "unknown" && detected_fmt != calibration_file_type_) {
    RCLCPP_FATAL(this->get_logger(),
      "Calibration file format mismatch!\n"
      "  File:               %s\n"
      "  Parameter declared: calibration_file_type = \"%s\"\n"
      "  File auto-detected: \"%s\"\n"
      "\n"
      "Fix: either correct 'calibration_file_type' to \"%s\", "
      "or provide a %s-format calibration file.\n"
      "  ROS format: produced by camera_calibration ros package or "
      "the usb_camera_calibration_node (saves *_calib_ros.yaml).\n"
      "  CV  format: produced by OpenCV FileStorage or "
      "the usb_camera_calibration_node (saves *_calib_cv.yaml).",
      fpath.c_str(),
      calibration_file_type_.c_str(),
      detected_fmt.c_str(),
      detected_fmt.c_str(),
      calibration_file_type_.c_str());
    return false;
  }

  if (calibration_file_type_ == "ros") {
    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, camera_name_);
    if (!cinfo_manager_->loadCameraInfo(calibration_file_)) {
      RCLCPP_FATAL(this->get_logger(),
        "camera_info_manager failed to load ROS calibration from: %s",
        calibration_file_.c_str());
      return false;
    }
    camera_info_msg_ = cinfo_manager_->getCameraInfo();
    RCLCPP_INFO(this->get_logger(),
      "Loaded ROS calibration from: %s", calibration_file_.c_str());

  } else {
    // calibration_file_type_ == "cv"
    cv::FileStorage fs(fpath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_FATAL(this->get_logger(),
        "Failed to open OpenCV calibration file: %s", fpath.c_str());
      return false;
    }

    cv::Mat camera_matrix, dist_coeffs;
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"]   >> dist_coeffs;
    fs.release();

    if (camera_matrix.empty()) {
      RCLCPP_FATAL(this->get_logger(),
        "OpenCV calibration file did not contain 'camera_matrix': %s", fpath.c_str());
      return false;
    }

    // Populate CameraInfo from OpenCV matrices
    sensor_msgs::msg::CameraInfo ci;
    ci.width  = static_cast<uint32_t>(image_width_);
    ci.height = static_cast<uint32_t>(image_height_);
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

    camera_info_msg_ = ci;
    RCLCPP_INFO(this->get_logger(),
      "Loaded OpenCV calibration from: %s", fpath.c_str());
  }

  calibration_loaded_ = true;
  return true;
}

// ─── image callback ──────────────────────────────────────────────────────────

std::shared_ptr<sensor_msgs::msg::Image>
CameraRunDriver::convertFrameToMessage(cv::Mat & frame)
{
  if (frame.rows != image_height_ || frame.cols != image_width_) {
    cv::resize(frame, frame, cv::Size(image_width_, image_height_));
  }

  auto ros_image = std::make_shared<sensor_msgs::msg::Image>();
  ros_image->height   = static_cast<uint32_t>(frame.rows);
  ros_image->width    = static_cast<uint32_t>(frame.cols);
  ros_image->encoding = "bgr8";
  ros_image->is_bigendian = false;
  ros_image->step     = static_cast<uint32_t>(frame.cols * frame.elemSize());

  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);

  if (frame.isContinuous()) {
    std::memcpy(ros_image->data.data(), frame.data, size);
  } else {
    uint8_t * dst = ros_image->data.data();
    uint8_t * src = frame.data;
    for (int i = 0; i < frame.rows; ++i,
         dst += ros_image->step,
         src += frame.step[0]) {
      std::memcpy(dst, src, ros_image->step);
    }
  }
  return ros_image;
}

void CameraRunDriver::imageCallback()
{
  cap_ >> frame_;
  if (frame_.empty()) return;

  // Rate limiting
  auto now = std::chrono::steady_clock::now();
  double elapsed_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_).count();
  if (elapsed_ms < (1000.0 / fps_)) return;
  last_frame_ = now;

  image_msg_ = convertFrameToMessage(frame_);

  auto ci = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_msg_);
  rclcpp::Time ts = this->get_clock()->now();

  image_msg_->header.stamp    = ts;
  image_msg_->header.frame_id = camera_name_;
  ci->header.stamp    = ts;
  ci->header.frame_id = camera_name_;

  camera_pub_.publish(image_msg_, ci);
}

}  // namespace usb_camera_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera_driver::CameraRunDriver)
