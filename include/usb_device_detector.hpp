/**
 * @file usb_device_detector.hpp
 * @brief Utility to locate /dev/videoN nodes by USB PID:VID and optional serial number.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace usb_camera_driver
{

struct DeviceMatch
{
  std::string device_node;   ///< e.g. "/dev/video0"
  std::string serial_number; ///< USB iSerial string (empty if device has none)
  std::string vid;           ///< lowercase hex, e.g. "046d"
  std::string pid;           ///< lowercase hex, e.g. "085c"
};

class UsbDeviceDetector
{
public:
  /**
   * @brief Parse a "VID:PID" string into numeric IDs.
   * Accepted formats: "046d:085c", "0x046d:0x085c", "046D:085C"
   * @throws std::invalid_argument on malformed input
   */
  static void parseVidPid(const std::string & vidpid_str,
                           uint16_t & out_vid,
                           uint16_t & out_pid);

  /**
   * @brief Enumerate all V4L2 video devices whose USB parent matches vid:pid.
   * @return One DeviceMatch per matching /dev/videoN
   */
  static std::vector<DeviceMatch> findVideoDevices(uint16_t vendor_id,
                                                    uint16_t product_id);
};

}  // namespace usb_camera_driver
