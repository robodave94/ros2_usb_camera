/**
 * @file usb_device_detector.cpp
 * @brief libudev-based USB video device lookup.
 */

#include "usb_device_detector.hpp"

#include <libudev.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace usb_camera_driver
{

// ─── helpers ─────────────────────────────────────────────────────────────────

static std::string hex_to_lower(const std::string & s)
{
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

// Strip leading "0x" or "0X" if present, then parse as hex.
static uint16_t parse_hex16(const std::string & tok, const std::string & context)
{
  std::string s = tok;
  if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    s = s.substr(2);
  }
  if (s.empty()) {
    throw std::invalid_argument("Empty hex token in: " + context);
  }
  try {
    unsigned long val = std::stoul(s, nullptr, 16);
    if (val > 0xFFFF) {
      throw std::invalid_argument("Value out of uint16_t range in: " + context);
    }
    return static_cast<uint16_t>(val);
  } catch (const std::exception &) {
    throw std::invalid_argument("Cannot parse hex token '" + tok + "' in: " + context);
  }
}

// ─── public API ──────────────────────────────────────────────────────────────

void UsbDeviceDetector::parseVidPid(const std::string & vidpid_str,
                                     uint16_t & out_vid,
                                     uint16_t & out_pid)
{
  auto colon = vidpid_str.find(':');
  if (colon == std::string::npos) {
    throw std::invalid_argument(
      "Invalid VID:PID format '" + vidpid_str + "'. Expected e.g. '046d:085c'");
  }
  std::string vid_tok = vidpid_str.substr(0, colon);
  std::string pid_tok = vidpid_str.substr(colon + 1);
  out_vid = parse_hex16(vid_tok, vidpid_str);
  out_pid = parse_hex16(pid_tok, vidpid_str);
}

std::vector<DeviceMatch> UsbDeviceDetector::findVideoDevices(uint16_t vendor_id,
                                                               uint16_t product_id)
{
  // Format the expected attribute strings (4-digit lowercase hex, no 0x prefix)
  char vid_str[5], pid_str[5];
  std::snprintf(vid_str, sizeof(vid_str), "%04x", vendor_id);
  std::snprintf(pid_str, sizeof(pid_str), "%04x", product_id);

  std::vector<DeviceMatch> results;

  struct udev * udev = udev_new();
  if (!udev) {
    throw std::runtime_error("Failed to create udev context");
  }

  struct udev_enumerate * enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "video4linux");
  udev_enumerate_scan_devices(enumerate);

  struct udev_list_entry * devices = udev_enumerate_get_list_entry(enumerate);
  struct udev_list_entry * entry;

  udev_list_entry_foreach(entry, devices) {
    const char * path = udev_list_entry_get_name(entry);
    struct udev_device * dev = udev_device_new_from_syspath(udev, path);
    if (!dev) continue;

    // Get the /dev node (e.g. /dev/video0)
    const char * devnode = udev_device_get_devnode(dev);
    if (!devnode) {
      udev_device_unref(dev);
      continue;
    }

    // Walk up the parent chain to find a USB device with matching id
    struct udev_device * usb_dev = udev_device_get_parent_with_subsystem_devtype(
      dev, "usb", "usb_device");

    if (usb_dev) {
      const char * id_vendor  = udev_device_get_sysattr_value(usb_dev, "idVendor");
      const char * id_product = udev_device_get_sysattr_value(usb_dev, "idProduct");

      if (id_vendor && id_product) {
        std::string found_vid = hex_to_lower(std::string(id_vendor));
        std::string found_pid = hex_to_lower(std::string(id_product));

        if (found_vid == std::string(vid_str) && found_pid == std::string(pid_str)) {
          DeviceMatch m;
          m.device_node = std::string(devnode);
          m.vid = found_vid;
          m.pid = found_pid;

          const char * serial = udev_device_get_sysattr_value(usb_dev, "serial");
          if (serial) {
            m.serial_number = std::string(serial);
          }
          results.push_back(m);
        }
      }
      // NOTE: usb_dev is owned by dev — do NOT udev_device_unref it.
    }

    udev_device_unref(dev);
  }

  udev_enumerate_unref(enumerate);
  udev_unref(udev);

  // Sort so that /dev/video0 < /dev/video1 etc. for determinism
  std::sort(results.begin(), results.end(),
    [](const DeviceMatch & a, const DeviceMatch & b) {
      return a.device_node < b.device_node;
    });

  // Remove entries with the same non-empty serial (same physical camera on multiple nodes)
  results.erase(
    std::unique(results.begin(), results.end(),
      [](const DeviceMatch & a, const DeviceMatch & b) {
        return !a.serial_number.empty() && a.serial_number == b.serial_number;
      }),
    results.end());

  return results;
}

}  // namespace usb_camera_driver
