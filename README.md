# usb_camera_driver

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue?logo=ros)
![Platform](https://img.shields.io/badge/platform-Linux-lightgrey?logo=linux)
![License](https://img.shields.io/badge/license-Apache%202.0-green)
![Build](https://img.shields.io/badge/build-colcon-informational)

A ROS 2 USB camera driver built on OpenCV's V4L2 backend. Cameras are identified by **USB VID:PID** (and an optional USB serial number), so no `/dev/videoN` hardcoding is needed — devices are found reliably regardless of plug order.

USB device discovery delegates entirely to [**serial-ros2**](https://github.com/robodave94/serial-ros2.git), a forked serial/USB utility library that provides robust `sysfs`-based video device enumeration and USB serial number lookup.

---

## Features

- **VID:PID autodetect** — locate any V4L2 camera by vendor/product ID; no fixed device paths
- **USB serial disambiguation** — when multiple identical cameras are connected, select by USB serial number
- **image\_transport integration** — publishes `image_raw` + `camera_info`; compressed/theora/etc. transports enabled automatically via plugins
- **Calibration loading** — loads both ROS (`camera_calibration_parsers`) and OpenCV FileStorage YAML calibration files, with format auto-detection and mismatch warnings
- **One-shot calibration node** — built-in checkerboard calibration node that saves both ROS and OpenCV YAML outputs named by the camera's USB serial number
- **Configurable capture backend** — `v4l2`, `gstreamer`, `ffmpeg`, or `any` (auto)
- **FOURCC pixel format override** — e.g. `MJPG`, `YUYV`
- **Clean failure messages** — `FATAL` log on any detection failure with actionable guidance (lists found devices/serials)

---

## Dependencies

| Dependency | Type | Notes |
|---|---|---|
| [serial-ros2](https://github.com/robodave94/serial-ros2.git) | ROS 2 package | **Required.** Forked serial/USB utility library — provides `findVideoDevicesByPIDVID`, `getUSBSerialForVideoDevice` |
| `rclcpp`, `rclcpp_components` | ROS 2 | Core node/component infrastructure |
| `sensor_msgs`, `std_msgs` | ROS 2 | Image and camera info messages |
| `image_transport` | ROS 2 | Multi-transport image publishing |
| `camera_calibration_parsers` | ROS 2 | ROS-format YAML calibration loading |
| `camera_info_manager` | ROS 2 | Camera info management |
| `OpenCV 4` | System | Video capture and calibration |
| `image_transport_plugins` | ROS 2 (exec) | Compressed/theora transport plugins |

### Clone serial-ros2

```bash
git clone https://github.com/robodave94/serial-ros2.git src/serial-ros2
```

---

## Build

```bash
# From your ROS 2 workspace root
colcon build --packages-select serial usb_camera_driver
source install/setup.bash
```

---

## Nodes

### `usb_camera_run_node` — main streaming driver

Finds the camera by VID:PID, loads calibration, and streams via `image_transport`.

**Launch (recommended):**
```bash
ros2 launch usb_camera_driver plaunch_run_camera.launch.py pid_vid:="046d:085c"
```

**Direct node:**
```bash
ros2 run usb_camera_driver usb_camera_run_node --ros-args -p pid_vid:="046d:085c"
```

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `pid_vid` | *(required)* | USB VID:PID string, e.g. `"046d:085c"`. Accepts uppercase, `0x`-prefixed, or plain lowercase hex. |
| `serial_number` | `""` | USB serial number string. Required only when multiple cameras share the same VID:PID. |
| `camera_name` | `"camera"` | ROS topic namespace and `frame_id` |
| `calibration_file` | `""` | Absolute path or `file://` URI to a calibration YAML. Leave empty to run uncalibrated. |
| `calibration_file_type` | `"ros"` | `"ros"` (camera\_calibration\_parsers format) or `"cv"` (OpenCV FileStorage format) |
| `image_width` | `1600` | Capture frame width in pixels |
| `image_height` | `1200` | Capture frame height in pixels |
| `fps` | `50.0` | Target capture framerate |
| `capture_backend` | `"any"` | OpenCV backend: `"any"`, `"v4l2"`, `"gstreamer"`, `"ffmpeg"` |
| `pixel_format` | `""` | FOURCC override, e.g. `"MJPG"`, `"YUYV"`. Empty = camera default. |

**Published topics** (under `/<camera_name>/`):

| Topic | Type |
|---|---|
| `image_raw` | `sensor_msgs/Image` |
| `image_raw/compressed` | `sensor_msgs/CompressedImage` *(via plugin)* |
| `camera_info` | `sensor_msgs/CameraInfo` |

---

### `usb_camera_calibration_node` — one-shot calibration

Opens the camera directly, collects checkerboard detections, runs `cv::calibrateCamera`, and saves two YAML files named after the camera's USB serial number:

- `<serial>_calib_ros.yaml` — ROS `camera_calibration_parsers` format (use with `calibration_file_type: ros`)
- `<serial>_calib_cv.yaml` — OpenCV FileStorage format (use with `calibration_file_type: cv`)

```bash
ros2 launch usb_camera_driver plaunch_calibration_camera.launch.py \
  pid_vid:="046d:085c" \
  calibration_output_dir:="/path/to/save" \
  checkerboard_rows:=6 \
  checkerboard_cols:=9 \
  square_size:=0.025 \
  num_frames:=20
```

**Additional parameters:**

| Parameter | Default | Description |
|---|---|---|
| `calibration_output_dir` | `"."` | Directory to write output YAML files |
| `checkerboard_rows` | `6` | Inner corner rows |
| `checkerboard_cols` | `9` | Inner corner columns |
| `square_size` | `0.025` | Physical square size in metres |
| `num_frames` | `20` | Frames to collect before computing calibration |

---

### `usb_camera_driver_node` — legacy driver

Original simple driver (OpenCV index-based, no VID:PID detection). Retained for compatibility.

```bash
ros2 run usb_camera_driver usb_camera_driver_node
```

---

## Device discovery & failure messages

The driver reports actionable `FATAL` errors on any detection failure:

| Situation | Message |
|---|---|
| Bad `pid_vid` format | `USB device enumeration failed: Invalid PID:VID format: <value>` |
| Device not plugged in / wrong VID:PID | `No video device found matching VID:PID = <value>` |
| Device found but serial mismatch | `One device found (/dev/videoX) but its serial number "…" does not match the requested serial "…"` |
| Multiple devices, no serial set | `N devices matched VID:PID=… but no serial_number was provided. Found devices: … Re-launch with -p serial_number:=<one of the above>` |
| Multiple devices, serial not matched | `Serial number "…" not found among the N devices matching VID:PID=…. Found serials: …` |

The node calls `rclcpp::shutdown()` immediately on any fatal detection error.

---

## Finding your camera's VID:PID

```bash
lsusb
# Bus 001 Device 004: ID 046d:085c Logitech, Inc. BRIO Ultra HD Webcam
```

The `ID` field is `VID:PID`. Pass it directly as the `pid_vid` parameter.

---

## Alternate packages

- [ros-drivers/usb\_cam](https://github.com/ros-drivers/usb_cam/tree/ros2) — feature-rich production driver
- [image\_pipeline](https://github.com/ros-perception/image_pipeline/tree/ros2) — full calibration toolchain

## References

- [serial-ros2 (robodave94 fork)](https://github.com/robodave94/serial-ros2.git)
- [image\_transport](http://wiki.ros.org/image_transport)
- [image\_transport\_plugins](https://github.com/ros-perception/image_transport_plugins/tree/ros2)
- [vision\_opencv](https://github.com/ros-perception/vision_opencv/tree/ros2)
