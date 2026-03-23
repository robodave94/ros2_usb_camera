"""
run_camera.launch.py

Launch the USB autodetect camera driver node.

Required argument:
  pid_vid               VID:PID of the camera, e.g. "0c45:6366"

Optional arguments:
  serial_number         USB serial number (required when >1 device with same VID:PID)
  camera_name           Camera name / frame_id / topic namespace  [default: "camera"]
  calibration_file      Path to calibration YAML (file:// or absolute path)  [default: ""]
  calibration_file_type "ros" or "cv"  [default: "ros"]
  image_width           Capture width  [default: 1280]
  image_height          Capture height [default: 720]
  fps                   Target framerate [default: 10.0]

Topics published (under /<camera_name>/):
  image_raw                         sensor_msgs/Image
  image_raw/compressed              (via image_transport compressed plugin)
  image_raw/camera_info             sensor_msgs/CameraInfo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pid_vid',
            description='USB VID:PID of the camera, e.g. "0c45:6366"'),

        DeclareLaunchArgument(
            'serial_number',
            default_value='',
            description='USB serial number — required when multiple cameras share the same VID:PID'),

        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Camera name used as frame_id and topic namespace'),

        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description='Path to calibration YAML (file:// prefix or absolute path). '
                        'Leave empty to run uncalibrated.'),

        DeclareLaunchArgument(
            'calibration_file_type',
            default_value='ros',
            description='"ros" (camera_calibration_parsers format, *_calib_ros.yaml) or '
                        '"cv" (OpenCV FileStorage format, *_calib_cv.yaml)'),

        DeclareLaunchArgument(
            'image_width',
            default_value='1600',
            description='Capture frame width in pixels'),

        DeclareLaunchArgument(
            'image_height',
            default_value='1200',
            description='Capture frame height in pixels'),

        DeclareLaunchArgument(
            'fps',
            default_value='50.0',
            description='Target capture framerate'),

        DeclareLaunchArgument(
            'capture_backend',
            default_value='any',
            description='OpenCV capture backend: "any" (auto), "v4l2", "gstreamer", "ffmpeg"'),

        DeclareLaunchArgument(
            'pixel_format',
            default_value='',
            description='FOURCC pixel format override, e.g. "MJPG" or "YUYV". '
                        'Leave empty to use the camera default.'),

        Node(
            package='usb_camera_driver',
            executable='usb_camera_run_node',
            name='usb_camera_run_driver',
            namespace=LaunchConfiguration('camera_name'),
            parameters=[{
                'pid_vid':               LaunchConfiguration('pid_vid'),
                'serial_number':         LaunchConfiguration('serial_number'),
                'camera_name':           LaunchConfiguration('camera_name'),
                'calibration_file':      LaunchConfiguration('calibration_file'),
                'calibration_file_type': LaunchConfiguration('calibration_file_type'),
                'image_width':           LaunchConfiguration('image_width'),
                'image_height':          LaunchConfiguration('image_height'),
                'fps':                   LaunchConfiguration('fps'),
                'capture_backend':       LaunchConfiguration('capture_backend'),
                'pixel_format':          LaunchConfiguration('pixel_format'),
            }],
            output='screen',
        ),
    ])
