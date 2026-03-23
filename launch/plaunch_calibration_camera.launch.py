"""
calibration_camera.launch.py

Launch the one-shot USB camera calibration node.

The node opens the camera hardware directly (no topic subscription needed),
collects checkerboard detections, runs cv::calibrateCamera, and saves two files:
  <serial>_calib_cv.yaml   — OpenCV FileStorage format
  <serial>_calib_ros.yaml  — ROS camera_calibration_parsers format

Required argument:
  pid_vid               USB VID:PID of the camera

Optional arguments:
  serial_number         Required when >1 camera with identical VID:PID
  calibration_output_dir  Directory to save calibration files [default: "."]
  checkerboard_rows     Inner corner rows  [default: 6]
  checkerboard_cols     Inner corner cols  [default: 9]
  square_size           Physical square size in metres  [default: 0.025]
  num_frames            Frames to collect before calibrating  [default: 20]
  image_width           Capture width  [default: 1280]
  image_height          Capture height [default: 720]
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
            'calibration_output_dir',
            default_value='.',
            description='Directory where calibration YAML files will be saved'),

        DeclareLaunchArgument(
            'checkerboard_rows',
            default_value='6',
            description='Number of inner corner rows on the calibration checkerboard'),

        DeclareLaunchArgument(
            'checkerboard_cols',
            default_value='9',
            description='Number of inner corner columns on the calibration checkerboard'),

        DeclareLaunchArgument(
            'square_size',
            default_value='0.025',
            description='Physical size of one checkerboard square in metres'),

        DeclareLaunchArgument(
            'num_frames',
            default_value='20',
            description='Number of valid checkerboard frames to collect before calibrating'),

        DeclareLaunchArgument(
            'image_width',
            default_value='1600',
            description='Capture frame width in pixels'),

        DeclareLaunchArgument(
            'image_height',
            default_value='1200',
            description='Capture frame height in pixels'),

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
            executable='usb_camera_calibration_node',
            name='usb_camera_calibration_driver',
            parameters=[{
                'pid_vid':                LaunchConfiguration('pid_vid'),
                'serial_number':          LaunchConfiguration('serial_number'),
                'calibration_output_dir': LaunchConfiguration('calibration_output_dir'),
                'checkerboard_rows':      LaunchConfiguration('checkerboard_rows'),
                'checkerboard_cols':      LaunchConfiguration('checkerboard_cols'),
                'square_size':            LaunchConfiguration('square_size'),
                'num_frames':             LaunchConfiguration('num_frames'),
                'image_width':            LaunchConfiguration('image_width'),
                'image_height':           LaunchConfiguration('image_height'),
                'capture_backend':        LaunchConfiguration('capture_backend'),
                'pixel_format':           LaunchConfiguration('pixel_format'),
            }],
            output='screen',
        ),
    ])
