#!/usr/bin/env python3
"""
Complete launch file for stereo cameras with your ORB-SLAM3 ROS2 package
This integrates your Logitech C920 stereo setup with your existing orbslam3_ros package
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Arguments
    declare_args = [
        DeclareLaunchArgument('baseline', default_value='0.1095'),
        DeclareLaunchArgument('left_device', 
            default_value='/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_1AC0956F-video-index0'),
        DeclareLaunchArgument('right_device', 
            default_value='/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_61EAC31F-video-index0'),
        DeclareLaunchArgument('voc_file', 
            default_value='/home/lukchin/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
        DeclareLaunchArgument('settings_file', 
            default_value='/home/lukchin/skuba_ws/src/orbslam3_ros2/config/stereo/C920.yaml'),
        DeclareLaunchArgument('enable_pangolin', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
    ]
    
    # Left camera node
    left_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='left_camera',
        namespace='stereo',
        parameters=[{
            'video_device': LaunchConfiguration('left_device'),
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'camera_frame_id': 'camera_left',
            'pixel_format': 'yuyv2rgb',
            'io_method': 'mmap',
            'camera_name': 'left_camera',
            'camera_info_url': ''  # Empty to avoid calibration file errors
        }],
        remappings=[
            ('image_raw', '/left/image_raw'),
            ('camera_info', '/left/camera_info')
        ],
        output='screen'
    )
    
    # Right camera node
    right_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='right_camera',
        namespace='stereo',
        parameters=[{
            'video_device': LaunchConfiguration('right_device'),
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'camera_frame_id': 'camera_right',
            'pixel_format': 'yuyv2rgb',
            'io_method': 'mmap',
            'camera_name': 'right_camera',
            'camera_info_url': ''  # Empty to avoid calibration file errors
        }],
        remappings=[
            ('image_raw', '/right/image_raw'),
            ('camera_info', '/right/camera_info')
        ],
        output='screen'
    )
    
    # ORB-SLAM3 Stereo Node (using your existing package)
    # Delayed start to ensure cameras are ready
    orbslam3_stereo = TimerAction(
        period=3.0,  # Wait 3 seconds for cameras to start
        actions=[
            Node(
                package='orbslam3_ros',  # Your package name
                executable='ros_stereo',  # Your stereo executable
                name='orb_slam3_stereo',
                output='screen',
                parameters=[{
                    'voc_file': LaunchConfiguration('voc_file'),
                    'settings_file': LaunchConfiguration('settings_file'),
                    'world_frame_id': 'world',
                    'cam_frame_id': 'camera',
                    'enable_pangolin': LaunchConfiguration('enable_pangolin')
                }],
                remappings=[
                    # IMPORTANT: Map your camera topics to what ORB-SLAM3 expects
                    ('/left/image_raw', '/left/image_raw'),
                    ('/right/image_raw', '/right/image_raw'),
                ]
            )
        ]
    )
    
    # RViz for visualization
    rviz_config = '/home/lukchin/skuba_ws/src/orbslam3_ros2/config/ntuviral_no_imu.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=LaunchConfiguration('launch_rviz')
    )
    
    # Optional: Status monitor node to check if everything is working
    # status_monitor = Node(
    #     package='orbslam3_ros',
    #     executable='status_monitor.py',  # You would need to create this
    #     name='status_monitor',
    #     output='screen',
    #     condition=LaunchConfiguration('monitor', default='false')
    # )
    
    return LaunchDescription([
        *declare_args,
        left_camera,
        right_camera,
        orbslam3_stereo,
        # rviz_node,  # Uncomment if you want RViz
    ])