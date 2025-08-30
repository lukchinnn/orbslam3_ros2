from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get paths (adjust these to your setup)
    # orb_slam3_path = os.path.expanduser('~/ORB_SLAM3')
    # config_path = os.path.expanduser('~/ros2_ws/src/stereo_c920_driver/config')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('voc_file', 
            default_value="/home/lukchin/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
        DeclareLaunchArgument('settings_file', 
            default_value='/home/lukchin/skuba_ws/src/orbslam3_ros2/config/stereo/C920.yaml'),
        DeclareLaunchArgument('enable_pangolin', default_value='true'),
        
        # Launch stereo cameras first
        # (You can include your camera launch here or launch separately)
        
        # ORB-SLAM3 Stereo Node
        Node(
            package='orbslam3_ros',  # Your package name
            executable='ros_stereo',  # Your executable name
            name='orb_slam3_stereo',
            output='screen',
            parameters=[{
                'voc_file': LaunchConfiguration('voc_file'),
                'settings_file': LaunchConfiguration('settings_file'),
                'world_frame_id': 'world',
                'cam_frame_id': 'camera',
                'enable_pangolin': LaunchConfiguration('enable_pangolin')
            }]
        )
    ])