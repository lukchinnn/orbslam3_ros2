# orb_slam3_rgbd_realsense.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_orb_slam3 = get_package_share_directory('orbslam3_ros')
    pkg_realsense = get_package_share_directory('realsense2_camera')

    # Paths
    voc_file = "/home/lukchin/skuba_ws/src/orbslam3_ros2/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    settings_file = "/home/lukchin/skuba_ws/src/orbslam3_ros2/config/rgb-d/RealSense_D435i.yaml"  # Changed to RGB-D config
    rviz_config = "/home/lukchin/skuba_ws/src/orbslam3_ros2/config/ntuviral_no_imu.rviz"

    # Launch the RealSense camera driver
    # realsense_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
    #     ),
    #     launch_arguments={
    #         'enable_color': 'true',
    #         'enable_depth': 'true',      # Enable depth for RGB-D
    #         'align_depth': 'true',       # Critical: align depth to color frame
    #         'enable_sync': 'true',       # Synchronize RGB and depth
    #         'pointcloud.enable': 'false',
    #         'color_width': '640',        # Set resolution
    #         'color_height': '480',
    #         'depth_width': '640',
    #         'depth_height': '480',
    #         'color_fps': '30',           # Set frame rate
    #         'depth_fps': '30'
    #     }.items()
    # )

    # ORB-SLAM3 RGB-D node
    orbslam_node = Node(
        package='orbslam3_ros',
        executable='ros_rgbd',          # Changed to RGB-D executable
        name='orb_slam3_rgbd',          # Changed name
        output='screen',
        # No remappings needed - topics are correct by default
        parameters=[{
            'voc_file': voc_file,
            'settings_file': settings_file,
            'world_frame_id': 'map',
            'cam_frame_id': 'camera_link',  # RealSense optical frame
            'enable_pangolin': False
        }]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        # realsense_node,
        orbslam_node,
        rviz_node
    ])