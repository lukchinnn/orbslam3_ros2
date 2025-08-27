# orb_slam3_mono_realsense.launch.py
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
    voc_file = "/home/lukchin/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    settings_file = "/home/lukchin/skuba_ws/src/orbslam3_ros2/config/monocular/RealSense_D435i.yaml"
    rviz_config = "/home/lukchin/skuba_ws/src/orbslam3_ros2/config/ntuviral_no_imu.rviz"

    # Launch the RealSense camera driver
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',   # we want pure monocular
            'pointcloud.enable': 'false'
        }.items()
    )

    # ORB-SLAM3 main node
    orbslam_node = Node(
        package='orbslam3_ros',
        executable='ros_mono',
        name='orb_slam3',
        output='screen',
        remappings=[
            ('/camera/image_raw', '/camera/camera/color/image_raw')
        ],
        parameters=[{
            'voc_file': voc_file,
            'settings_file': settings_file,
            'world_frame_id': 'world',
            'cam_frame_id': 'camera',
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
        realsense_node,
        orbslam_node,
        rviz_node
    ])
