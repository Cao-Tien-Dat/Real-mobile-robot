from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Đường dẫn tới các launch file
    my_bot_dir = get_package_share_directory('my_bot')
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    slam_dir = get_package_share_directory('slam_gmapping')
    nav2_dir = get_package_share_directory('nav2_bringup')
    teleop_dir = get_package_share_directory('teleop_twist_joy')

    return LaunchDescription([
        # 1. Khởi động robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(my_bot_dir, 'launch', 'launch_robot.launch.py'))
        ),

        # 2.Delay 10s rồi khởi động LiDAR
        TimerAction(
            period=10.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sllidar_dir, 'launch', 'sllidar_c1_launch.py'))
            )]
        ),
        # 4.  15s rồi khởi động Nav2
        TimerAction(
            period=15.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(my_bot_dir, 'launch', 'cartographer.launch.py'))
            )]
        ),

        # 4.  15s rồi khởi động Nav2
        TimerAction(
            period=20.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py'))
            )]
        ),

        # 5.  8s rồi khởi động Teleop
        TimerAction(
            period=30.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(teleop_dir, 'launch', 'teleop-launch.py'))
            )]
        ),
    ])
