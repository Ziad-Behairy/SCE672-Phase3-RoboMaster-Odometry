"""Tag detector + Phase 3 EKF node. Start the robomaster driver first, in its own terminal.

    ros2 launch rm_odom_ekf phase3.launch.py trial:=1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('rm_odom_ekf'), 'config', 'robot.yaml')
    trial = ParameterValue(LaunchConfiguration('trial'), value_type=int)
    return LaunchDescription([
        DeclareLaunchArgument('trial', description='trial number 1..10'),
        DeclareLaunchArgument('results_dir', default_value='~/SCE672-Phase3-RoboMaster-Odometry/results/trials'),
        DeclareLaunchArgument('save_frames_dir', default_value='',
                              description='if set, the detector saves every annotated camera frame here (video)'),
        Node(package='rm_odom_ekf', executable='tag_detector', output='screen',
             parameters=[cfg, {'save_frames_dir': LaunchConfiguration('save_frames_dir')}]),
        Node(package='rm_odom_ekf', executable='phase3_ekf', output='screen', emulate_tty=True,
             parameters=[cfg, {'trial': trial, 'results_dir': LaunchConfiguration('results_dir')}]),
    ])
