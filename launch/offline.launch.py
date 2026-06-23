import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')

    input_bag = LaunchConfiguration('input_bag')
    parameter_file = LaunchConfiguration('params_file')
    save_directory = LaunchConfiguration('save_directory')

    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'input_bag',
            default_value='',
            description='Path to input rosbag2 directory.'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'params.yaml'),
            description='Path to params.yaml.'
        ),

        DeclareLaunchArgument(
            'save_directory',
            default_value='./map',
            description='Directory to save offline map.'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'map', 'odom'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        Node(
            package='lio_sam',
            executable='lio_sam_run_offline',
            name='lio_sam_run_offline',
            parameters=[parameter_file],
            arguments=[
                '--input_bag', input_bag,
                '--params_file', parameter_file,
                '--save_directory', save_directory
            ],
            output='screen'
        )
    ])
