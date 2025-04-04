from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import os

def launch_setup(context, *args, **kwargs):
    
    use_sim_time=LaunchConfiguration('use_sim_time')
    # Directories
    pkg_tortoisebot_gazebo = get_package_share_directory(
        'tortoisebot_bringup')
    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')
    pkg_rtabmap = get_package_share_directory(
        'tortoisebot_gazebo')

    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('tortoisebot_gazebo'), 'config', 'tortoise_nav2_param.yaml']
    )

    # Paths
    gazebo_launch = PathJoinSubstitution(
        [pkg_tortoisebot_gazebo, 'launch', 'bringup.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])
    rtabmap_launch = PathJoinSubstitution(
        [pkg_rtabmap, 'launch', 'tortoise_rgbd.launch.py'])

    # Includes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('use_sim_time', use_sim_time)
        ]
    )
    
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('params_file', nav2_params_file)
        ]
    )
    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch])
    )
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', use_sim_time)
        ]
    )
    
    return [
        # Nodes to launch
        nav2,
        rviz,
        rtabmap,
        gazebo
    ]

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='False',
            description='Launch in localization mode.'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Launch in simulation mode.'),

        OpaqueFunction(function=launch_setup)
    ])
