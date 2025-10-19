#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace, SetRemap
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='a200_0000')
    setup_path = LaunchConfiguration('setup_path', default=os.path.join(os.path.expanduser('~'), 'clearpath/'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='a200_0000',
        description='Namespace for the robot'
    )
    
    declare_setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value=os.path.join(os.path.expanduser('~'), 'clearpath/'),
        description='Path to the setup folder'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('clearpath_gz'),
                'launch',
                'simulation.launch.py'
            ])
        ])
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('clearpath_viz'),
                'launch',
                'view_navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('clearpath_nav2_demos'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'setup_path': setup_path,
            'use_sim_time': use_sim_time
        }.items()
    )


    nav2_launch = GroupAction([
        PushRosNamespace(namespace),  
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('inspector'),
                    'launch',
                    'custom_nav2_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution([
                    FindPackageShare('inspector'),
                    'config',
                    'nav2_params.yaml'
                ]),
                'namespace': namespace
            }.items()
        )
    ])

    # AZ Velocity Smoother Node  
    velocity_smoother_node = Node(
        package='inspector',
        executable='inspector',
        name='AZ_velocity_smoother',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Create timer actions with appropriate delays
    rviz_timer = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='Launching RViz...'),
            rviz_launch
        ]
    )

    slam_timer = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg='Launching SLAM...'),
            slam_launch
        ]
    )

    velocity_smoother_timer = TimerAction(
        period=40.0,  
        actions=[
            LogInfo(msg='Launching AZ Velocity Smoother...'),
            velocity_smoother_node
        ]
    )

    nav2_timer = TimerAction(
        period=45.0,
        actions=[
            LogInfo(msg='Launching Nav2 with remapped velocity_smoother...'),
            nav2_launch
        ]
    )

    return LaunchDescription([
        declare_namespace_arg,
        declare_setup_path_arg,
        declare_use_sim_time_arg,
        simulation_launch,
        rviz_timer,
        slam_timer,
        velocity_smoother_timer,
        nav2_timer,
    ]) 