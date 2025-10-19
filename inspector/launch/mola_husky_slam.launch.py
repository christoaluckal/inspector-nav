#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, GroupAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, SetParameter
# 
def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Include existing simulation launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('clearpath_gz'),
                'launch',
                'simulation.launch.py'
            ])
        ])
    )

    # Simple RViz2 node with basic Husky configuration
    group_view_model = GroupAction([
        PushRosNamespace(namespace='a200_0000'),
        Node(package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('inspector'),
                'rviz',
                'husky.rviz'
            ])],
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            remappings=[('/tf','tf'),('/tf_static','tf_static')],
            output='screen'

        )
    ])
    

    # MOLA LiDAR Odometry launch with our specific parameters
    mola_launch =IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mola_lidar_odometry'),
                'ros2-launchs',
                'ros2-lidar-odometry.launch.py'
            ])
        ]),
        launch_arguments={
            'lidar_topic_name': 'sensors/lidar3d_0/points',
            'use_namespace': 'True',
            'namespace': 'a200_0000',
            'imu_topic_name': 'sensors/imu_0/imu_data',
            'use_state_estimator': 'False',
            'use_rviz': 'False'
        }.items()
    )

    mola_group = GroupAction([
        SetParameter(name='use_sim_time', value=use_sim_time),
        mola_launch,
    ])

    # Create a custom octomap launch with proper TF remapping
    octomap_node = Node(
        package='octomap_server2',
        executable='octomap_server',
        name='octomap_server',
        output='screen',
        remappings=[
            ('cloud_in', '/a200_0000/lidar_odometry/localmap_points'),
            ('/tf','/a200_0000/tf'),
            ('/tf_static','/a200_0000/tf_static'),
        ],
        parameters=[{
            'base_frame_id': 'base_link',
            'frame_id': 'map',
            'use_sim_time': use_sim_time,
            'pointcloud_min_z': 0.05,
            'pointcloud_max_z': 1.0,
            'resolution': 0.1
        }]
    )

    # Topic relay to bridge /projected_map to /a200_0000/map
    map_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='map_relay',
        arguments=['/projected_map', '/a200_0000/map'],
        output='screen'
    )

    #Az velocity smoother
    velocity_smoother_node = Node(
        package='inspector',
        executable='inspector',
        name='AZ_velocity_smoother',
        namespace='a200_0000',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    nav2_launch = GroupAction([
        PushRosNamespace(namespace='a200_0000'),  
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
                'namespace': 'a200_0000'
            }.items()
        )
    ])

    # rviz timer
    rviz_timer = TimerAction(
        period=15.0,  
        actions=[
            LogInfo(msg='Launching rviz...'),
            group_view_model
        ]
    )
    # MOLA system needs time for simulation to fully start
    mola_timer = TimerAction(
        period=20.0,  
        actions=[
            LogInfo(msg='Launching MOLA SLAM System...'),
            mola_group
        ]
    )

    octomap_timer = TimerAction(
        period=25.0,  
        actions=[
            LogInfo(msg='Launching Octomap Server...'),
            octomap_node
        ]
    )

    velocity_smoother_timer = TimerAction(
        period=30.0,  
        actions=[
            LogInfo(msg='Launching AZ Velocity Smoother...'),
            velocity_smoother_node
        ]
    )

    nav2_timer = TimerAction(
        period=35.0,
        actions=[
            LogInfo(msg='Launching Nav2 with remapped velocity_smoother...'),
            nav2_launch
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        simulation_launch,
        map_relay_node,
        
        # Timed MOLA launch
        rviz_timer,
        mola_timer,
        octomap_timer,
        velocity_smoother_timer,
        nav2_timer
    ]) 