import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'))
    
    rviz_param_dir = launch.substitutions.LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'rviz',
            'mapping.rviz'))

    mapping = launch_ros.actions.Node(
        package='frontend_slam',
        executable='frontend_slam_node',
        parameters=[main_param_dir],
        #remappings=[('/input_cloud','/velodyne_points')],
        #remappings=[('/input_cloud','/laser_controller/out')],
        remappings=[('/input_cloud','/points_raw')],
        output='screen'
        )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        #arguments=['0','0','0','0','0','0','1','base_link','velodyne']
        arguments=['0','0','0','0','0','0','1','base_link','laser_frame']
        )


    graphbasedslam = launch_ros.actions.Node(
        package='backend_slam',
        executable='backend_slam_node',
        parameters=[main_param_dir],
        output='screen'
        )
    
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param_dir]
        )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
        mapping,
        #tf,
        graphbasedslam,
        rviz,
            ])
