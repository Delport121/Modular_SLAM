import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    backendslam_param_dir = launch.substitutions.LaunchConfiguration(
        'backendslam_param_dir',
        default=os.path.join(
            get_package_share_directory('backend_slam'),
            'param',
            'backendslam.yaml'))

    backendslam = launch_ros.actions.Node(
        package='backend_slam',
        executable='backend_slam_node',
        parameters=[backendslam_param_dir],
        output='screen',
        remappings=[
            ('submap', 'submap'), #a200_1057/submap
            # ('current_pose', 'current_pose'),
            # ('map', 'map'),
            # ('path', 'path'),
            # ('tf_static', 'a200_1057/tf_static'),
            # ('tf', 'a200_1057/tf'),
        ]
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'backendslam_param_dir',
            default_value=backendslam_param_dir,
            description='Full path to backendslam parameter file to load'),
        backendslam,
            ])