import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    graphbasedslam_param_dir = launch.substitutions.LaunchConfiguration(
        'graphbasedslam_param_dir',
        default=os.path.join(
            get_package_share_directory('graph_based_slam'),
            'param',
            'graphbasedslam.yaml'))

    graphbasedslam_copy = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[graphbasedslam_param_dir],
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
            'graphbasedslam_param_dir',
            default_value=graphbasedslam_param_dir,
            description='Full path to graphbasedslam parameter file to load'),
        graphbasedslam_copy,
            ])