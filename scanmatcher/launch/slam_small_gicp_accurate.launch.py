import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the config file path
    package_dir = get_package_share_directory('scanmatcher')
    config_file = os.path.join(package_dir, 'config', 'small_gicp_high_accuracy.yaml')

    # scanmatcher node with high accuracy configuration
    scanmatcher_node = Node(
        package='scanmatcher',
        executable='scanmatcher_component_copy',
        name='scanmatcher_copy',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('input_cloud', 'ouster/points'),
            ('current_pose', 'current_pose'),
            ('map', 'map'),
            ('path', 'path'),
        ]
    )

    return LaunchDescription([
        scanmatcher_node,
    ])