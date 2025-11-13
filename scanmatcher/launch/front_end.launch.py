import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node as StaticTransformNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the config file path
    package_dir = get_package_share_directory('scanmatcher')
    config_file = os.path.join(package_dir, 'config', 'small_gicp_high_accuracy.yaml')

    # Static transform publisher for lidar to base_link (identity transform)
    # Adjust the parent frame if your lidar publishes with a different frame_id
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'os1_lidar'],
        output='screen'
    )

    # scanmatcher node with high speed configuration
    scanmatcher_node = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        name='scanmatcher',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('input_cloud', 'ouster/points'), #a200_1057/cloud3/map #ouster/points #points_raw #os1_cloud_node/points
            ('current_pose', 'current_pose'),
            ('map', 'map'),
            ('path', 'path'),
            # ('tf_static', 'a200_1057/tf_static'),
            # ('tf', 'a200_1057/tf'),
        ]
    )

    return LaunchDescription([
        static_tf_node,
        scanmatcher_node,
    ])