#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch arguments for scan-to-model pipeline
    registration_method_arg = DeclareLaunchArgument(
        'registration_method',
        default_value='SCAN_TO_MODEL',
        description='Registration method: SCAN_TO_MODEL for scan-to-model pipeline'
    )
    
    # Small GICP parameters used by scan-to-model
    small_gicp_num_neighbors_arg = DeclareLaunchArgument(
        'small_gicp_num_neighbors',
        default_value='10',
        description='Number of neighbors for covariance estimation'
    )
    
    small_gicp_voxel_resolution_arg = DeclareLaunchArgument(
        'small_gicp_voxel_resolution',
        default_value='0.5',
        description='Voxel resolution for scan-to-model pipeline (meters)'
    )
    
    # LRU voxel map parameters - key for scan-to-model performance
    lru_horizon_arg = DeclareLaunchArgument(
        'lru_horizon',
        default_value='150',
        description='LRU horizon for voxel map memory management'
    )
    
    lru_clear_cycle_arg = DeclareLaunchArgument(
        'lru_clear_cycle',
        default_value='8',
        description='LRU clear cycle frequency'
    )
    
    voxel_search_offsets_arg = DeclareLaunchArgument(
        'voxel_search_offsets',
        default_value='1',
        description='Number of voxel search offsets'
    )
    
    # Processing parameters optimized for scan-to-model
    vg_size_input_arg = DeclareLaunchArgument(
        'vg_size_for_input',
        default_value='0.15',
        description='Voxel grid size for input cloud downsampling'
    )
    
    vg_size_map_arg = DeclareLaunchArgument(
        'vg_size_for_map',
        default_value='0.1',
        description='Voxel grid size for map cloud downsampling'
    )
    
    trans_mapupdate_arg = DeclareLaunchArgument(
        'trans_for_mapupdate',
        default_value='0.8',
        description='Translation threshold for map update (meters)'
    )
    
    use_min_max_filter_arg = DeclareLaunchArgument(
        'use_min_max_filter',
        default_value='true',
        description='Enable min/max range filtering'
    )
    
    scan_min_range_arg = DeclareLaunchArgument(
        'scan_min_range',
        default_value='0.1',
        description='Minimum scan range (meters)'
    )
    
    scan_max_range_arg = DeclareLaunchArgument(
        'scan_max_range',
        default_value='40.0',
        description='Maximum scan range (meters)'
    )
    
    debug_flag_arg = DeclareLaunchArgument(
        'debug_flag',
        default_value='false',
        description='Enable debug output'
    )

    # scanmatcher node
    scanmatcher_node = Node(
        package='scanmatcher',
        executable='scanmatcher_node_copy',
        name='scanmatcher_copy',
        output='screen',
        parameters=[{
            'global_frame_id': 'map',
            'robot_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'registration_method': LaunchConfiguration('registration_method'),
            
            # Small GICP parameters for scan-to-model
            'small_gicp_num_neighbors': LaunchConfiguration('small_gicp_num_neighbors'),
            'small_gicp_voxel_resolution': LaunchConfiguration('small_gicp_voxel_resolution'),
            
            # LRU voxel map parameters - crucial for scan-to-model performance
            'lru_horizon': LaunchConfiguration('lru_horizon'),
            'lru_clear_cycle': LaunchConfiguration('lru_clear_cycle'),
            'voxel_search_offsets': LaunchConfiguration('voxel_search_offsets'),
            
            # Processing parameters
            'vg_size_for_input': LaunchConfiguration('vg_size_for_input'),
            'vg_size_for_map': LaunchConfiguration('vg_size_for_map'),
            'trans_for_mapupdate': LaunchConfiguration('trans_for_mapupdate'),
            'use_min_max_filter': LaunchConfiguration('use_min_max_filter'),
            'scan_min_range': LaunchConfiguration('scan_min_range'),
            'scan_max_range': LaunchConfiguration('scan_max_range'),
            'debug_flag': LaunchConfiguration('debug_flag'),
            
            # Pose and other settings
            'set_initial_pose': True,
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_z': 0.0,
            'initial_pose_qx': 0.0,
            'initial_pose_qy': 0.0,
            'initial_pose_qz': 0.0,
            'initial_pose_qw': 1.0,
            'publish_tf': True,
            'use_odom': False,
            'use_imu': False,
            'scan_period': 0.1,
            'map_publish_period': 8.0,
            'num_targeted_cloud': 10,
        }],
        remappings=[
            ('input_cloud', 'ouster/points'),
            ('current_pose', 'current_pose'),
            ('map', 'map'),
            ('path', 'path'),
        ]
    )

    return LaunchDescription([
        registration_method_arg,
        small_gicp_num_neighbors_arg,
        small_gicp_voxel_resolution_arg,
        lru_horizon_arg,
        lru_clear_cycle_arg,
        voxel_search_offsets_arg,
        vg_size_input_arg,
        vg_size_map_arg,
        trans_mapupdate_arg,
        use_min_max_filter_arg,
        scan_min_range_arg,
        scan_max_range_arg,
        debug_flag_arg,
        scanmatcher_node,
    ])