#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Parameters
    global_frame_id = LaunchConfiguration('global_frame_id')
    robot_frame_id = LaunchConfiguration('robot_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    registration_method = LaunchConfiguration('registration_method')
    
    # VGICP specific parameters - optimized for speed
    small_gicp_num_neighbors = LaunchConfiguration('small_gicp_num_neighbors')
    small_gicp_voxel_resolution = LaunchConfiguration('small_gicp_voxel_resolution')
    small_gicp_max_correspondence_distance = LaunchConfiguration('small_gicp_max_correspondence_distance')
    
    # LRU voxel map parameters
    lru_horizon = LaunchConfiguration('lru_horizon')
    lru_clear_cycle = LaunchConfiguration('lru_clear_cycle')
    voxel_search_offsets = LaunchConfiguration('voxel_search_offsets')
    
    trans_for_mapupdate = LaunchConfiguration('trans_for_mapupdate')
    vg_size_for_input = LaunchConfiguration('vg_size_for_input')
    vg_size_for_map = LaunchConfiguration('vg_size_for_map')
    scan_max_range = LaunchConfiguration('scan_max_range')
    num_targeted_cloud = LaunchConfiguration('num_targeted_cloud')
    debug_flag = LaunchConfiguration('debug_flag')
    
    ndt_thread_num = LaunchConfiguration('ndt_num_threads')

    # Node
    scanmatcher_node = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        name='scanmatcher',
        output='screen',
        parameters=[
            {'global_frame_id': global_frame_id},
            {'robot_frame_id': robot_frame_id},
            {'odom_frame_id': odom_frame_id},
            {'registration_method': registration_method},
            
            # VGICP parameters - optimized for speed
            {'small_gicp_num_neighbors': small_gicp_num_neighbors},
            {'small_gicp_voxel_resolution': small_gicp_voxel_resolution},
            {'small_gicp_max_correspondence_distance': small_gicp_max_correspondence_distance},
            
            # LRU voxel map parameters
            {'lru_horizon': lru_horizon},
            {'lru_clear_cycle': lru_clear_cycle},
            {'voxel_search_offsets': voxel_search_offsets},
            
            {'trans_for_mapupdate': trans_for_mapupdate},
            {'vg_size_for_input': vg_size_for_input},
            {'vg_size_for_map': vg_size_for_map},
            {'scan_max_range': scan_max_range},
            {'num_targeted_cloud': num_targeted_cloud},
            {'debug_flag': debug_flag},
            
            {'set_initial_pose': True},
            {'publish_tf': True},
            {'use_odom': False},
            {'use_imu': False},
            {'use_min_max_filter': True},
            {'scan_min_range': 0.1},
            {'scan_period': 0.1},
            {'map_publish_period': 8.0},
            
            # Initial pose
            {'initial_pose_x': 0.0},
            {'initial_pose_y': 0.0},
            {'initial_pose_z': 0.0},
            {'initial_pose_qx': 0.0},
            {'initial_pose_qy': 0.0},
            {'initial_pose_qz': 0.0},
            {'initial_pose_qw': 1.0},
        ],
        remappings=[
            ('input_cloud', 'ouster/points'), #a200_1057/cloud3/map #ouster/points #points_raw
            # ('current_pose', 'current_pose'),
            # ('map', 'map'),
            # ('path', 'path'),
            # ('tf_static', 'a200_1057/tf_static'),
            # ('tf', 'a200_1057/tf'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument('robot_frame_id', default_value='base_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('registration_method', default_value='SMALL_GICP'), # NDT, GICP, or SMALL_GICP
        
        # VGICP optimized parameters
        DeclareLaunchArgument('small_gicp_num_neighbors', default_value='10'),
        DeclareLaunchArgument('small_gicp_voxel_resolution', default_value='0.2'),
        DeclareLaunchArgument('small_gicp_max_correspondence_distance', default_value='0.3'),
        
        # LRU voxel map parameters
        DeclareLaunchArgument('lru_horizon', default_value='100'),
        DeclareLaunchArgument('lru_clear_cycle', default_value='10'),
        DeclareLaunchArgument('voxel_search_offsets', default_value='1'),
        
        DeclareLaunchArgument('trans_for_mapupdate', default_value='1.0'),
        DeclareLaunchArgument('vg_size_for_input', default_value='0.2'),
        DeclareLaunchArgument('vg_size_for_map', default_value='0.1'),
        DeclareLaunchArgument('scan_max_range', default_value='50.0'),
        DeclareLaunchArgument('num_targeted_cloud', default_value='10'),
        DeclareLaunchArgument('debug_flag', default_value='false'),
        
        DeclareLaunchArgument('ndt_num_threads', default_value='7'),
        DeclareLaunchArgument('use_min_max_filter', default_value='true'),
        DeclareLaunchArgument('scan_min_range', default_value='0.1'),
        DeclareLaunchArgument('scan_max_range', default_value='50.0'),
        
        scanmatcher_node,
    ])