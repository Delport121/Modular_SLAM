import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch arguments for easy parameter tuning
    registration_method_arg = DeclareLaunchArgument(
        'registration_method',
        default_value='SMALL_GICP',
        description='Registration method: NDT, GICP, or SMALL_GICP'
    )
    
    small_gicp_type_arg = DeclareLaunchArgument(
        'small_gicp_registration_type',
        default_value='GICP',
        description='Small GICP registration type: GICP or VGICP'
    )
    
    num_neighbors_arg = DeclareLaunchArgument(
        'small_gicp_num_neighbors',
        default_value='10',
        description='Number of neighbors for covariance estimation'
    )
    
    voxel_resolution_arg = DeclareLaunchArgument(
        'small_gicp_voxel_resolution',
        default_value='0.2',
        description='Voxel resolution for VGICP (meters)'
    )
    
    max_correspondence_distance_arg = DeclareLaunchArgument(
        'small_gicp_max_correspondence_distance',
        default_value='0.3',
        description='Maximum correspondence distance (meters)'
    )
    
    ndt_threads_arg = DeclareLaunchArgument(
        'ndt_num_threads',
        default_value='7',
        description='Number of threads for parallel processing'
    )
    
    vg_size_input_arg = DeclareLaunchArgument(
        'vg_size_for_input',
        default_value='0.2',
        description='Voxel grid size for input cloud downsampling'
    )
    
    vg_size_map_arg = DeclareLaunchArgument(
        'vg_size_for_map',
        default_value='0.1',
        description='Voxel grid size for map cloud downsampling'
    )
    
    trans_mapupdate_arg = DeclareLaunchArgument(
        'trans_for_mapupdate',
        default_value='1.0',
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
        default_value='50.0',
        description='Maximum scan range (meters)'
    )
    
    debug_flag_arg = DeclareLaunchArgument(
        'debug_flag',
        default_value='false',
        description='Enable debug output'
    )
    
    # LRU voxel map parameters
    lru_horizon_arg = DeclareLaunchArgument(
        'lru_horizon',
        default_value='100',
        description='LRU horizon for voxel map memory management'
    )
    
    lru_clear_cycle_arg = DeclareLaunchArgument(
        'lru_clear_cycle',
        default_value='10',
        description='LRU clear cycle frequency'
    )
    
    voxel_search_offsets_arg = DeclareLaunchArgument(
        'voxel_search_offsets',
        default_value='1',
        description='Number of voxel search offsets'
    )

    # scanmatcher node
    scanmatcher_node = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        name='scanmatcher',
        output='screen',
        parameters=[{
            'global_frame_id': 'map',
            'robot_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'registration_method': LaunchConfiguration('registration_method'),
            
            # Small GICP specific parameters
            'small_gicp_registration_type': LaunchConfiguration('small_gicp_registration_type'),
            'small_gicp_num_neighbors': LaunchConfiguration('small_gicp_num_neighbors'),
            'small_gicp_voxel_resolution': LaunchConfiguration('small_gicp_voxel_resolution'),
            'small_gicp_max_correspondence_distance': LaunchConfiguration('small_gicp_max_correspondence_distance'),
            
            # LRU voxel map parameters
            'lru_horizon': LaunchConfiguration('lru_horizon'),
            'lru_clear_cycle': LaunchConfiguration('lru_clear_cycle'),
            'voxel_search_offsets': LaunchConfiguration('voxel_search_offsets'),
            
            # Common parameters
            'ndt_num_threads': LaunchConfiguration('ndt_num_threads'),
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
            ('input_cloud', 'a200_1057/cloud3/map'), #/a200_1057/cloud3/map #ouster/points
            ('current_pose', 'current_pose'),
            ('map', 'map'),
            ('path', 'path'),
            ('tf_static', 'a200_1057/tf_static'),
            ('tf', 'a200_1057/tf'),
        ]
    )

    return LaunchDescription([
        registration_method_arg,
        small_gicp_type_arg,
        num_neighbors_arg,
        voxel_resolution_arg,
        max_correspondence_distance_arg,
        ndt_threads_arg,
        vg_size_input_arg,
        vg_size_map_arg,
        trans_mapupdate_arg,
        use_min_max_filter_arg,
        scan_min_range_arg,
        scan_max_range_arg,
        debug_flag_arg,
        lru_horizon_arg,
        lru_clear_cycle_arg,
        voxel_search_offsets_arg,
        scanmatcher_node,
    ])