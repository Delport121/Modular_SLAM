# Modular-SLAM

**Note: This package was inspired by the architecture used in [lidarslam_ros2](https://github.com/rsasaki0109/lidarslam_ros2) by Ryohei Sasaki. All original credits go to the original author.**

## Installation and Build Guide

### Prerequisites
- ROS 2 (Humble or later)
- PCL (Point Cloud Library)
- Eigen3
- g2o (graph optimization library)
- OpenMP

### Installation Steps

1. **Clone the repository into your ROS 2 workspace:**
   ```bash
   cd ~/your_ros2_workspace/src
   git clone https://github.com/Delport121/Modular_SLAM.git lidarslam_ros2
   ```

2. **Install dependencies:**
   ```bash
   cd ~/your_ros2_workspace
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   cd ~/your_ros2_workspace
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

4. **Source the workspace:**
   ```bash
   source ~/your_ros2_workspace/install/setup.bash
   ```

## Running the SLAM System

### Option 1: Running Frontend and Backend Separately

This approach gives you more control and is useful for debugging individual components.

**Terminal 1 - Launch Frontend:**
```bash
ros2 launch frontend_slam frontendslam.launch.py
```

**Terminal 2 - Launch Backend:**
```bash
ros2 launch backend_slam backendslam.launch.py
```

### Option 2: Running Complete System Together

Launch the entire SLAM system with a single command using the integrated launch file:

```bash
ros2 launch lidarslam lidarslam.launch.py
```

**Other integrated launch options:**
```bash
# For Voyager robot configuration
ros2 launch lidarslam lidarslam_voyager.launch.py

# For Tukuba dataset
ros2 launch lidarslam lidarslam_tukuba.launch.py
```

## Configuration and Customization

### Tuning Parameters

Parameters can be adjusted in the YAML configuration files located in:
- `frontend_slam/config/` - Frontend SLAM parameters
- `backend_slam/param/` - Backend optimization parameters
- `lidarslam/param/` - Integrated system parameters

**Available frontend configurations:**
- `mapping_robot.yaml` - General robot mapping
- `mapping_car.yaml` - Car/vehicle mapping
- `scan_to_model.yaml` - Scan-to-model registration
- `small_gicp_balanced.yaml` - Balanced performance
- `small_gicp_high_speed.yaml` - High-speed operation
- `small_gicp_high_accuracy.yaml` - High accuracy

### Changing Configuration Files

To use a different configuration, modify the launch file. For example, in `frontendslam.launch.py`:

```python
parameters=[os.path.join(
    get_package_share_directory('frontend_slam'),
    'config',
    'scan_to_model.yaml'  # Change this to your desired config file
)]
```

### Topic Remapping

Topic remappings can be configured in the launch files. For example:

```python
remappings=[
    ('input_cloud', '/your/lidar/topic'),
    ('current_pose', '/your/pose/topic'),
    ('map', '/your/map/topic')
]
```

Common remappings in launch files:
- `input_cloud` - Point cloud input from LiDAR
- `current_pose` - Current robot pose output
- `map` - Map point cloud output
- `path` - Trajectory path output

## Modifications and Enhancements

The package currently has the following features:

### Added Features:
- **Small GICP Integration**: Enhanced scan matching with small_gicp library for improved accuracy and performance
- **Multiple Algorithm Support**: Support for various scan matching algorithms (NDT, GICP, Small GICP, VGICP)
- **ScanContext Integration**: Loop closure detection using ScanContext for robust SLAM
- **Enhanced Launch Configurations**: Multiple launch files for different scenarios and robot configurations
- **Improved Backend**: Enhanced graph-based SLAM with better optimization
- **Modular Design**: More modular and configurable system architecture

### New Launch Files:
- `slam_small_gicp.launch.py` - Small GICP based SLAM
- `slam_small_gicp_fast.launch.py` - High-speed configuration
- `slam_small_gicp_accurate.launch.py` - High-accuracy configuration
- `slam_vgicp.launch.py` - VGICP based SLAM
- `slam_scan_to_model.launch.py` - Scan-to-model matching
- `lidarslam_voyager.launch.py` - Configuration for Voyager robot

### Configuration Files:
- Multiple YAML configurations for different performance profiles
- Specialized configurations for different robot platforms

## Original README Content Below:

## To do
- SCAN context paramters needs to be tuned in its header file in the Thirdparty folder. Its const variables it cannot be changed with ros
- Test constant velocity model in more detial
- Test Lidar undistortion in more detail
- Try using GTSAM instead of g20 in the backend
---