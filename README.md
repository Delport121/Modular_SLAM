# Modular-SLAM

**Note: This package was inspired by the architecture used in [lidarslam_ros2](https://github.com/rsasaki0109/lidarslam_ros2) by Ryohei Sasaki. All original credits go to the original author.**

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

- SCAN context paramters needs to be tuned in its header file in the Thirdparty folder. Its const variables it cannot be changed with ros
---