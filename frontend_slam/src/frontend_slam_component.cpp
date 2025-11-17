#include "frontend_slam/frontend_slam_component.h"
#include <chrono>
#include <fstream>
#include <iomanip>

// Test small_gicp integration
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling.hpp>

// Small GICP PCL interface
#ifdef BUILD_WITH_SMALL_GICP_PCL
#include <small_gicp/pcl/pcl_registration.hpp>
#endif

// Scan-to-model pipeline includes
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/util/normal_estimation.hpp>
#include <small_gicp/registration/reduction_omp.hpp>

using namespace std::chrono_literals;

namespace frontendslam
{
FrontendSlamComponent::FrontendSlamComponent(const rclcpp::NodeOptions & options)
: Node("frontend_slam", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  RCLCPP_INFO(get_logger(), "initialization start");
  double ndt_resolution;
  int ndt_num_threads;
  double gicp_corr_dist_threshold;

  declare_parameter("global_frame_id", "map");
  get_parameter("global_frame_id", global_frame_id_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter("odom_frame_id", "odom");
  get_parameter("odom_frame_id", odom_frame_id_);
  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method_);
  declare_parameter("ndt_resolution", 1.0);
  get_parameter("ndt_resolution", ndt_resolution);
  declare_parameter("ndt_num_threads", 7);
  get_parameter("ndt_num_threads", ndt_num_threads);
  declare_parameter("gicp_corr_dist_threshold", 1.0);
  get_parameter("gicp_corr_dist_threshold", gicp_corr_dist_threshold);
  
  // Small GICP specific parameters
  declare_parameter("small_gicp_num_neighbors", 10);
  get_parameter("small_gicp_num_neighbors", small_gicp_num_neighbors_);
  declare_parameter("small_gicp_voxel_resolution", 1.0);
  get_parameter("small_gicp_voxel_resolution", small_gicp_voxel_resolution_);
  declare_parameter("small_gicp_registration_type", "GICP");
  get_parameter("small_gicp_registration_type", small_gicp_registration_type_);
  declare_parameter("small_gicp_max_correspondence_distance", 1.0);
  get_parameter("small_gicp_max_correspondence_distance", small_gicp_max_correspondence_distance_);
  
  // LRU voxel map parameters
  declare_parameter("lru_horizon", 100);
  get_parameter("lru_horizon", lru_horizon_);
  declare_parameter("lru_clear_cycle", 10);
  get_parameter("lru_clear_cycle", lru_clear_cycle_);
  declare_parameter("voxel_search_offsets", 1);
  get_parameter("voxel_search_offsets", voxel_search_offsets_);
  
  declare_parameter("trans_for_mapupdate", 1.0);
  get_parameter("trans_for_mapupdate", trans_for_mapupdate_);
  declare_parameter("vg_size_for_input", 0.2);
  get_parameter("vg_size_for_input", vg_size_for_input_);
  declare_parameter("vg_size_for_map", 0.1);
  get_parameter("vg_size_for_map", vg_size_for_map_);
  declare_parameter("use_min_max_filter", false);
  get_parameter("use_min_max_filter", use_min_max_filter_);
  declare_parameter("scan_min_range", 0.1);
  get_parameter("scan_min_range", scan_min_range_);
  declare_parameter("scan_max_range", 50.0);
  get_parameter("scan_max_range", scan_max_range_);
  declare_parameter("scan_period", 0.1);
  get_parameter("scan_period", scan_period_);
  declare_parameter("map_publish_period", 8.0);
  get_parameter("map_publish_period", map_publish_period_);  
  declare_parameter("num_targeted_cloud", 10);
  get_parameter("num_targeted_cloud", num_targeted_cloud_);
  if (num_targeted_cloud_ < 1) {
    std::cout << "num_targeted_cloud should be positive" << std::endl;
    num_targeted_cloud_ = 1;
  }

  declare_parameter("initial_pose_x", 0.0);
  get_parameter("initial_pose_x", initial_pose_x_);
  declare_parameter("initial_pose_y", 0.0);
  get_parameter("initial_pose_y", initial_pose_y_);
  declare_parameter("initial_pose_z", 0.0);
  get_parameter("initial_pose_z", initial_pose_z_);
  declare_parameter("initial_pose_qx", 0.0);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  declare_parameter("initial_pose_qy", 0.0);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  declare_parameter("initial_pose_qz", 0.0);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  declare_parameter("initial_pose_qw", 1.0);
  get_parameter("initial_pose_qw", initial_pose_qw_);

  declare_parameter("set_initial_pose", true);
  get_parameter("set_initial_pose", set_initial_pose_);
  declare_parameter("publish_tf", true);
  get_parameter("publish_tf", publish_tf_);
  declare_parameter("use_odom", false);
  get_parameter("use_odom", use_odom_);
  declare_parameter("use_imu", false);
  get_parameter("use_imu", use_imu_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);
  declare_parameter("tum_log_filename", "/tmp/tum_trajectory.txt");
  get_parameter("tum_log_filename", tum_log_filename_);
  declare_parameter("iteration_log_filename", "/tmp/iteration_stats.txt");
  get_parameter("iteration_log_filename", iteration_log_filename_);

  std::cout << "registration_method:" << registration_method_ << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "gicp_corr_dist_threshold[m]:" << gicp_corr_dist_threshold << std::endl;
  std::cout << "small_gicp_num_neighbors:" << small_gicp_num_neighbors_ << std::endl;
  std::cout << "small_gicp_voxel_resolution[m]:" << small_gicp_voxel_resolution_ << std::endl;
  std::cout << "small_gicp_registration_type:" << small_gicp_registration_type_ << std::endl;
  std::cout << "small_gicp_max_correspondence_distance[m]:" << small_gicp_max_correspondence_distance_ << std::endl;
  std::cout << "lru_horizon:" << lru_horizon_ << std::endl;
  std::cout << "lru_clear_cycle:" << lru_clear_cycle_ << std::endl;
  std::cout << "voxel_search_offsets:" << voxel_search_offsets_ << std::endl;
  std::cout << "trans_for_mapupdate[m]:" << trans_for_mapupdate_ << std::endl;
  std::cout << "vg_size_for_input[m]:" << vg_size_for_input_ << std::endl;
  std::cout << "vg_size_for_map[m]:" << vg_size_for_map_ << std::endl;
  std::cout << "use_min_max_filter:" << std::boolalpha << use_min_max_filter_ << std::endl;
  std::cout << "scan_min_range[m]:" << scan_min_range_ << std::endl;
  std::cout << "scan_max_range[m]:" << scan_max_range_ << std::endl;
  std::cout << "set_initial_pose:" << std::boolalpha << set_initial_pose_ << std::endl;
  std::cout << "publish_tf:" << std::boolalpha << publish_tf_ << std::endl;
  std::cout << "use_odom:" << std::boolalpha << use_odom_ << std::endl;
  std::cout << "use_imu:" << std::boolalpha << use_imu_ << std::endl;
  std::cout << "scan_period[sec]:" << scan_period_ << std::endl;
  std::cout << "debug_flag:" << std::boolalpha << debug_flag_ << std::endl;
  std::cout << "map_publish_period[sec]:" << map_publish_period_ << std::endl;
  std::cout << "num_targeted_cloud:" << num_targeted_cloud_ << std::endl;
  std::cout << "------------------" << std::endl;

  if (registration_method_ == "NDT") {

    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr
      ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    // ndt_omp
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}

    registration_ = ndt;

  } else if (registration_method_ == "GICP") {
	  boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
      gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
    gicp->setTransformationEpsilon(1e-8);
    registration_ = gicp;
  } else if (registration_method_ == "SMALL_GICP") {
#ifdef BUILD_WITH_SMALL_GICP_PCL
    boost::shared_ptr<small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>>
      small_gicp_reg(new small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>());
    
    // Configure small_gicp parameters
    small_gicp_reg->setNumThreads(ndt_num_threads);
    small_gicp_reg->setNumNeighborsForCovariance(small_gicp_num_neighbors_);
    small_gicp_reg->setVoxelResolution(small_gicp_voxel_resolution_);
    small_gicp_reg->setRegistrationType(small_gicp_registration_type_);
    small_gicp_reg->setMaxCorrespondenceDistance(small_gicp_max_correspondence_distance_);
    small_gicp_reg->setTransformationEpsilon(1e-8);
    
    registration_ = small_gicp_reg;
    RCLCPP_INFO(get_logger(), "Using Small GICP registration with type: %s", small_gicp_registration_type_.c_str());
#else
    RCLCPP_ERROR(get_logger(), "Small GICP not available. Compile with BUILD_WITH_SMALL_GICP_PCL=ON");
    exit(1);
#endif
  } else if (registration_method_ == "SCAN_TO_MODEL") {
    // Initialize scan-to-model pipeline
    voxel_map_ = nullptr;  // Will be initialized on first scan
    T_world_lidar_ = Eigen::Isometry3d::Identity();
    RCLCPP_INFO(get_logger(), "Using Scan-to-Model registration pipeline");
    
    // We don't need a traditional registration object for scan-to-model
    registration_ = nullptr;
  } else {
    RCLCPP_ERROR(get_logger(), "invalid registration method");
    exit(1);
  }

  map_array_msg_.header.frame_id = global_frame_id_;
  map_array_msg_.cloud_coordinate = map_array_msg_.LOCAL;

  path_.header.frame_id = global_frame_id_;

  lidar_undistortion_.setScanPeriod(scan_period_);

  initializePubSub();

  if (set_initial_pose_) {
    RCLCPP_INFO(get_logger(), "set initial pose");
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.position.x = initial_pose_x_;
    msg->pose.position.y = initial_pose_y_;
    msg->pose.position.z = initial_pose_z_;
    msg->pose.orientation.x = initial_pose_qx_;
    msg->pose.orientation.y = initial_pose_qy_;
    msg->pose.orientation.z = initial_pose_qz_;
    msg->pose.orientation.w = initial_pose_qw_;
    current_pose_stamped_ = *msg;
    pose_pub_->publish(current_pose_stamped_);
    initial_pose_received_ = true;

    path_.poses.push_back(*msg);
  }

  // Initialize TUM format logging if debug flag is active
  if (debug_flag_) {
    tum_pose_log_.open(tum_log_filename_, std::ios::out);
    if (tum_pose_log_.is_open()) {
      RCLCPP_INFO(get_logger(), "TUM format pose logging enabled: %s", tum_log_filename_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to open TUM format log file: %s", tum_log_filename_.c_str());
    }
    
    // Initialize iteration stats logging
    iteration_log_.open(iteration_log_filename_, std::ios::out);
    if (iteration_log_.is_open()) {
      RCLCPP_INFO(get_logger(), "Iteration stats logging enabled: %s", iteration_log_filename_.c_str());
      // Write header
      iteration_log_ << "iteration,time_ms,distance_moved,avg_time_ms" << std::endl;
    } else {
      RCLCPP_WARN(get_logger(), "Failed to open iteration log file: %s", iteration_log_filename_.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "initialization end");
}

void FrontendSlamComponent::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");
  // sub
  auto initial_pose_callback =
    [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
    {
      if (msg->header.frame_id != global_frame_id_) {
        RCLCPP_WARN(get_logger(), "This initial_pose is not in the global frame");
        return;
      }
      RCLCPP_INFO(get_logger(), "initial_pose is received");

      current_pose_stamped_ = *msg;
      previous_position_.x() = current_pose_stamped_.pose.position.x;
      previous_position_.y() = current_pose_stamped_.pose.position.y;
      previous_position_.z() = current_pose_stamped_.pose.position.z;
      initial_pose_received_ = true;

      pose_pub_->publish(current_pose_stamped_);
    };

  auto cloud_callback =
    [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
    {

      rclcpp::Clock system_clock;
      rclcpp::Time pipeline_start = system_clock.now();

      if (!initial_pose_received_)
      {
        RCLCPP_WARN(get_logger(), "initial_pose is not received");
        return;
      }

      sensor_msgs::msg::PointCloud2 transformed_msg;
      try {
        tf2::TimePoint time_point = tf2::TimePoint(
          std::chrono::seconds(msg->header.stamp.sec) +
          std::chrono::nanoseconds(msg->header.stamp.nanosec));
        const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
          robot_frame_id_, msg->header.frame_id, time_point);
        tf2::doTransform(*msg, transformed_msg, transform); // TODO:slow now(https://github.com/ros/geometry2/pull/432)
      } catch (tf2::TransformException & e) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s. Using untransformed point cloud.", e.what());
        transformed_msg = *msg;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(transformed_msg, *tmp_ptr);
      //pcl::fromROSMsg(*msg, *tmp_ptr);

      if (use_imu_) {
        double scan_time = msg->header.stamp.sec +
          msg->header.stamp.nanosec * 1e-9;
        lidar_undistortion_.adjustDistortion(tmp_ptr, scan_time);
      }

      if (use_min_max_filter_) {
        double r;
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr2(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto & p : tmp_ptr->points) {
          r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
          if (scan_min_range_ < r && r < scan_max_range_) {tmp_ptr2->points.push_back(p);}
        }
        tmp_ptr = tmp_ptr2;
      }

      if (!initial_cloud_received_) {
        RCLCPP_INFO(get_logger(), "Initial_cloud is received");
        initial_cloud_received_ = true;
        initializeMap(tmp_ptr, msg->header);
        last_map_time_ = clock_.now();
      }

      // Process cloud in main SLAM pipeline
      if (initial_cloud_received_) {receiveCloud(tmp_ptr, msg->header.stamp);}

      // Logging pipeline time
      rclcpp::Time pipeline_end = system_clock.now();
      iteration_count_ = iteration_count_ + 1;
      double pipeline_time_ms = (pipeline_end.seconds() - pipeline_start.seconds()) * 1000.0;
      RCLCPP_INFO(get_logger(), "Iteration count: %d, Pipeline time: %.3fms", iteration_count_, pipeline_time_ms);
      
      // Log iteration stats if debug flag is enabled
      if (debug_flag_) {
        logIterationStats(pipeline_time_ms, submap_distance_);
      }

    };

  auto imu_callback =
    [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
    {
      if (initial_pose_received_) {receiveImu(*msg);}
    };

  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseStamped>(
    "initial_pose", rclcpp::QoS(10), initial_pose_callback);

  imu_sub_ =
    create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(), imu_callback);

  input_cloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", rclcpp::SensorDataQoS(), cloud_callback); 

  // pub
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "current_pose",
    rclcpp::QoS(10));
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("unmodified_map", rclcpp::QoS(10));
  map_submap_pub_ = create_publisher<lidarslam_msgs::msg::SubMap>("submap", rclcpp::QoS(10));
  map_array_pub_ =
    create_publisher<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(
      rclcpp::KeepLast(
        1)).reliable());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void FrontendSlamComponent::initializeMap(const pcl::PointCloud <pcl::PointXYZI>::Ptr & tmp_ptr, const std_msgs::msg::Header & header)
{
  RCLCPP_INFO(get_logger(), "create a first map");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
  voxel_grid.setInputCloud(tmp_ptr);
  voxel_grid.filter(*cloud_ptr);

  Eigen::Matrix4f sim_trans = getTransformation(current_pose_stamped_.pose);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, sim_trans);
  
  // Only set input target for traditional registration methods
  if (registration_method_ != "SCAN_TO_MODEL") {
    registration_->setInputTarget(transformed_cloud_ptr);
  } else {
    // For scan-to-model, initialize the voxel map with the first cloud
    auto points = pclToSmallGicp(cloud_ptr);
    auto downsampled = small_gicp::voxelgrid_sampling(*points, small_gicp_voxel_resolution_);
    small_gicp::estimate_covariances(*downsampled, small_gicp_num_neighbors_);
    voxel_map_ = std::make_shared<small_gicp::GaussianVoxelMap>(small_gicp_voxel_resolution_);
    
    // Configure LRU parameters
    voxel_map_->lru_horizon = lru_horizon_;
    voxel_map_->lru_clear_cycle = lru_clear_cycle_;
    voxel_map_->set_search_offsets(voxel_search_offsets_);
    
    voxel_map_->insert(*downsampled);
  }

  // map
  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*transformed_cloud_ptr, *map_msg_ptr);

  // map array
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud_ptr, *cloud_msg_ptr);
  lidarslam_msgs::msg::SubMap submap;
  submap.header = header;
  submap.distance = 0;
  submap.pose = current_pose_stamped_.pose;
  submap.cloud = *cloud_msg_ptr;
  map_array_msg_.header = header;
  map_array_msg_.submaps.push_back(submap);

  map_pub_->publish(submap.cloud);
}

void FrontendSlamComponent::receiveCloud(
  const pcl::PointCloud < pcl::PointXYZI> ::ConstPtr & input_cloud_ptr,
  const rclcpp::Time stamp)
{
  if (mapping_flag_ && mapping_future_.valid()) {
    auto status = mapping_future_.wait_for(0s);
    if (status == std::future_status::ready) {
      if (is_map_updated_ == true && registration_method_ != "SCAN_TO_MODEL") {
        pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(
            targeted_cloud_));
        if (registration_method_ == "NDT") {
          registration_->setInputTarget(targeted_cloud_ptr);
        } else {
          pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_targeted_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
          pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
          voxel_grid.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
          voxel_grid.setInputCloud(targeted_cloud_ptr);
          voxel_grid.filter(*filtered_targeted_cloud_ptr);
          registration_->setInputTarget(filtered_targeted_cloud_ptr);
        }
        is_map_updated_ = false;
      }
      mapping_flag_ = false;
      mapping_thread_.detach();
    }
  }

  /* Filter input cloud and add it to the registration */
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
  voxel_grid.setInputCloud(input_cloud_ptr);
  voxel_grid.filter(*filtered_cloud_ptr);
  
  // Set up registration based on method
  if (registration_method_ != "SCAN_TO_MODEL") {
    registration_->setInputSource(filtered_cloud_ptr);
  }

  // I raninto issue with the conversion from ros poses to eigen matrices. This happens rarely, but it is a wierd issue with the SCAN_TO_MODEL. 
  // Preferably, do not convert pose formats to much
  // FOr future todo: Change all Eigen::Matrix4f to Eigen::Isometry3d for better consistency
  Eigen::Matrix4f sim_trans = Eigen::Matrix4f::Identity();  
  if (registration_method_ != "SCAN_TO_MODEL") {
    sim_trans = getTransformation(current_pose_stamped_.pose);
  } else {
    sim_trans = T_world_lidar_.matrix().cast<float>();
  }

  /* Use odometry for better initialization */
  if (use_odom_) {
    geometry_msgs::msg::TransformStamped odom_trans;
    try {
      odom_trans = tfbuffer_.lookupTransform(
        odom_frame_id_, robot_frame_id_, tf2_ros::fromMsg(
          stamp));
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    Eigen::Affine3d odom_affine = tf2::transformToEigen(odom_trans);
    Eigen::Matrix4f odom_mat = odom_affine.matrix().cast<float>();
    if (previous_odom_mat_ != Eigen::Matrix4f::Identity()) {
      sim_trans = sim_trans * previous_odom_mat_.inverse() * odom_mat;
    }
    previous_odom_mat_ = odom_mat;
  }

  /*Do alignment*/
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  Eigen::Matrix4f final_transformation;
  
  if (registration_method_ == "SCAN_TO_MODEL") {
    // Use scan-to-model pipeline with initial transformation
    final_transformation = scanToModelAlignment(filtered_cloud_ptr, sim_trans);
  } else {
    // Use traditional PCL registration
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    registration_->align(*output_cloud, sim_trans);
    final_transformation = registration_->getFinalTransformation();
  }
  
  rclcpp::Time time_align_end = system_clock.now();

  publishMapAndPose(input_cloud_ptr, final_transformation, stamp);

  if (!debug_flag_) {return;}

  tf2::Quaternion quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(current_pose_stamped_.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "nanoseconds: " << stamp.nanoseconds() << std::endl;
  std::cout << "submap_moving_distance: " << submap_distance_ << std::endl;
  std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() << "s" <<
    std::endl;
  std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
  std::cout << "initial transformation:" << std::endl;
  std::cout << sim_trans << std::endl;
  if (registration_method_ != "SCAN_TO_MODEL") {
    std::cout << "has converged: " << registration_->hasConverged() << std::endl;
    std::cout << "fitness score: " << registration_->getFitnessScore() << std::endl;
  } else {
    std::cout << "scan-to-model alignment completed" << std::endl;
    std::cout << "voxel map size: " << voxel_map_->size() << std::endl;
  }
  std::cout << "final transformation:" << std::endl;
  std::cout << final_transformation << std::endl;
  std::cout << "rpy" << std::endl;
  std::cout << "roll:" << roll * 180 / M_PI << "," <<
    "pitch:" << pitch * 180 / M_PI << "," <<
    "yaw:" << yaw * 180 / M_PI << std::endl;
  int num_submaps = map_array_msg_.submaps.size();
  std::cout << "num_submaps:" << num_submaps << std::endl;
  std::cout << "moving distance:" << latest_distance_ << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  
  // Log pose in TUM format for evo processing
  logPoseInTUMFormat(stamp, current_pose_stamped_.pose);
}

void FrontendSlamComponent::publishMapAndPose(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_ptr,
  const Eigen::Matrix4f final_transformation, const rclcpp::Time stamp)
{

  Eigen::Vector3d position = final_transformation.block<3, 1>(0, 3).cast<double>();

  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  if(publish_tf_){
    geometry_msgs::msg::TransformStamped base_to_map_msg;
    base_to_map_msg.header.stamp = stamp;
    base_to_map_msg.header.frame_id = global_frame_id_;
    base_to_map_msg.child_frame_id = robot_frame_id_;
    base_to_map_msg.transform.translation.x = position.x();
    base_to_map_msg.transform.translation.y = position.y();
    base_to_map_msg.transform.translation.z = position.z();
    base_to_map_msg.transform.rotation = quat_msg;

    if(use_odom_){
        geometry_msgs::msg::TransformStamped odom_to_map_msg;
        odom_to_map_msg = calculateMaptoOdomTransform(base_to_map_msg, stamp);
        broadcaster_.sendTransform(odom_to_map_msg);
    }
    else{
      broadcaster_.sendTransform(base_to_map_msg);
    }
  }

  current_pose_stamped_.header.stamp = stamp;
  current_pose_stamped_.pose.position.x = position.x();
  current_pose_stamped_.pose.position.y = position.y();
  current_pose_stamped_.pose.position.z = position.z();
  current_pose_stamped_.pose.orientation = quat_msg;
  pose_pub_->publish(current_pose_stamped_);

  path_.poses.push_back(current_pose_stamped_);
  path_pub_->publish(path_);

  /* publish submap */
  map_submap_msg_.header.stamp = stamp;
  map_submap_msg_.header.frame_id = global_frame_id_;
  trans_submap_ = (position - previous_submap_position_).norm();
  submap_distance_ += trans_submap_;
  map_submap_msg_.distance = submap_distance_;
  previous_submap_position_ = position;
  map_submap_msg_.pose = current_pose_stamped_.pose;
  pcl::toROSMsg(*cloud_ptr, map_submap_msg_.cloud);
  map_submap_pub_->publish(map_submap_msg_);

  /* check whether to update map */
  trans_ = (position - previous_position_).norm();
  if (trans_ >= trans_for_mapupdate_ && !mapping_flag_) {
    geometry_msgs::msg::PoseStamped current_pose_stamped;
    current_pose_stamped = current_pose_stamped_;
    previous_position_ = position;
    mapping_task_ =
      std::packaged_task<void()>(
      std::bind(
        &FrontendSlamComponent::updateMap, this, cloud_ptr,
        final_transformation, current_pose_stamped));
    mapping_future_ = mapping_task_.get_future();
    mapping_thread_ = std::thread(std::move(std::ref(mapping_task_)));
    mapping_flag_ = true;
  }
}

void FrontendSlamComponent::updateMap(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr,
  const Eigen::Matrix4f final_transformation,
  const geometry_msgs::msg::PoseStamped current_pose_stamped)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
  voxel_grid.setInputCloud(cloud_ptr);
  voxel_grid.filter(*filtered_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*filtered_cloud_ptr, *transformed_cloud_ptr, final_transformation);

  /* Used to setup target cloud*/
  targeted_cloud_.clear();
  targeted_cloud_ += *transformed_cloud_ptr;
  int num_submaps = map_array_msg_.submaps.size();
  for (int i = 0; i < num_targeted_cloud_ - 1; i++) {
    if (num_submaps - 1 - i < 0) {continue;}
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg_.submaps[num_submaps - 1 - i].cloud, *tmp_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Affine3d submap_affine;
    tf2::fromMsg(map_array_msg_.submaps[num_submaps - 1 - i].pose, submap_affine);
    pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
    targeted_cloud_ += *transformed_tmp_ptr;
  }

  /* map array */
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*filtered_cloud_ptr, *cloud_msg_ptr);

  lidarslam_msgs::msg::SubMap submap;
  submap.header.frame_id = global_frame_id_;
  submap.header.stamp = current_pose_stamped.header.stamp;
  latest_distance_ += trans_;
  submap.distance = latest_distance_;
  submap.pose = current_pose_stamped.pose;
  submap.cloud = *cloud_msg_ptr;
  submap.cloud.header.frame_id = global_frame_id_;
  map_array_msg_.header.stamp = current_pose_stamped.header.stamp;
  map_array_msg_.submaps.push_back(submap);
  map_array_pub_->publish(map_array_msg_);

  is_map_updated_ = true;

  rclcpp::Time map_time = clock_.now();
  double dt = map_time.seconds() - last_map_time_.seconds();
  if (dt > map_publish_period_) {
    publishMap(map_array_msg_, global_frame_id_);
    last_map_time_ = map_time;
  }
}

Eigen::Matrix4f FrontendSlamComponent::getTransformation(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f sim_trans = affine.matrix().cast<float>();
  return sim_trans;
}

void FrontendSlamComponent::receiveImu(const sensor_msgs::msg::Imu msg)
{
  if (!use_imu_) {return;}

  double roll, pitch, yaw;
  tf2::Quaternion orientation;
  tf2::fromMsg(msg.orientation, orientation);
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  float acc_x = static_cast<float>(msg.linear_acceleration.x) + sin(pitch) * 9.81;
  float acc_y = static_cast<float>(msg.linear_acceleration.y) - cos(pitch) * sin(roll) * 9.81;
  float acc_z = static_cast<float>(msg.linear_acceleration.z) - cos(pitch) * cos(roll) * 9.81;

  Eigen::Vector3f angular_velo{
    static_cast<float>(msg.angular_velocity.x),
    static_cast<float>(msg.angular_velocity.y),
    static_cast<float>(msg.angular_velocity.z)};
  Eigen::Vector3f acc{acc_x, acc_y, acc_z};
  Eigen::Quaternionf quat{
    static_cast<float>(msg.orientation.w),
    static_cast<float>(msg.orientation.x),
    static_cast<float>(msg.orientation.y),
    static_cast<float>(msg.orientation.z)};
  double imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);
}

void FrontendSlamComponent::publishMap(const lidarslam_msgs::msg::MapArray & map_array_msg , const std::string & map_frame_id)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for (auto & submap : map_array_msg.submaps) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);

    Eigen::Affine3d affine;
    tf2::fromMsg(submap.pose, affine);
    pcl::transformPointCloud(
      *submap_cloud_ptr, *transformed_submap_cloud_ptr,
      affine.matrix().cast<float>());

    *map_ptr += *transformed_submap_cloud_ptr;
  }
  RCLCPP_INFO(get_logger(), "publish a map, number of points in the map : %ld", map_ptr->size());

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = map_frame_id;
  map_pub_->publish(*map_msg_ptr);
}

geometry_msgs::msg::TransformStamped FrontendSlamComponent::calculateMaptoOdomTransform(
  const geometry_msgs::msg::TransformStamped &base_to_map_msg,
  const rclcpp::Time stamp
)
{
  geometry_msgs::msg::TransformStamped odom_to_map_msg;
  try {
    geometry_msgs::msg::PoseStamped odom_to_map;
    geometry_msgs::msg::PoseStamped base_to_map;

    tf2::Transform odom_to_map_tf;
    tf2::Transform base_to_map_msg_tf;
    base_to_map.header.frame_id = robot_frame_id_;

    tf2::fromMsg(base_to_map_msg.transform, base_to_map_msg_tf);
    tf2::toMsg(base_to_map_msg_tf.inverse(), base_to_map.pose);
    tfbuffer_.transform(base_to_map, odom_to_map, odom_frame_id_);
    tf2::impl::Converter<true, false>::convert(odom_to_map.pose, odom_to_map_tf);
    tf2::impl::Converter<false, true>::convert(odom_to_map_tf.inverse(), odom_to_map_msg.transform);

    odom_to_map_msg.header.stamp = stamp;
    odom_to_map_msg.header.frame_id = global_frame_id_ ;
    odom_to_map_msg.child_frame_id = odom_frame_id_;
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform from base_link to odom failed: %s", e.what());
  }
  return odom_to_map_msg;
}

// Scan-to-model helper functions
small_gicp::PointCloud::Ptr FrontendSlamComponent::pclToSmallGicp(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pcl_cloud) {
  auto points = std::make_shared<small_gicp::PointCloud>();
  points->resize(pcl_cloud->size());
  
  // Could potentially do this in a more optimised way than a loop
  for (size_t i = 0; i < pcl_cloud->size(); ++i) {
    points->point(i) = Eigen::Vector4d(
      pcl_cloud->points[i].x,
      pcl_cloud->points[i].y, 
      pcl_cloud->points[i].z,
      1.0
    );
  }
  
  return points;
}

Eigen::Matrix4f FrontendSlamComponent::scanToModelAlignment(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_ptr, const Eigen::Matrix4f& initial_transform) {
  // Convert PCL to small_gicp point cloud
  auto points = pclToSmallGicp(cloud_ptr);
  
  // Downsample the point cloud (already done by PCL voxel grid, but small_gicp expects its format)
  auto downsampled = small_gicp::voxelgrid_sampling(*points, small_gicp_voxel_resolution_);
  
  // Estimate covariances for the points
  small_gicp::estimate_covariances(*downsampled, small_gicp_num_neighbors_);
  
  if (voxel_map_ == nullptr) {
    // First frame - initialize the voxel map
    voxel_map_ = std::make_shared<small_gicp::GaussianVoxelMap>(small_gicp_voxel_resolution_);
    // voxel_map_->lru_horizon = lru_horizon_;
    // voxel_map_->lru_clear_cycle = lru_clear_cycle_;
    // voxel_map_->set_search_offsets(voxel_search_offsets_);
    
    voxel_map_->insert(*downsampled);
    
    // Initialize T_world_lidar_ with the initial transform for first frame
    T_world_lidar_ = Eigen::Isometry3d(initial_transform.cast<double>());
    
    // Return the initial transformation for first frame
    return initial_transform;
  }
  
  // Convert initial transform to Isometry3d for use with small_gicp
  Eigen::Isometry3d initial_transform_iso = Eigen::Isometry3d(initial_transform.cast<double>());
  
  // Perform registration using GICP with initial transformation from input (may include odometry)
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
  auto result = registration.align(*voxel_map_, *downsampled, *voxel_map_, initial_transform_iso);
  // auto result = registration.align(*voxel_map_, *downsampled, *voxel_map_, T_world_lidar_);
  
  // Update transformation
  T_world_lidar_ = result.T_target_source;
  
  // Insert new points into the voxel map
  voxel_map_->insert(*downsampled, T_world_lidar_);
  
  // Convert Isometry3d to Matrix4f for compatibility
  return T_world_lidar_.matrix().cast<float>();
}

void FrontendSlamComponent::logPoseInTUMFormat(const rclcpp::Time& timestamp, const geometry_msgs::msg::Pose& pose) {
  if (!tum_pose_log_.is_open()) {
    return;
  }
  
  // TUM format: timestamp tx ty tz qx qy qz qw
  double time_seconds = timestamp.seconds();
  tum_pose_log_ << std::fixed << std::setprecision(6) << time_seconds << " "
                << std::setprecision(6) << pose.position.x << " "
                << pose.position.y << " "
                << pose.position.z << " "
                << pose.orientation.x << " "
                << pose.orientation.y << " "
                << pose.orientation.z << " "
                << pose.orientation.w << std::endl;
}

void FrontendSlamComponent::logIterationStats(double iteration_time_ms, double distance_moved) {
  if (!iteration_log_.is_open()) {
    return;
  }
  
  // Update cumulative time
  total_iteration_time_ += iteration_time_ms;
  
  // Calculate average time
  double avg_time_ms = total_iteration_time_ / iteration_count_;
  
  // Log: iteration, time_ms, distance_moved, avg_time_ms
  iteration_log_ << iteration_count_ << ","
                 << std::fixed << std::setprecision(3) << iteration_time_ms << ","
                 << std::setprecision(6) << distance_moved << ","
                 << std::setprecision(3) << avg_time_ms << std::endl;
}

} // namespace frontendslam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(frontendslam::FrontendSlamComponent)
