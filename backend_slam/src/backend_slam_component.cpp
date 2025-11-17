#include "backend_slam/backend_slam_component.h"
#include <chrono>
#include <fstream>
#include <iomanip>

// Small GICP PCL interface
#ifdef BUILD_WITH_SMALL_GICP_PCL
#include <small_gicp/pcl/pcl_registration.hpp>
#endif

#include "Scancontext.h" 

using namespace std::chrono_literals;

namespace backendslam
{
BackendSlamComponent::BackendSlamComponent(const rclcpp::NodeOptions & options)
: Node("backend_slam", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  RCLCPP_INFO(get_logger(), "initialization start");
  std::string registration_method;
  double voxel_leaf_size;
  double ndt_resolution;
  int ndt_num_threads;

  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method);
  declare_parameter("voxel_leaf_size", 0.2);
  get_parameter("voxel_leaf_size", voxel_leaf_size);
  declare_parameter("ndt_resolution", 1.0);
  get_parameter("ndt_resolution", ndt_resolution);
  declare_parameter("ndt_num_threads", 2);
  get_parameter("ndt_num_threads", ndt_num_threads);
  declare_parameter("small_gicp_num_neighbors", 10);
  get_parameter("small_gicp_num_neighbors", small_gicp_num_neighbors_);
  declare_parameter("small_gicp_voxel_resolution", 1.0);
  get_parameter("small_gicp_voxel_resolution", small_gicp_voxel_resolution_);
  declare_parameter("small_gicp_registration_type", "GICP");
  get_parameter("small_gicp_registration_type", small_gicp_registration_type_);
  declare_parameter("small_gicp_max_correspondence_distance", 1.0);
  get_parameter("small_gicp_max_correspondence_distance", small_gicp_max_correspondence_distance_);
  declare_parameter("loop_detection_period", 1000);
  get_parameter("loop_detection_period", loop_detection_period_);
  declare_parameter("threshold_loop_closure_score", 1.0);
  get_parameter("threshold_loop_closure_score", threshold_loop_closure_score_);
  declare_parameter("distance_loop_closure", 20.0);
  get_parameter("distance_loop_closure", distance_loop_closure_);
  declare_parameter("range_of_searching_loop_closure", 20.0);
  get_parameter("range_of_searching_loop_closure", range_of_searching_loop_closure_);
  declare_parameter("search_submap_num", 3);
  get_parameter("search_submap_num", search_submap_num_);
  declare_parameter("num_adjacent_pose_constraints", 1);
  get_parameter("num_adjacent_pose_constraints", num_adjacent_pose_constraints_);
  declare_parameter("use_save_map_in_loop", false);
  get_parameter("use_save_map_in_loop", use_save_map_in_loop_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);
  declare_parameter("headless_debug_flag_", false);
  get_parameter("headless_debug_flag_", headless_debug_flag_);
  declare_parameter("use_scancontext", true);
  get_parameter("use_scancontext", use_scancontext_);
  declare_parameter("use_geometric_verification", true);
  get_parameter("use_geometric_verification", use_geometric_verification_);
  declare_parameter("scancontext_dist_threshold", 0.15);
  get_parameter("scancontext_dist_threshold", scancontext_dist_threshold_);
  declare_parameter("node_spacing_method", "F"); // F - frame D - distance
  get_parameter("node_spacing_method", node_spacing_method_);
  declare_parameter("distance_for_keyframe", 1.0);
  get_parameter("distance_for_keyframe", distance_for_keyframe_);
  declare_parameter("keyframe_interval", 30);
  get_parameter("keyframe_interval", keyframe_interval_);
  declare_parameter("tum_log_unoptimized_filename", "/tmp/tum_trajectory_unoptimized.txt");
  get_parameter("tum_log_unoptimized_filename", tum_log_unoptimized_filename_);
  declare_parameter("tum_log_optimized_filename", "/tmp/tum_trajectory_optimized.txt");
  get_parameter("tum_log_optimized_filename", tum_log_optimized_filename_);
  declare_parameter("loop_edges_log_filename", "/tmp/loop_edges.txt");
  get_parameter("loop_edges_log_filename", loop_edges_log_filename_);
  declare_parameter("map_filename", "map.pcd");
  get_parameter("map_filename", map_filename_);

  std::cout << "registration_method:" << registration_method << std::endl;
  std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "small_gicp_num_neighbors:" << small_gicp_num_neighbors_ << std::endl;
  std::cout << "small_gicp_voxel_resolution[m]:" << small_gicp_voxel_resolution_ << std::endl;
  std::cout << "small_gicp_registration_type:" << small_gicp_registration_type_ << std::endl;
  std::cout << "small_gicp_max_correspondence_distance[m]:" << small_gicp_max_correspondence_distance_ << std::endl;
  std::cout << "loop_detection_period[Hz]:" << loop_detection_period_ << std::endl;
  std::cout << "threshold_loop_closure_score:" << threshold_loop_closure_score_ << std::endl;
  std::cout << "distance_loop_closure[m]:" << distance_loop_closure_ << std::endl;
  std::cout << "range_of_searching_loop_closure[m]:" << range_of_searching_loop_closure_ <<
    std::endl;
  std::cout << "search_submap_num:" << search_submap_num_ << std::endl;
  std::cout << "num_adjacent_pose_constraints:" << num_adjacent_pose_constraints_ << std::endl;
  std::cout << "use_save_map_in_loop:" << std::boolalpha << use_save_map_in_loop_ << std::endl;
  std::cout << "debug_flag:" << std::boolalpha << debug_flag_ << std::endl;
  std::cout << "headless_debug_flag:" << std::boolalpha << headless_debug_flag_ << std::endl;
  std::cout << "use_scancontext:" << std::boolalpha << use_scancontext_ << std::endl;
  std::cout << "use_geometric_verification:" << std::boolalpha << use_geometric_verification_ << std::endl;
  std::cout << "scancontext_dist_threshold:" << scancontext_dist_threshold_ << std::endl;
  std::cout << "node_spacing_method:" << node_spacing_method_ << std::endl;
  std::cout << "distance_for_keyframe[m]:" << distance_for_keyframe_ << std::endl;
  std::cout << "keyframe_interval:" << keyframe_interval_ << std::endl;

  std::cout << "Scancontext Manager parameters:" << std::endl;
  std::cout << "LIDAR_HEIGHT:" << sc_manager_.LIDAR_HEIGHT << std::endl;
  std::cout << "PC_NUM_RING:" << sc_manager_.PC_NUM_RING << std::endl;
  std::cout << "PC_NUM_SECTOR:" << sc_manager_.PC_NUM_SECTOR << std::endl;
  std::cout << "PC_MAX_RADIUS:" << sc_manager_.PC_MAX_RADIUS << std::endl;
  std::cout << "NUM_EXCLUDE_RECENT:" << sc_manager_.NUM_EXCLUDE_RECENT << std::endl;
  std::cout << "NUM_CANDIDATES_FROM_TREE:" << sc_manager_.NUM_CANDIDATES_FROM_TREE << std::endl;
  std::cout << "SEARCH_RATIO:" << sc_manager_.SEARCH_RATIO << std::endl;
  std::cout << "SC_DIST_THRES:" << sc_manager_.SC_DIST_THRES << std::endl;
  std::cout << "TREE_MAKING_PERIOD_:" << sc_manager_.TREE_MAKING_PERIOD_ << std::endl;

  std::cout << "------------------" << std::endl;

  voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  if (registration_method == "NDT") {
	  boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>
      ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setMaximumIterations(200);
    ndt->setResolution(ndt_resolution);
    // ndt->setTransformationEpsilon(0.01);
    ndt->setTransformationEpsilon(1e-6);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
    registration_ = ndt;
  } else if (registration_method == "GICP") {
	  boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
      gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(0.2);
    gicp->setMaximumIterations(200);
    //gicp->setCorrespondenceRandomness(20);
    gicp->setTransformationEpsilon(1e-6);
    gicp->setEuclideanFitnessEpsilon(1e-6);
    gicp->setRANSACIterations(0);
    registration_ = gicp;
  } else if (registration_method == "SMALL_GICP") {
#ifdef BUILD_WITH_SMALL_GICP_PCL
    boost::shared_ptr<small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>>
      small_gicp_reg(new small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>());
    
    // Configure small_gicp parameters
    small_gicp_reg->setNumThreads(ndt_num_threads);
    small_gicp_reg->setNumNeighborsForCovariance(small_gicp_num_neighbors_);
    small_gicp_reg->setVoxelResolution(small_gicp_voxel_resolution_);
    small_gicp_reg->setRegistrationType(small_gicp_registration_type_);
    small_gicp_reg->setMaxCorrespondenceDistance(small_gicp_max_correspondence_distance_);
    small_gicp_reg->setTransformationEpsilon(1e-6);
    
    registration_ = small_gicp_reg;
    RCLCPP_INFO(get_logger(), "Using Small GICP registration with type: %s", small_gicp_registration_type_.c_str());
#else
    RCLCPP_ERROR(get_logger(), "Small GICP not available. Compile with BUILD_WITH_SMALL_GICP_PCL=ON");
    exit(1);
#endif
  } else {
    RCLCPP_ERROR(get_logger(), "invalid registration_method");
    exit(1);
  }

  initializePubSub();

  // Continues method variables
  map_array2_msg_.header.frame_id = "map";

  // Map save service
  auto map_save_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      const std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
    {
      std::cout << "Received an request to save the map" << std::endl;
      if (initial_map_array_received_ == false) {
        std::cout << "initial map is not received" << std::endl;
        return;
      }
      // doPoseAdjustment(map_array_msg_, true);
      doPoseAdjustment(map_array2_msg_, true);
      
    };

  map_save_srv_ = create_service<std_srvs::srv::Empty>("map_save", map_save_callback);

}

void BackendSlamComponent::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "Initialize Publishers and Subscribers");

  auto submap_callback =
    [this](const typename lidarslam_msgs::msg::SubMap::SharedPtr msg_ptr) -> void
    {
      std::lock_guard<std::mutex> lock(mtx_);
      map_submap_msg_ = *msg_ptr;

      // Process the received submap
      RCLCPP_INFO(get_logger(), "Processing frame:%d", submap_id_);
      RCLCPP_INFO(get_logger(), "Received submap distance:%f", map_submap_msg_.distance);
      lidarslam_msgs::msg::SubMap latest_submap;
      latest_submap = map_submap_msg_;
      Eigen::Affine3d latest_submap_affine;
      tf2::fromMsg(latest_submap.pose, latest_submap_affine);
      pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(latest_submap.cloud, *latest_submap_cloud_ptr);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
      Eigen::Affine3d latest_affine;
      tf2::fromMsg(latest_submap.pose, latest_affine);
      pcl::transformPointCloud(
        *latest_submap_cloud_ptr, *transformed_latest_submap_cloud_ptr,
        latest_affine.matrix().cast<float>());

      // --- FIX: Clean Source Cloud and Check for Empty ---
      pcl::PointCloud<pcl::PointXYZI>::Ptr cleaned_source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      std::vector<int> nan_indices_source;
      pcl::removeNaNFromPointCloud(*transformed_latest_submap_cloud_ptr, *cleaned_source_cloud_ptr, nan_indices_source);

      // CRITICAL CHECK: If the source cloud is empty, stop processing this frame.
      if (cleaned_source_cloud_ptr->points.empty()) {
          RCLCPP_WARN(get_logger(), "Source cloud is empty after NaN filtering. Skipping keyframe processing.");
          return;
      }

      registration_->setInputSource(cleaned_source_cloud_ptr);
      double latest_moving_distance = latest_submap.distance;

      // First submap handling
      if (initial_submap_received_ == false) {
        initial_submap_received_ = true;
        map_array2_msg_.submaps.push_back(map_submap_msg_);
        RCLCPP_INFO(get_logger(), "Number of submaps in map_array2_msg_: %ld", map_array2_msg_.submaps.size());
        sc_manager_.makeAndSaveScancontextAndKeys(*latest_submap_cloud_ptr); 
      }else{
        submap_id_++;
      }

      // Determine if keyframe based on distance moved
      if (node_spacing_method_ == "F") {
        if (submap_id_ % keyframe_interval_ == 0) {
          RCLCPP_INFO(get_logger(), "Keyframe detected by frame interval. Distance moved:%f", latest_moving_distance - previous_submap_distance_);
          previous_submap_distance_ = latest_moving_distance;
        } else {
          return;
        }
      } else if (node_spacing_method_ == "D") {
        if (latest_moving_distance - previous_submap_distance_ >= distance_for_keyframe_) {
          RCLCPP_INFO(get_logger(), "Keyframe detected. Distance moved:%f", latest_moving_distance - previous_submap_distance_);
          previous_submap_distance_ = latest_moving_distance;
        } else {
          return;
        }
      }

      // Store latest submap information to map array
      map_array2_msg_.submaps.push_back(map_submap_msg_); 
      RCLCPP_INFO(get_logger(), "Number of submaps in map_array2_msg_: %ld", map_array2_msg_.submaps.size());

      // Publish unmodified path and map cloud if debug flag is active
      if (headless_debug_flag_) {
        // Publish unmodified path from all submaps
        nav_msgs::msg::Path unmodified_path;
        unmodified_path.header.frame_id = "map";
        unmodified_path.header.stamp = this->get_clock()->now();
        
        // Accumulate all submap clouds and publish unmodified map cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr unmodified_map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (const auto& submap : map_array2_msg_.submaps) {
          // Add pose to path
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.header.frame_id = "map";
          pose_stamped.header.stamp = this->get_clock()->now();
          pose_stamped.pose = submap.pose;
          unmodified_path.poses.push_back(pose_stamped);
          
          // // Transform and accumulate cloud
          // pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
          // pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);
          // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
          // Eigen::Affine3d submap_affine;
          // tf2::fromMsg(submap.pose, submap_affine);
          // pcl::transformPointCloud(*submap_cloud_ptr, *transformed_submap_cloud_ptr, submap_affine.matrix().cast<float>());
          // *unmodified_map_cloud_ptr += *transformed_submap_cloud_ptr;
        }
        
        RCLCPP_INFO(get_logger(), "Publishing unmodified path with %ld poses", unmodified_path.poses.size());
        unmodified_path_pub_->publish(unmodified_path);
        
        // // Publish unmodified map cloud
        // sensor_msgs::msg::PointCloud2 unmodified_map_cloud_msg;
        // pcl::toROSMsg(*unmodified_map_cloud_ptr, unmodified_map_cloud_msg);
        // unmodified_map_cloud_msg.header.frame_id = "map";
        // unmodified_map_cloud_msg.header.stamp = this->get_clock()->now();
        // unmodified_map_cloud_pub_->publish(unmodified_map_cloud_msg);
      }

      // Add cloud to Scan context
      if (use_scancontext_) {
        RCLCPP_INFO(get_logger(), "Adding submap to Scancontext");
        sc_manager_.makeAndSaveScancontextAndKeys(*latest_submap_cloud_ptr);
      }

      // Setup for loop closure detection
      int num_submaps = map_array2_msg_.submaps.size();
      double min_fitness_score = std::numeric_limits<double>::max();
      bool is_candidate = false;
      int id_min = 0;
      double min_dist = std::numeric_limits<double>::max();
      double sc_yaw_difference_ = 0.0;
      lidarslam_msgs::msg::SubMap min_submap;
      Eigen::Vector3d latest_submap_pos{
        latest_submap.pose.position.x,
        latest_submap.pose.position.y,
        latest_submap.pose.position.z
      };

      // Radius search based loop detection (Can integrate a kd-tree to make more efficient)
      if (use_geometric_verification_) {
        for (int i = 0; i < num_submaps; i++) {
          auto submap = map_array2_msg_.submaps[i];
          Eigen::Vector3d submap_pos{submap.pose.position.x, submap.pose.position.y,
            submap.pose.position.z};
          double dist = (latest_submap_pos - submap_pos).norm();
          if (latest_moving_distance - submap.distance > distance_loop_closure_ &&
            dist < range_of_searching_loop_closure_)
          {
            is_candidate = true;
            if (dist < min_dist) {
              id_min = i;
              min_dist = dist;
              min_submap = submap;
              std::cout << "[RS] Candidate found - id:" << i << " Latest id:" << num_submaps - 1 << " distance:" << dist << std::endl;
            }
          }
        }
      }

      // Try Scancontext-based loop detection
      if (use_scancontext_ && !is_candidate) {
        auto detectResult = sc_manager_.detectLoopClosureID();
        int sc_loop_id = detectResult.first;
        float sc_yaw_diff = detectResult.second;
        
        if (sc_loop_id != -1 && sc_loop_id < num_submaps - 1) {
          is_candidate = true;
          id_min = sc_loop_id;
          min_submap = map_array2_msg_.submaps[sc_loop_id];
          Eigen::Vector3d sc_submap_pos{min_submap.pose.position.x, min_submap.pose.position.y, min_submap.pose.position.z}; // Calculate distance for logging
          min_dist = (latest_submap_pos - sc_submap_pos).norm();
          sc_yaw_difference_ = sc_yaw_diff;
          if (debug_flag_) {
            std::cout << "[SC] Candidate found - id:" << sc_loop_id << " Latest id:" << num_submaps - 1 << " distance:" << min_dist << " yaw_diff:" << sc_yaw_diff << std::endl;
          }
        }
      }

      if (is_candidate) {
        RCLCPP_INFO(get_logger(), "Loop closure candidate found - id:%d distance:%f", id_min, min_dist);

        // Combine nearby submaps around the candidate submap for registration target
        pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        for (int j = 0; j <= 2 * search_submap_num_; ++j) {
          if (id_min + j - search_submap_num_ < 0) {continue;}
          auto near_submap = map_array2_msg_.submaps[id_min + j - search_submap_num_];
          pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
          pcl::fromROSMsg(near_submap.cloud, *submap_cloud_ptr);
          pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZI>);
          Eigen::Affine3d affine;
          tf2::fromMsg(near_submap.pose, affine);
          pcl::transformPointCloud(
            *submap_cloud_ptr, *transformed_submap_cloud_ptr,
            affine.matrix().cast<float>());
          *submap_clouds_ptr += *transformed_submap_cloud_ptr;
        }

        // // Apply voxel grid filter to target submap clouds
        // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        // voxelgrid_.setInputCloud(submap_clouds_ptr);
        // voxelgrid_.filter(*filtered_clouds_ptr);
        // registration_->setInputTarget(filtered_clouds_ptr);

        // Apply voxel grid filter to target submap clouds
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        voxelgrid_.setInputCloud(submap_clouds_ptr);
        voxelgrid_.filter(*filtered_clouds_ptr);
        
        // --- FIX: Explicitly remove NaNs from the filtered target cloud and check for emptiness ---
        pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_for_reg(new pcl::PointCloud<pcl::PointXYZI>());
        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud(*filtered_clouds_ptr, *target_cloud_for_reg, nan_indices);
        
        // CRITICAL CHECK: If the target cloud is empty, stop the loop closure attempt.
        if (target_cloud_for_reg->points.empty()) {
            RCLCPP_WARN(get_logger(), "Target cloud for registration (loop closure) is empty after filtering. Skipping loop closure attempt.");
            return;
        }

        registration_->setInputTarget(target_cloud_for_reg);

        // Publish transformed latest submap (source cloud)
        sensor_msgs::msg::PointCloud2 transformed_latest_submap_msg;
        pcl::toROSMsg(*cleaned_source_cloud_ptr, transformed_latest_submap_msg);
        transformed_latest_submap_msg.header.frame_id = "map";
        transformed_latest_submap_msg.header.stamp = this->get_clock()->now();
        transformed_latest_submap_pub_->publish(transformed_latest_submap_msg);

        // Publish filtered target clouds (registration target)
        sensor_msgs::msg::PointCloud2 filtered_clouds_msg;
        pcl::toROSMsg(*filtered_clouds_ptr, filtered_clouds_msg);
        filtered_clouds_msg.header.frame_id = "map";
        filtered_clouds_msg.header.stamp = this->get_clock()->now();
        filtered_clouds_pub_->publish(filtered_clouds_msg);

        // Perform registration
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        if (use_scancontext_ && std::abs(sc_yaw_difference_) > 0.05) {  // 3 degrees threshold
          Eigen::AngleAxisf rotation(-sc_yaw_difference_, Eigen::Vector3f::UnitZ());
          initial_guess.block<3,3>(0,0) = rotation.toRotationMatrix();
          // Note: Untested - might not work as intended
          std::cout << "Using scan context yaw difference: " << sc_yaw_difference_ << " rad (" 
                    << sc_yaw_difference_ * 180.0f / M_PI << " deg)" << std::endl;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        registration_->align(*output_cloud_ptr,  initial_guess);
        double fitness_score = registration_->getFitnessScore();

        // Publish aligned source cloud (result of registration)
        sensor_msgs::msg::PointCloud2 aligned_source_cloud_msg;
        pcl::toROSMsg(*output_cloud_ptr, aligned_source_cloud_msg);
        aligned_source_cloud_msg.header.frame_id = "map";
        aligned_source_cloud_msg.header.stamp = this->get_clock()->now();
        aligned_source_cloud_pub_->publish(aligned_source_cloud_msg);

        // Check if loop closure is valid based on configuration
        bool loop_valid = false;
        if (fitness_score < threshold_loop_closure_score_){
          loop_valid = true;
          RCLCPP_INFO(get_logger(), "Loop closure valid - fitness_score:%f threshold:%f", fitness_score, threshold_loop_closure_score_);
        }else{
          RCLCPP_INFO(get_logger(), "Loop closure invalid - fitness_score:%f threshold:%f", fitness_score, threshold_loop_closure_score_);
        }

        if (loop_valid) {

          // Get poses 
          Eigen::Affine3d init_affine;
          tf2::fromMsg(latest_submap.pose, init_affine);
          Eigen::Affine3d submap_affine;
          tf2::fromMsg(min_submap.pose, submap_affine);

          // Create loop edge
          LoopEdge loop_edge;
          loop_edge.pair_id = std::pair<int, int>(id_min, num_submaps - 1);
          Eigen::Isometry3d from = Eigen::Isometry3d(submap_affine.matrix());
          Eigen::Isometry3d to = Eigen::Isometry3d(
            registration_->getFinalTransformation().cast<double>() * init_affine.matrix());
          loop_edge.relative_pose = Eigen::Isometry3d(from.inverse() * to);
          loop_edges_.push_back(loop_edge);
          
          // Log loop closure information and perform pose adjustment
          std::cout << "---" << std::endl;
          std::cout << "PoseAdjustment distance:" << min_submap.distance << ", score:" << fitness_score << std::endl;
          std::cout << "id_loop_point 1:" << id_min << " id_loop_point 2:" << num_submaps - 1 << std::endl;
          std::cout << "Final transformation:" << std::endl;
          std::cout << "Registration converged: " << registration_->hasConverged() << std::endl;
          std::cout << registration_->getFinalTransformation() << std::endl;
          doPoseAdjustment(map_array2_msg_, use_save_map_in_loop_);

          return;
        }
        std::cout << "min_submap_distance:" << min_submap.distance << " min_fitness_score:" << fitness_score << std::endl;
      }


    };

  map_submap_sub_ =
    create_subscription<lidarslam_msgs::msg::SubMap>(
    "submap", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), submap_callback);

  auto map_array_callback =
    [this](const typename lidarslam_msgs::msg::MapArray::SharedPtr msg_ptr) -> void
    {
      std::lock_guard<std::mutex> lock(mtx_);
      map_array_msg_ = *msg_ptr;
      initial_map_array_received_ = true;
      is_map_array_updated_ = true;
    };

  map_array_sub_ =
    create_subscription<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), map_array_callback);

  // std::chrono::milliseconds period(loop_detection_period_);
  // loop_detect_timer_ = create_wall_timer(
  //   std::chrono::duration_cast<std::chrono::nanoseconds>(period),
  //   std::bind(&GraphBasedSlamComponent::searchLoop, this)
  // );

  modified_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map",
    rclcpp::QoS(10));

  modified_map_array_pub_ = create_publisher<lidarslam_msgs::msg::MapArray>(
    "modified_map_array", rclcpp::QoS(10));

  modified_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "modified_path",
    rclcpp::QoS(10));

  unmodified_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "unmodified_path",
    rclcpp::QoS(10));

  unmodified_map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "unmodified_map_cloud",
    rclcpp::QoS(10));

  transformed_latest_submap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "transformed_latest_submap",
    rclcpp::QoS(10));

  filtered_clouds_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "filtered_clouds",
    rclcpp::QoS(10));

  aligned_source_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "aligned_source_cloud",
    rclcpp::QoS(10));

  loop_edges_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "loop_edges",
    rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "initialization end");


}

void BackendSlamComponent::searchLoop()
{

  // Conditions to run loop detection
  if (initial_map_array_received_ == false) {return;}
  if (is_map_array_updated_ == false) {return;}
  if (map_array_msg_.cloud_coordinate != map_array_msg_.LOCAL) {
    RCLCPP_WARN(get_logger(), "Cloud_coordinate should be local, but it's not local.");
  }
  is_map_array_updated_ = false;

  // Initialise local variables
  lidarslam_msgs::msg::MapArray map_array_msg = map_array_msg_;
  std::lock_guard<std::mutex> lock(mtx_);
  int num_submaps = map_array_msg.submaps.size();
  double min_fitness_score = std::numeric_limits<double>::max();
  double distance_min_fitness_score = 0;
  bool is_candidate = false;
  RCLCPP_INFO(get_logger(), "Searching loop, num_submaps:%d", num_submaps);

  // Get latest submap and add to registration source
  lidarslam_msgs::msg::SubMap latest_submap;
  latest_submap = map_array_msg.submaps[num_submaps - 1];
  Eigen::Affine3d latest_submap_affine;
  tf2::fromMsg(latest_submap.pose, latest_submap_affine);
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(latest_submap.cloud, *latest_submap_cloud_ptr);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Affine3d latest_affine;
  tf2::fromMsg(latest_submap.pose, latest_affine);
  pcl::transformPointCloud(
    *latest_submap_cloud_ptr, *transformed_latest_submap_cloud_ptr,
    latest_affine.matrix().cast<float>());
  registration_->setInputSource(transformed_latest_submap_cloud_ptr);
  
  // Add latest submap to Scancontext manager if enabled
  if (use_scancontext_) {
    sc_manager_.makeAndSaveScancontextAndKeys(*latest_submap_cloud_ptr);
  }
  
  // Radius and distance check to find candidate submaps for loop closure
  double latest_moving_distance = latest_submap.distance;
  Eigen::Vector3d latest_submap_pos{
    latest_submap.pose.position.x,
    latest_submap.pose.position.y,
    latest_submap.pose.position.z};
  int id_min = 0;
  double min_dist = std::numeric_limits<double>::max();
  lidarslam_msgs::msg::SubMap min_submap;
  for (int i = 0; i < num_submaps; i++) {
    auto submap = map_array_msg.submaps[i];
    Eigen::Vector3d submap_pos{submap.pose.position.x, submap.pose.position.y,
      submap.pose.position.z};
    double dist = (latest_submap_pos - submap_pos).norm();
    if (latest_moving_distance - submap.distance > distance_loop_closure_ &&
      dist < range_of_searching_loop_closure_)
    {
      is_candidate = true;
      if (dist < min_dist) {
        id_min = i;
        min_dist = dist;
        min_submap = submap;
      }
    }
  }
  if (debug_flag_) {
    if (is_candidate) {
      std::cout << "[RS] Candidate found - id:" << id_min << " Latest id:" << num_submaps - 1 << " distance:" << min_dist << std::endl;
    } else {
      std::cout << "[RS] No geometric loop candidate found" << std::endl;
    }
  }


  // Try Scancontext-based loop detection if enabled and geometric method didn't find a loop
  if (use_scancontext_ && !is_candidate) {
    auto detectResult = sc_manager_.detectLoopClosureID();
    int sc_loop_id = detectResult.first;
    float sc_yaw_diff = detectResult.second;
    
    if (sc_loop_id != -1 && sc_loop_id < num_submaps - 1) {
      is_candidate = true;
      id_min = sc_loop_id;
      min_submap = map_array_msg.submaps[sc_loop_id];
      Eigen::Vector3d sc_submap_pos{min_submap.pose.position.x, min_submap.pose.position.y, min_submap.pose.position.z}; // Calculate distance for logging
      min_dist = (latest_submap_pos - sc_submap_pos).norm();
      if (debug_flag_) {
        std::cout << "[SC] Candidate found - id:" << sc_loop_id << " Latest id:" << num_submaps - 1 << " distance:" << min_dist << " yaw_diff:" << sc_yaw_diff << std::endl;
      }
    }
  }

  if (is_candidate) {
    RCLCPP_INFO(get_logger(), "Loop closure candidate found - id:%d distance:%f", id_min, min_dist);

    // Combine nearby submaps around the candidate submap for registration target
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (int j = 0; j <= 2 * search_submap_num_; ++j) {
      if (id_min + j - search_submap_num_ < 0) {continue;}
      auto near_submap = map_array_msg.submaps[id_min + j - search_submap_num_];
      pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(near_submap.cloud, *submap_cloud_ptr);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
      Eigen::Affine3d affine;
      tf2::fromMsg(near_submap.pose, affine);
      pcl::transformPointCloud(
        *submap_cloud_ptr, *transformed_submap_cloud_ptr,
        affine.matrix().cast<float>());
      *submap_clouds_ptr += *transformed_submap_cloud_ptr;
    }

    // Apply voxel grid filter to target submap clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid_.setInputCloud(submap_clouds_ptr);
    voxelgrid_.filter(*filtered_clouds_ptr);
    registration_->setInputTarget(filtered_clouds_ptr);

    // Perform registration
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    registration_->align(*output_cloud_ptr);
    double fitness_score = registration_->getFitnessScore();

    // Check if loop closure is valid based on configuration
    bool loop_valid = false;
    if (fitness_score < threshold_loop_closure_score_){
      loop_valid = true;
      RCLCPP_INFO(get_logger(), "Loop closure valid - fitness_score:%f threshold:%f", fitness_score, threshold_loop_closure_score_);
    }else{
      RCLCPP_INFO(get_logger(), "Loop closure invalid - fitness_score:%f threshold:%f", fitness_score, threshold_loop_closure_score_);
    }

    if (loop_valid) {

      // Get poses 
      Eigen::Affine3d init_affine;
      tf2::fromMsg(latest_submap.pose, init_affine);
      Eigen::Affine3d submap_affine;
      tf2::fromMsg(min_submap.pose, submap_affine);

      // Create loop edge
      LoopEdge loop_edge;
      loop_edge.pair_id = std::pair<int, int>(id_min, num_submaps - 1);
      Eigen::Isometry3d from = Eigen::Isometry3d(submap_affine.matrix());
      Eigen::Isometry3d to = Eigen::Isometry3d(
        registration_->getFinalTransformation().cast<double>() * init_affine.matrix());
      loop_edge.relative_pose = Eigen::Isometry3d(from.inverse() * to);
      loop_edges_.push_back(loop_edge);
      
      // Log loop closure information and perform pose adjustment
      std::cout << "---" << std::endl;
      std::cout << "PoseAdjustment distance:" << min_submap.distance << ", score:" << fitness_score << std::endl;
      std::cout << "id_loop_point 1:" << id_min << " id_loop_point 2:" << num_submaps - 1 << std::endl;
      std::cout << "final transformation:" << std::endl;
      std::cout << registration_->getFinalTransformation() << std::endl;
      doPoseAdjustment(map_array_msg, use_save_map_in_loop_);

      return;
    }
    std::cout << "min_submap_distance:" << min_submap.distance << " min_fitness_score:" << fitness_score << std::endl;
  }
}

void BackendSlamComponent::doPoseAdjustment(
  lidarslam_msgs::msg::MapArray map_array_msg,
  bool do_save_map)
{

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
    std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(
    std::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));

  optimizer.setAlgorithm(solver);

  int submaps_size = map_array_msg.submaps.size();
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg.submaps[i].pose, affine);
    Eigen::Isometry3d pose(affine.matrix());

    // Create vertex
    g2o::VertexSE3 * vertex_se3 = new g2o::VertexSE3();
    vertex_se3->setId(i);
    vertex_se3->setEstimate(pose);
    if (i == 0) {vertex_se3->setFixed(true);}
    if (i < num_adjacent_pose_constraints_) {
      vertex_se3->setFixed(true); // This prevents poses adjacent to the fixed origin from shifting when there is a loop to the origin
    }
    optimizer.addVertex(vertex_se3);

    


    /* adjacent pose constraint */
    if (i > num_adjacent_pose_constraints_) {
      for (int j = 0; j < num_adjacent_pose_constraints_; j++) {
        Eigen::Affine3d pre_affine;
        Eigen::fromMsg(
          map_array_msg.submaps[i - num_adjacent_pose_constraints_ + j].pose,
          pre_affine);
        Eigen::Isometry3d pre_pose(pre_affine.matrix());
        Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;
        g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] = optimizer.vertex(i - num_adjacent_pose_constraints_ + j);
        edge_se3->vertices()[1] = optimizer.vertex(i);
        optimizer.addEdge(edge_se3);
      }
    }

  }
  /* loop edge */
  for (auto loop_edge : loop_edges_) {
    g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(loop_edge.relative_pose);
    edge_se3->setInformation(info_mat);
    edge_se3->vertices()[0] = optimizer.vertex(loop_edge.pair_id.first);
    edge_se3->vertices()[1] = optimizer.vertex(loop_edge.pair_id.second);
    optimizer.addEdge(edge_se3);
  }

  /* optimize */
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  optimizer.save("pose_graph.g2o");

  /* modified_map publish */
  std::cout << "Modified_map publish" << std::endl;
  lidarslam_msgs::msg::MapArray modified_map_array_msg;
  modified_map_array_msg.header = map_array_msg.header;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < submaps_size; i++) {
    g2o::VertexSE3 * vertex_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
    Eigen::Affine3d se3 = vertex_se3->estimate();
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map */
    Eigen::Affine3d previous_affine;
    tf2::fromMsg(map_array_msg.submaps[i].pose, previous_affine);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg.submaps[i].cloud, *cloud_ptr);

    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, se3.matrix().cast<float>());
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud_ptr, *cloud_msg_ptr);
    *map_ptr += *transformed_cloud_ptr;

    /* submap */
    lidarslam_msgs::msg::SubMap submap;
    submap.header = map_array_msg.submaps[i].header;
    submap.pose = pose;
    submap.cloud = *cloud_msg_ptr;
    modified_map_array_msg.submaps.push_back(submap);

    /* path */
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = submap.header;
    pose_stamped.pose = submap.pose;
    path.poses.push_back(pose_stamped);

  }

  modified_map_array_pub_->publish(modified_map_array_msg);
  modified_path_pub_->publish(path);

  /* loop edges publish */
  visualization_msgs::msg::MarkerArray loop_edges_markers;
  for (size_t i = 0; i < loop_edges_.size(); ++i) {
    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = this->get_clock()->now();
    edge_marker.ns = "loop_edges";
    edge_marker.id = static_cast<int>(i);
    edge_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Set line properties
    edge_marker.scale.x = 0.04; // Line width
    edge_marker.color.r = 0.0; // Red color
    edge_marker.color.g = 0.0;
    edge_marker.color.b = 1.0;
    edge_marker.color.a = 1.0; // Opaque (no transparency)
    
    // Get optimized poses for the loop edge endpoints
    g2o::VertexSE3 * vertex_from = static_cast<g2o::VertexSE3 *>(optimizer.vertex(loop_edges_[i].pair_id.first));
    g2o::VertexSE3 * vertex_to = static_cast<g2o::VertexSE3 *>(optimizer.vertex(loop_edges_[i].pair_id.second));
    
    Eigen::Affine3d pose_from = vertex_from->estimate();
    Eigen::Affine3d pose_to = vertex_to->estimate();
    
    // Add two points to create a line
    geometry_msgs::msg::Point p1, p2;
    p1.x = pose_from.translation().x();
    p1.y = pose_from.translation().y();
    p1.z = pose_from.translation().z();
    
    p2.x = pose_to.translation().x();
    p2.y = pose_to.translation().y();
    p2.z = pose_to.translation().z();
    
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
    
    loop_edges_markers.markers.push_back(edge_marker);
  }
  
  loop_edges_pub_->publish(loop_edges_markers);

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  modified_map_pub_->publish(*map_msg_ptr);
  
  if (do_save_map) {
    pcl::io::savePCDFileASCII(map_filename_, *map_ptr);
    RCLCPP_INFO(get_logger(), "Saved map to: %s", map_filename_.c_str());
    
    // Log unoptimized trajectory in TUM format
    logTrajectoryInTUMFormat(map_array_msg, tum_log_unoptimized_filename_);
    
    // Log optimized trajectory in TUM format
    logTrajectoryInTUMFormat(modified_map_array_msg, tum_log_optimized_filename_);
    
    // Log loop edges
    logLoopEdges(loop_edges_log_filename_);
    
    RCLCPP_INFO(get_logger(), "Saved trajectories in TUM format:");
    RCLCPP_INFO(get_logger(), "  Unoptimized: %s", tum_log_unoptimized_filename_.c_str());
    RCLCPP_INFO(get_logger(), "  Optimized: %s", tum_log_optimized_filename_.c_str());
    RCLCPP_INFO(get_logger(), "Saved loop edges log: %s", loop_edges_log_filename_.c_str());
  }

}

void BackendSlamComponent::logTrajectoryInTUMFormat(
  const lidarslam_msgs::msg::MapArray& map_array_msg,
  const std::string& filename)
{
  std::ofstream tum_log;
  tum_log.open(filename, std::ios::out);
  
  if (!tum_log.is_open()) {
    RCLCPP_WARN(get_logger(), "Failed to open TUM format log file: %s", filename.c_str());
    return;
  }
  
  // Write header comment
  tum_log << "# TUM format trajectory file" << std::endl;
  tum_log << "# timestamp tx ty tz qx qy qz qw" << std::endl;
  
  // Log each submap pose
  for (const auto& submap : map_array_msg.submaps) {
    double timestamp = submap.header.stamp.sec + submap.header.stamp.nanosec * 1e-9;
    
    tum_log << std::fixed << std::setprecision(6) << timestamp << " "
            << std::setprecision(6) << submap.pose.position.x << " "
            << submap.pose.position.y << " "
            << submap.pose.position.z << " "
            << submap.pose.orientation.x << " "
            << submap.pose.orientation.y << " "
            << submap.pose.orientation.z << " "
            << submap.pose.orientation.w << std::endl;
  }
  
  tum_log.close();
  RCLCPP_INFO(get_logger(), "Logged %zu poses to %s", map_array_msg.submaps.size(), filename.c_str());
}

void BackendSlamComponent::logLoopEdges(const std::string& filename)
{
  std::ofstream loop_log;
  loop_log.open(filename, std::ios::out);
  
  if (!loop_log.is_open()) {
    RCLCPP_WARN(get_logger(), "Failed to open loop edges log file: %s", filename.c_str());
    return;
  }
  
  // Write header comment
  loop_log << "# Loop edges log file" << std::endl;
  loop_log << "# Total loop closures detected: " << loop_edges_.size() << std::endl;
  loop_log << "# Format: from_id to_id tx ty tz qx qy qz qw" << std::endl;
  
  // Log each loop edge
  for (const auto& loop_edge : loop_edges_) {
    Eigen::Affine3d relative_pose_affine(loop_edge.relative_pose.matrix());
    Eigen::Quaterniond quat(relative_pose_affine.rotation());
    Eigen::Vector3d trans = relative_pose_affine.translation();
    
    loop_log << std::fixed << std::setprecision(0) 
             << loop_edge.pair_id.first << " " 
             << loop_edge.pair_id.second << " "
             << std::setprecision(6)
             << trans.x() << " "
             << trans.y() << " "
             << trans.z() << " "
             << quat.x() << " "
             << quat.y() << " "
             << quat.z() << " "
             << quat.w() << std::endl;
  }
  
  loop_log.close();
  RCLCPP_INFO(get_logger(), "Logged %zu loop edges to %s", loop_edges_.size(), filename.c_str());
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(backendslam::BackendSlamComponent)