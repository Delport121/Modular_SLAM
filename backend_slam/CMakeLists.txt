cmake_minimum_required(VERSION 3.5)
project(backend_slam)

# ────────────────────────────────────────────────
# Language Standards
# ────────────────────────────────────────────────
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")
add_compile_definitions(G2O_USE_VENDORED_CERES)

# For catching missing symbols at link time
add_link_options(-Wl,--no-undefined)

# ────────────────────────────────────────────────
# Dependencies
# ────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(PCL REQUIRED)
find_package(g2o REQUIRED COMPONENTS
  g2o::core
  g2o::types_slam3d
  g2o::solver_eigen
)
find_package(OpenMP)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies
  rclcpp 
  rclcpp_components 
  tf2_ros 
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  geometry_msgs 
  sensor_msgs
  visualization_msgs
  nav_msgs
  std_srvs
  lidarslam_msgs
  ndt_omp_ros2
  OpenCV
  cv_bridge
)

# ────────────────────────────────────────────────
# OpenMP
# ────────────────────────────────────────────────
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ────────────────────────────────────────────────
# Component Library
# ────────────────────────────────────────────────
add_library(backend_slam_component SHARED
  src/backend_slam_component.cpp
  ../Thirdparty/scancontext/cpp/module/Scancontext/Scancontext.cpp
)

target_compile_definitions(backend_slam_component PRIVATE "GS_GBS_BUILDING_DLL")

ament_target_dependencies(backend_slam_component ${dependencies})

target_link_libraries(backend_slam_component
  g2o::core
  g2o::types_slam3d
  g2o::solver_eigen
)

# Link small_gicp if found
if(small_gicp_FOUND)
  target_link_libraries(backend_slam_component small_gicp::small_gicp)
  target_compile_definitions(backend_slam_component PRIVATE BUILD_WITH_SMALL_GICP_PCL)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/scancontext/cpp/module/Scancontext
  ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/small_gicp/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rclcpp_components_register_nodes(backend_slam_component 
  "backendslam::BackendSlamComponent"
)

# ────────────────────────────────────────────────
# Node Executable
# ────────────────────────────────────────────────
add_executable(backend_slam_node
  src/backend_slam_node.cpp
)

target_link_libraries(backend_slam_node
  backend_slam_component
)

ament_target_dependencies(backend_slam_node rclcpp)

# ────────────────────────────────────────────────
# Install
# ────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  backend_slam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS 
  backend_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

# ────────────────────────────────────────────────
# Export
# ────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(backend_slam_component)
ament_export_dependencies(${dependencies})

ament_package()
