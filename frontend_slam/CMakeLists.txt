cmake_minimum_required(VERSION 3.5)
project(frontend_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wextra -Wpedantic)
endif()

#SET(CMAKE_BUILD_TYPE "RELEASE")
SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

# Try to find small_gicp
find_package(small_gicp REQUIRED)

set(dependencies
  rclcpp 
  rclcpp_components 
  tf2_ros 
  tf2_geometry_msgs 
  tf2_sensor_msgs 
  tf2_eigen 
  pcl_conversions
  geometry_msgs 
  sensor_msgs
  nav_msgs
  lidarslam_msgs
  ndt_omp_ros2
)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(frontend_slam_component SHARED
src/frontend_slam_component.cpp
)

target_compile_definitions(frontend_slam_component PRIVATE "GS_SM_BUILDING_DLL")

ament_target_dependencies(frontend_slam_component ${dependencies})

# Remove the copy component library as we're using the main one
# (copy component library removed)

ament_target_dependencies(frontend_slam_component ${dependencies})

add_executable(frontend_slam_node
src/frontend_slam_node.cpp
)

target_link_libraries(frontend_slam_node
frontend_slam_component ${PCL_LIBRARIES})

ament_target_dependencies(frontend_slam_node
  rclcpp)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../small_gicp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/small_gicp/include
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# Link small_gicp if found
if(small_gicp_FOUND)
    target_link_libraries(frontend_slam_component small_gicp::small_gicp)
    target_compile_definitions(frontend_slam_component PRIVATE BUILD_WITH_SMALL_GICP_PCL)
    message(STATUS "small_gicp found and linked")
else()
    message(WARNING "small_gicp not found as package, using direct include")
endif()

rclcpp_components_register_nodes(frontend_slam_component 
  "frontendslam::FrontendSlamComponent")

ament_export_libraries(frontend_slam_component)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  frontend_slam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  frontend_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
