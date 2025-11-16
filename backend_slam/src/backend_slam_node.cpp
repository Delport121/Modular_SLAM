#include "backend_slam/backend_slam_component.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  rclcpp::spin(std::make_shared<backendslam::BackendSlamComponent>(options));
  rclcpp::shutdown();
  return 0;
}