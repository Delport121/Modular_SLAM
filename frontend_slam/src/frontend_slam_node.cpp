#include "frontend_slam/frontend_slam_component.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  rclcpp::spin(std::make_shared<frontendslam::FrontendSlamComponent>(options));
  rclcpp::shutdown();
  return 0;
}
