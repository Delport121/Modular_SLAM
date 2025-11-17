#include <frontend_slam/frontend_slam_component.h>
#include <backend_slam/backend_slam_component.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  rclcpp::executors::MultiThreadedExecutor exec;

  auto frontend = std::make_shared<frontendslam::FrontendSlamComponent>(options);
  exec.add_node(frontend);
  auto backend = std::make_shared<backendslam::BackendSlamComponent>(options);
  exec.add_node(backend);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
