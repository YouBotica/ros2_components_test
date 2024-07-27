// component_manager_main.cpp

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "my_cpp_package/minimal_publisher.hpp"
#include "my_cpp_package/minimal_subscriber.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto publisher_node = std::make_shared<PointCloudPublisher>(options);
  auto subscriber_node = std::make_shared<PointCloud2Subscriber>(options);
  exec.add_node(publisher_node);
  exec.add_node(subscriber_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

