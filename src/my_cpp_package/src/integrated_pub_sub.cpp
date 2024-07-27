// component_manager_main.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto manager = std::make_shared<rclcpp_components::ComponentManager>(executor, "component_manager");
  executor->add_node(manager);

  executor->spin();
  rclcpp::shutdown();
  return 0;
}


// // main.cpp
// #include "rclcpp/rclcpp.hpp
// #include "rclcpp_components/component_manager.hpp"
// #include "minimal_publisher.cpp"
// #include "minimal_subscriber.cpp"

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto options = rclcpp::NodeOptions();
//   auto component_manager = std::make_shared<rclcpp_components::ComponentManager>(options);
//   auto pointcloud_subscriber = component_manager->create_node_instance("PointCloud2Subscriber");
//   auto node_b = component_manager->create_node_instance("PointCloudPublisher");
//   rclcpp::spin(component_manager);
//   rclcpp::shutdown();
//   return 0;
// }
