#include "my_cpp_package/minimal_subscriber.hpp"

PointCloud2Subscriber::PointCloud2Subscriber(const rclcpp::NodeOptions & options)
: Node("pointcloud2_subscriber", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud",  // Replace with your topic name
    10,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
      RCLCPP_INFO(this->get_logger(), "Pointer: %p", pointcloud_msg.get());
    }
  );
}
