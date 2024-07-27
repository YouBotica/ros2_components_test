#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloud2Subscriber : public rclcpp::Node
{
public:
  PointCloud2Subscriber(const rclcpp::NodeOptions & options)
  : Node("pointcloud2_subscriber", rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud",  // Replace with your topic name
      10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
        RCLCPP_INFO(this->get_logger(), "Pointer: %p", pointcloud_msg);
      }
    );
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PointCloud2Subscriber>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloud2Subscriber)
