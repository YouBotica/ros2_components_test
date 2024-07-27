#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher(const rclcpp::NodeOptions & options)
  : Node("pointcloud_publisher", rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { publish_pointcloud(); });
  }

private:
  void publish_pointcloud()
  {
    auto pointcloud_msg = sensor_msgs::msg::PointCloud2();
    pointcloud_msg.header.stamp = this->now();
    pointcloud_msg.header.frame_id = "map";
    pointcloud_msg.height = 1;
    pointcloud_msg.width = 100;

    // Setting the point cloud data type and fields
    pointcloud_msg.fields.resize(3);
    pointcloud_msg.fields[0].name = "x";
    pointcloud_msg.fields[0].offset = 0;
    pointcloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud_msg.fields[0].count = 1;

    pointcloud_msg.fields[1].name = "y";
    pointcloud_msg.fields[1].offset = 4;
    pointcloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud_msg.fields[1].count = 1;

    pointcloud_msg.fields[2].name = "z";
    pointcloud_msg.fields[2].offset = 8;
    pointcloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud_msg.fields[2].count = 1;

    pointcloud_msg.point_step = 12;
    pointcloud_msg.row_step = 12 * 100;
    pointcloud_msg.is_dense = true;

    pointcloud_msg.data.resize(12 * 100);

    // Fill data
    for (size_t i = 0; i < 100; ++i)
    {
      float* pt = reinterpret_cast<float*>(&pointcloud_msg.data[i * 12]);
      pt[0] = static_cast<float>(i); // x
      pt[1] = 0.0f; // y
      pt[2] = 0.0f; // z
    }

    // RCLCPP_INFO(this->get_logger(), "Pointer: %p", pointcloud_msg);
    std::cout << "Pointer: " << std::endl;

    // std::cout <<
    //   "Published message with address: 0x%", msg->data,
    //   reinterpret_cast<std::uintptr_t>(pointcloud_msg) << std::cin;

    publisher_->publish(pointcloud_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PointCloudPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudPublisher)