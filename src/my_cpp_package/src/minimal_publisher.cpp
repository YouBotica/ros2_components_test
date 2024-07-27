#include "my_cpp_package/minimal_publisher.hpp"

PointCloudPublisher::PointCloudPublisher(const rclcpp::NodeOptions & options)
: Node("pointcloud_publisher", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() { publish_pointcloud(); });
}

void PointCloudPublisher::publish_pointcloud()
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

  RCLCPP_INFO(this->get_logger(), "Pointer: %p", &pointcloud_msg);

  publisher_->publish(pointcloud_msg);
}
