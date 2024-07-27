#ifndef POINT_CLOUD_PUBLISHER_HPP_
#define POINT_CLOUD_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudPublisher : public rclcpp::Node
{
public:
  explicit PointCloudPublisher(const rclcpp::NodeOptions & options);

private:
  void publish_pointcloud();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudPublisher)

#endif // POINT_CLOUD_PUBLISHER_HPP_
