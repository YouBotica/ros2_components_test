#ifndef POINT_CLOUD2_SUBSCRIBER_HPP_
#define POINT_CLOUD2_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloud2Subscriber : public rclcpp::Node
{
public:
  explicit PointCloud2Subscriber(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloud2Subscriber)

#endif // POINT_CLOUD2_SUBSCRIBER_HPP_