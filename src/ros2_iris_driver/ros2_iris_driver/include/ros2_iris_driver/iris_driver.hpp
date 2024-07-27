#ifndef IRIS_DRIVER_HPP_
#define IRIS_DRIVER_HPP_

#include <unordered_set>
#include <unordered_map>

#include "ros2_iris_driver/interface/iris_data_interface.hpp"
#include "ros2_iris_driver/interface/iris_control_interface.hpp"

#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "luminar_iris_msgs/msg/sensor_health.hpp"

namespace ros2_iris_driver {
namespace node {
class IrisDriverNode : public rclcpp::Node {
public:
    IrisDriverNode(const rclcpp::NodeOptions& options);
    ~IrisDriverNode();
    void stop();
private:
    std::unordered_set<uint8_t> sensor_ids_;
    std::unordered_map<uint8_t, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_publishers_;
    std::unordered_map<uint8_t, rclcpp::Publisher<luminar_iris_msgs::msg::SensorHealth>::SharedPtr> sensor_health_publishers_;
    std::unordered_map<uint8_t, std::shared_ptr<interface::IrisDataInterface>> data_interfaces_;
    std::unique_ptr<interface::IrisControlInterface> control_interface_;
    std::thread sensor_status_thread_{};
    YAML::Node lidar_config_;
};
}  // namespace node
}  // namespace ros2_iris_driver

#endif  // IRIS_DRIVER_HPP_