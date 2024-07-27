#include "ros2_iris_driver/iris_driver.hpp"

#include <csignal>

#include "yaml-cpp/yaml.h"
#include "lum_platform_interface/lum_platform_interface.h"

namespace ros2_iris_driver {
namespace node {
IrisDriverNode::IrisDriverNode(const rclcpp::NodeOptions& options)
    : Node("iris_driver", options), control_interface_{nullptr} {
    auto yaml_fp = this->declare_parameter<std::string>("yaml_fp", "/home/autera-admin/iris_ws/ros2_iris_driver/ros2_iris_driver/param/car_2.yaml");
    lidar_config_ = YAML::LoadFile(yaml_fp);
    std::cout << "Lidar config loaded" << std::endl;
    for (const auto & sensor_node : lidar_config_["sensors"]) {
        sensor_ids_.insert(sensor_node["sensor_id"].as<uint8_t>());
        auto data_interface = std::make_shared<interface::IrisDataInterface>(sensor_node);
        data_interfaces_[sensor_node["sensor_id"].as<uint8_t>()] = data_interface;
        auto sensor_health_publisher = this->create_publisher<luminar_iris_msgs::msg::SensorHealth>(sensor_node["lidar_name"].as<std::string>() + "/status", rclcpp::SensorDataQoS());
        sensor_health_publishers_[sensor_node["sensor_id"].as<uint8_t>()] = sensor_health_publisher;
        auto pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(sensor_node["lidar_name"].as<std::string>()  + "/points", rclcpp::SensorDataQoS());
        pointcloud_publishers_[sensor_node["sensor_id"].as<uint8_t>()] = pointcloud_publisher;
    }
    lum::platform::initialize();
    control_interface_ = std::make_unique<interface::IrisControlInterface>(lidar_config_, sensor_ids_);
    
    RCLCPP_INFO(this->get_logger(), "Created control interface!");
    
    for (uint8_t sensor_id : sensor_ids_) {
        control_interface_->setSensorHealthCallback([this](luminar_iris_msgs::msg::SensorHealth& sensor_health, uint8_t sensor_id) {
            sensor_health.stamp = this->now();
            sensor_health_publishers_[sensor_id]->publish(sensor_health);
        });
        control_interface_->setScanPattern(sensor_id);
    }

    RCLCPP_INFO(this->get_logger(), "Set scan pattern!");

    for (uint8_t sensor_id : sensor_ids_) {
        control_interface_->setSystemMode(sensor_id, "ACTIVE");
        data_interfaces_[sensor_id]->setPointCloudCallback([this, sensor_id](sensor_msgs::msg::PointCloud2& pointcloud) {
            pointcloud.header.stamp = this->now();
            pointcloud_publishers_[sensor_id]->publish(pointcloud);
        });
    }

    RCLCPP_INFO(this->get_logger(), "Set the lidar to active!");

    sensor_status_thread_ = std::thread([this]() {
        while (rclcpp::ok()) {
            for (uint8_t sensor_id : sensor_ids_) {
                control_interface_->updateSensorHealth(sensor_id);
            }
            RCLCPP_INFO(this->get_logger(), "Sending out PC message!");
            std::this_thread::sleep_for(std::chrono::seconds(lidar_config_["time_bw_sensor_status_calls_sec"].as<uint8_t>()));
        }
    });

    RCLCPP_INFO(this->get_logger(), "Setup pointcloud callback!");
}

void IrisDriverNode::stop() {
    RCLCPP_INFO(this->get_logger(), "Switching to standby");
    for (uint8_t sensor_id : sensor_ids_) {
        control_interface_->setSystemMode(sensor_id, "STANDBY");
    }
    control_interface_->stop();
    for (uint8_t sensor_id : sensor_ids_) {
        data_interfaces_[sensor_id]->stop();
    }
    lum::platform::shutdown();
}

IrisDriverNode::~IrisDriverNode() {
    stop();
}
}  // namespace node
}  // namespace ros2_iris_driver

static void signalHandler(int signum) {
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_iris_driver::node::IrisDriverNode>(rclcpp::NodeOptions());
    signal(SIGINT, signalHandler);
    rclcpp::on_shutdown([&]() {
        node->stop();
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}