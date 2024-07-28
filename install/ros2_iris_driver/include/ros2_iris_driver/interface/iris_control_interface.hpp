// Copyright 2023, Indy Innovation Challenge, Inc. All rights reserved.

#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <yaml-cpp/yaml.h>

#include "lum_drivers_lidar_iris_vsomeip/vsomeip_transport.h"
#include "lum_drivers_lidar_iris_control_interface/i_sensor_control.h"
#include "luminar_iris_msgs/msg/sensor_health.hpp"

#ifndef IRIS_CONTROL_INTERFACE_HPP_
#define IRIS_CONTROL_INTERFACE_HPP_

namespace ros2_iris_driver {
namespace interface {
struct IrisServiceIdentifier {
    uint8_t sensor_id;
    uint16_t service_id;

    bool operator==(const IrisServiceIdentifier& other) const {
        return sensor_id == other.sensor_id && service_id == other.service_id;
    }    
};
}  // namespace interface
}  // namespace ros2_iris_driver

template<>
struct std::hash<ros2_iris_driver::interface::IrisServiceIdentifier> {
    std::size_t operator()(const ros2_iris_driver::interface::IrisServiceIdentifier& k) const {
        return std::hash<uint8_t>()(k.sensor_id) ^ std::hash<uint16_t>()(k.service_id);
    }
};

namespace ros2_iris_driver {
namespace interface {

struct IrisServiceStatus {
    bool has_received_client{false};
    std::chrono::time_point<std::chrono::high_resolution_clock> last_triggered{};
    lum::drivers::lidar::iris::ISensorControl::ClientAHandle service_client;
};

class IrisControlInterface {
public:
    IrisControlInterface(YAML::Node& lidar_config, std::unordered_set<uint8_t> sensor_ids);
    ~IrisControlInterface();
    bool checkForAllServices();
    void setSensorHealthCallback(
        std::function<void(luminar_iris_msgs::msg::SensorHealth&, uint8_t)> callback);
    void start();
    void stop();
    bool setSystemMode(uint8_t sensor_id, const std::string & mode);
    bool setScanPattern(uint8_t sensor_id);
    void updateSensorHealth(uint8_t sensor_id);

private:
    std::shared_ptr<lum::drivers::lidar::iris::VSomeIpTransport> transport_;
    std::unique_ptr<lum::drivers::lidar::iris::ISensorControl> sensor_control_;
    std::thread service_checking_thread_{};
    std::unordered_map<IrisServiceIdentifier, IrisServiceStatus> service_status_;
    std::unordered_map<uint8_t, YAML::Node> sensor_configs_{};
    uint16_t sensor_id_{0};
    YAML::Node lidar_config_{};
    std::unordered_set<uint8_t> sensor_ids_{};
    
    void onSomeipServiceAvailabilityChanged(
        const lum::drivers::lidar::iris::ISensorControl::ClientHandle& received_client,
        const lum::drivers::lidar::iris::ISensorControl::ServiceIds& services);

    void sleepIfNeeded(const IrisServiceIdentifier & service_id);
    bool areEquivalentSystemModes(const lum::drivers::lidar::iris::types::SystemModeCommand cmd,
                   const lum::drivers::lidar::iris::types::SystemModeStatus status);
    std::string modeToString(const lum::drivers::lidar::iris::types::SystemModeStatus& status);
    lum::drivers::lidar::iris::types::SystemModeCommand stringToMode(std::string status);
    bool stop_job_sent_{false};

    std::function<void(luminar_iris_msgs::msg::SensorHealth&,uint8_t)>
        sensor_health_callback_{};
};
}  // namespace interface
}  // namespace ros2_iris_driver

#endif  // IRIS_CONTROL_INTERFACE_HPP_
