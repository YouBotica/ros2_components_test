#include "ros2_iris_driver/interface/iris_control_interface.hpp"

#include <cstdlib>
#include <iostream>
#include <vector>

#include "lum_platform_networking_types/i_someip_transport.h"
#include "lum_platform_interface/lum_platform_interface.h"
#include "lum_platform_networking_types/i_someip_transport.h"
#include "lum_platform_signal/signal.h"
#include "vsomeip/vsomeip.hpp"

namespace ros2_iris_driver {
namespace interface {

IrisControlInterface::IrisControlInterface(YAML::Node& lidar_config, std::unordered_set<uint8_t> sensor_ids) : transport_{nullptr}, sensor_control_{nullptr}, lidar_config_{lidar_config}, sensor_ids_{sensor_ids} {
    setenv("VSOMEIP_CONFIGURATION",
           lidar_config["vsomeip_config"].as<std::string>().c_str(), 1);
    transport_ = std::make_unique<lum::drivers::lidar::iris::VSomeIpTransport>(vsomeip_v3::runtime::get());
    std::cout << "In control interface" << std::endl;
    sensor_control_ = lum::drivers::lidar::iris::makeSensorControlA(transport_);
    if (!sensor_control_->init()) {
        throw std::runtime_error{"Failed to initialize sensor control!"};
    }
    const auto availability_sub = sensor_control_->subscribe(std::bind(&IrisControlInterface::onSomeipServiceAvailabilityChanged, this, std::placeholders::_1, std::placeholders::_2));
    if (!sensor_control_->start()) {
        throw std::runtime_error{"Failed to start sensor control!"};
    }
    unsetenv("VSOMEIP_CONFIGURATION");
    for (uint8_t sensor_id : sensor_ids_) {
        IrisServiceIdentifier service_identifier;
        service_identifier.sensor_id = sensor_id;

        service_identifier.service_id = lum::drivers::lidar::iris::types::SCAN_PATTERN_SERVICE_ID;
        service_status_[service_identifier] = IrisServiceStatus{};
        service_status_[service_identifier].last_triggered = std::chrono::high_resolution_clock::now();

        service_identifier.service_id = lum::drivers::lidar::iris::types::SYSTEM_MODE_SERVICE_ID;
        service_status_[service_identifier] = IrisServiceStatus{};
        service_status_[service_identifier].last_triggered = std::chrono::high_resolution_clock::now();

        service_identifier.service_id = lum::drivers::lidar::iris::types::SYSTEM_INFO_SERVICE_ID;
        service_status_[service_identifier] = IrisServiceStatus{};
        service_status_[service_identifier].last_triggered = std::chrono::high_resolution_clock::now();
    }
    for (const auto & sensor_node : lidar_config_["sensors"]) {
      YAML::Node yaml_node = sensor_node;
      sensor_configs_[sensor_node["sensor_id"].as<uint8_t>()] = yaml_node;
    }
    service_checking_thread_ = std::thread(&IrisControlInterface::start, this);
    service_checking_thread_.join();
}

void IrisControlInterface::start()
{
  while (!checkForAllServices()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void IrisControlInterface::stop() {
  sensor_control_->stop();
    std::cout << "Closed control sensor" << std::endl;
}

IrisControlInterface::~IrisControlInterface() {
  stop();
  sensor_control_.reset();
}

void IrisControlInterface::setSensorHealthCallback(
    std::function<void(luminar_iris_msgs::msg::SensorHealth&, uint8_t)>
        sensor_health_callback) {
  sensor_health_callback_ = sensor_health_callback;
}

void IrisControlInterface::onSomeipServiceAvailabilityChanged(const lum::drivers::lidar::iris::ISensorControl::ClientHandle& received_client,
                                        const lum::drivers::lidar::iris::ISensorControl::ServiceIds& services)
{
  // Save the recieved client into our variable shared across threads
  const auto someip_client =
    std::dynamic_pointer_cast<lum::drivers::lidar::iris::ISensorControl::IClientA>(received_client);

  if (nullptr == someip_client)
  {
    throw std::runtime_error{"Failed to represent the Iris A client interface!"};
  }

  // Process all services that became available under sensor ID
  const auto instance_id = received_client->getSensorId();
  if (sensor_ids_.find(instance_id) != sensor_ids_.end()) {
    for (const auto& service : services)
    {
      if (received_client->isAvailable(service))
      {
        IrisServiceIdentifier service_identifier;
        service_identifier.sensor_id = instance_id;
        service_identifier.service_id = service;
        service_status_[service_identifier].has_received_client = true;
        service_status_[service_identifier].service_client = someip_client;
        std::cout << "Service " << service << " is available on sensor ID " << instance_id << std::endl;
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void IrisControlInterface::updateSensorHealth(uint8_t sensor_id)
{
  if (!sensor_health_callback_) {
    return;
  }
  IrisServiceIdentifier service_identifier;
  service_identifier.sensor_id = sensor_id;
  service_identifier.service_id = lum::drivers::lidar::iris::types::SYSTEM_INFO_SERVICE_ID;
  if (!service_status_[service_identifier].has_received_client) {
    return;
  }
  auto service_client = service_status_[service_identifier].service_client;
  sleepIfNeeded(service_identifier);
  const auto sensor_health = service_client->getSensorHealthStatusEvent();
  luminar_iris_msgs::msg::SensorHealth sensor_health_msg;
  sensor_health_msg.battery_voltage = sensor_health.battery_voltage; ;
  sensor_health_msg.system_voltage = sensor_health.system_voltage;
  sensor_health_msg.system_temperature = sensor_health.system_temperature.inCelsius();
  sensor_health_msg.system_mode = static_cast<uint8_t>(sensor_health.system_mode);
  sensor_health_msg.system_ok = sensor_health.system_ok;
  sensor_health_msg.laser_ok = sensor_health.laser_ok;
  sensor_health_msg.scanner_ok = sensor_health.scanner_ok;
  sensor_health_msg.receiver_ok = sensor_health.receiver_ok;
  sensor_health_msg.datapath_ok = sensor_health.datapath_ok;
  sensor_health_callback_(sensor_health_msg, sensor_id);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool IrisControlInterface::setSystemMode(uint8_t sensor_id, const std::string & mode)
{
  IrisServiceIdentifier service_identifier;
  service_identifier.sensor_id = sensor_id;
  service_identifier.service_id = lum::drivers::lidar::iris::types::SYSTEM_MODE_SERVICE_ID;
  if (!service_status_[service_identifier].has_received_client) {
    return false;
  }
  auto service_client = service_status_[service_identifier].service_client;
  sleepIfNeeded(service_identifier);
  lum::drivers::lidar::iris::types::SystemModeStatus starting_mode;
  lum::drivers::lidar::iris::types::SystemModeCommand desired_mode;
  desired_mode = stringToMode(mode);
  if (!service_client->getSystemModeStatus(starting_mode))
  {
    std::cout << "Failed to get start system mode!" << std::endl;
    return false;
  }

  if (areEquivalentSystemModes(desired_mode, starting_mode))
  {
    std::cout << "Equivalent system modes" << std::endl;
    return true;
  }

  // Note: This mode change takes a moment to activate and won't be reflected immediately by a
  // call to getSystemMode()
  if (!service_client->setSystemMode(desired_mode))
  {
    std::cout << "Failed to set desired mode!" << std::endl;
    return false;
  }
  std::cout << "Desired mode set" << std::endl;
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool IrisControlInterface::setScanPattern(uint8_t sensor_id)
{
  IrisServiceIdentifier service_identifier;
  service_identifier.sensor_id = sensor_id;
  service_identifier.service_id = lum::drivers::lidar::iris::types::SCAN_PATTERN_SERVICE_ID;
  if (!service_status_[service_identifier].has_received_client) {
    return false;
  }
  auto service_client = service_status_[service_identifier].service_client;
  sleepIfNeeded(service_identifier);
  lum::drivers::lidar::iris::types::DesiredScanSettings settings;
  if (!service_client->getDesiredScanSettings(settings)) {
    std::cout << "Failed to get desired scan settings!" << std::endl;
    return false;
  }
  lum::drivers::lidar::iris::types::ScanDatum scan_datum;
  if (!service_client->getScanDatum(scan_datum)) {
    std::cout << "Failed to get scan datum!" << std::endl;
    return false;
  }
  lum::drivers::lidar::iris::types::ScanPattern scan_pattern;
  // Check if frame rate exists in lidar config
  if (sensor_configs_[sensor_id]["frame_rate"])
  {
    settings.frame_rate = lum::common::types::units::FrequencyInHertz<float>(20.0f);
    std::cout << "Changing frame rate to " << sensor_configs_[sensor_id]["frame_rate"].as<float>() << std::endl;
  }
  if (sensor_configs_[sensor_id]["horizontal_interlace_mode"]) {
    const auto horizontal_interlace_mode = sensor_configs_[sensor_id]["horizontal_interlace_mode"].as<std::string>();
    if (horizontal_interlace_mode == "off") {
      settings.horizontal_interlace_mode = lum::drivers::lidar::iris::types::ScanHorizontalInterlaceMode::OFF;
    } else if (horizontal_interlace_mode == "half") {
      settings.horizontal_interlace_mode = lum::drivers::lidar::iris::types::ScanHorizontalInterlaceMode::ONE_HALF;
    } else if (horizontal_interlace_mode == "third") {
      settings.horizontal_interlace_mode = lum::drivers::lidar::iris::types::ScanHorizontalInterlaceMode::ONE_THIRD;
    } else if (horizontal_interlace_mode == "quarter") {
      settings.horizontal_interlace_mode = lum::drivers::lidar::iris::types::ScanHorizontalInterlaceMode::ONE_QUARTER;
    } else if (horizontal_interlace_mode == "eighth") {
      settings.horizontal_interlace_mode = lum::drivers::lidar::iris::types::ScanHorizontalInterlaceMode::ONE_EIGHTH;
    }
    std::cout << "Changing horizontal interlace mode to " << sensor_configs_[sensor_id]["horizontal_interlace_mode"].as<std::string>() << std::endl;
  }
  if (sensor_configs_[sensor_id]["scan_control_line"]) {
    const auto scan_control_line = sensor_configs_[sensor_id]["scan_control_line"].as<std::string>();
    if (scan_control_line == "a") {
      settings.control_line = lum::drivers::lidar::iris::types::ScanControlLine::LINE_A;
    } else if (scan_control_line == "b") {
      settings.control_line = lum::drivers::lidar::iris::types::ScanControlLine::LINE_B;
    }
    std::cout << "Changing scan control line to " << sensor_configs_[sensor_id]["scan_control_line"].as<std::string>() << std::endl;
  }
  if (sensor_configs_[sensor_id]["scan_fill_mode"]) {
    const auto scan_fill_mode = sensor_configs_[sensor_id]["scan_fill_mode"].as<std::string>();
    if (scan_fill_mode == "uniform") {
      settings.fill_mode = lum::drivers::lidar::iris::types::ScanFillMode::UNIFORM;
    } else if (scan_fill_mode == "min_power") {
      settings.fill_mode = lum::drivers::lidar::iris::types::ScanFillMode::MIN_POWER;
    }
    std::cout << "Changing scan fill mode to " << sensor_configs_[sensor_id]["scan_fill_mode"].as<std::string>() << std::endl;
  }
  if (sensor_configs_[sensor_id]["elevation"]) {
    scan_datum.elevation = lum::common::types::units::AngleInDegrees<float>(sensor_configs_[sensor_id]["elevation"].as<float>());
    std::cout << "Changing elevation to " << sensor_configs_[sensor_id]["elevation"].as<float>() << std::endl;
  }

  if (sensor_configs_[sensor_id]["scan_pattern"]) {
    bool scan_pattern_changed = false;
    for (YAML::const_iterator it=sensor_configs_[sensor_id]["scan_pattern"].begin(); it!=sensor_configs_[sensor_id]["scan_pattern"].end(); ++it) {
      auto cmd = it->first.as<std::string>();
      if (cmd == "point") {
        auto point = it->second.as<std::vector<float>>();
        scan_pattern.addPointCommand(lum::common::types::units::AngleInDegrees<float>(point[0]),
                                     lum::common::types::units::AngleInDegrees<float>(point[1]),
                                     static_cast<std::uint8_t>(point[2]));
        scan_pattern_changed = true;
      } else if (cmd == "minlines") {
        scan_pattern.addMinLinesCommand(it->second.as<float>());
        scan_pattern_changed = true;
      } else if (cmd == "snap") {
        scan_pattern.addSnapCommand();
        scan_pattern_changed = true;
      }
    }
    if (!scan_pattern_changed) {
      std::cout << "Scan pattern did not change" << std::endl;
      return true;
    }
    if (!service_client->setDesiredScanSettings(settings))// || !service_client->setScanDatum(scan_datum))
    {
      std::cout << "Failed to set scan pattern!" << std::endl;
      return false;
    }
    std::cout << "Scan pattern set" << std::endl;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool IrisControlInterface::checkForAllServices()
{
  // Exit if all services have executed once
  if (std::all_of(std::begin(service_status_), std::end(service_status_), [](const auto& kv) {
        return kv.second.has_received_client;
      }))
  {
    std::cout << "All services available!" << std::endl;
    return true;
  }
  std::cout << "A service not available!" << std::endl;
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Services need a little time to startup; even when isAvailable() is true, the methods may not
// yet be callable. We sleep for time to wait for service availability, but one could implement a
// more sophisticated sleep-retry loop if desired.
void IrisControlInterface::sleepIfNeeded(const IrisServiceIdentifier & service_id)
{
  const auto found_service_state = service_status_.find(service_id);
  if (found_service_state == std::end(service_status_))
  {
    std::cout << "Unknown service requested: " << service_id.service_id << ". Terminating." << std::endl;
    std::terminate();
  }
  auto current_time = std::chrono::high_resolution_clock::now();
  auto diff = current_time - found_service_state->second.last_triggered;
  auto thresh = std::chrono::seconds(lidar_config_["time_bw_service_calls"].as<int32_t>());
  if (diff < thresh) {
    found_service_state->second.last_triggered = current_time;
    std::this_thread::sleep_for(thresh - diff);
  }
}

bool IrisControlInterface::areEquivalentSystemModes(const lum::drivers::lidar::iris::types::SystemModeCommand cmd,
                   const lum::drivers::lidar::iris::types::SystemModeStatus status)
{
  return (lum::drivers::lidar::iris::types::SystemModeCommand::ACTIVE == cmd &&
          lum::drivers::lidar::iris::types::SystemModeStatus::ACTIVE == status) ||
         (lum::drivers::lidar::iris::types::SystemModeCommand::STANDBY == cmd &&
          lum::drivers::lidar::iris::types::SystemModeStatus::STANDBY == status) ||
         (lum::drivers::lidar::iris::types::SystemModeCommand::SHUTDOWN == cmd &&
          lum::drivers::lidar::iris::types::SystemModeStatus::SHUTDOWN == status);
}

std::string IrisControlInterface::modeToString(const lum::drivers::lidar::iris::types::SystemModeStatus& status)
{
  switch (status)
  {
  case lum::drivers::lidar::iris::types::SystemModeStatus::ACTIVE:
    return "ACTIVE";
  case lum::drivers::lidar::iris::types::SystemModeStatus::STANDBY:
    return "STANDBY";
  case lum::drivers::lidar::iris::types::SystemModeStatus::SHUTDOWN:
    return "SHUTDOWN";
  case lum::drivers::lidar::iris::types::SystemModeStatus::UNKNOWN:
  default:
    return "UNKNOWN";
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
lum::drivers::lidar::iris::types::SystemModeCommand IrisControlInterface::stringToMode(std::string status)
{
  std::transform(begin(status), end(status), begin(status), [](auto c) {
    return static_cast<decltype(c)>(std::toupper(c));
  });

  if (status == "ACTIVE")
  {
    return lum::drivers::lidar::iris::types::SystemModeCommand::ACTIVE;
  }
  else if (status == "STANDBY")
  {
    return lum::drivers::lidar::iris::types::SystemModeCommand::STANDBY;
  }
  else if (status == "SHUTDOWN")
  {
    return lum::drivers::lidar::iris::types::SystemModeCommand::SHUTDOWN;
  }

  std::cout << "Unknown system mode: " << status << "." << std::endl;
  std::exit(EXIT_FAILURE);
}

}  // namespace interface
}  // namespace ros2_iris_driver
