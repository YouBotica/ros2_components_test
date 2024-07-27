// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_I_SENSOR_CONTROL_H
#define LUM_DRIVERS_LIDAR_IRIS_I_SENSOR_CONTROL_H

#include <unordered_map>
#include <unordered_set>

#include <lum_common_types_observable/observable.h>
#include <lum_drivers_lidar_iris_internal_types/control_types.h>
#include <lum_platform_networking_types/i_someip_transport.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

class ISensorControl
{
public:
  /// @brief Initialize communications
  /// @return True if connection to service discovery is successfully established
  virtual bool init() noexcept = 0;

  /// @brief Request sensor services
  /// @return True if services have been requested successfully. Subscriptions will start calling
  /// now.
  virtual bool start() noexcept = 0;

  /// @brief Release sensor services
  /// @return True if services have been released successfully.  Subscriptions will stop calling
  /// now.
  virtual bool stop() noexcept = 0;

  /// @brief Shutdown communications
  /// @return True if connection to service discovery is successfully released
  virtual bool shutdown() noexcept = 0;

  /// @brief The base client interface
  class IClient;

  /// @brief Represents the Iris A client interface
  class IClientA;

  /// @brief A reference to a service client
  using ClientHandle = std::shared_ptr<IClient>;

  /// @brief A reference to a service client a
  using ClientAHandle = std::shared_ptr<IClientA>;

  /// @brief a set of service ids
  using ServiceIds = std::unordered_set<std::uint16_t>;

  /// @brief The service availability subscription handle
  using AvailabilitySubscription =
    common::types::observable::SubscriptionHandle<const ClientHandle&, const ServiceIds&>;

  /// @brief The service availability callback function
  using AvailabilityCallback = std::function<void(const ClientHandle&, const ServiceIds&)>;

  /// @brief Subscribe to be notified when the availability status of a service instance changes.
  /// @param[in] callback Handler that will be called when a service instance's availability
  /// changes.
  /// @return The subscription handle
  /// @note User must store the subscription handle or the subscription will be removed.
  /// @note The same client handle is used for all services that belong to a sensor; only the
  /// available services can be interfaced with.
  /// @note when the callback is called, a list of service ids is provided that represents all the
  /// service instances associated with the sensor whose availability changed.
  /// @throw Exceptions from LumPDK
  virtual AvailabilitySubscription
  subscribe(const AvailabilityCallback& callback) noexcept(false) = 0;

  // Polymorphic base class boilerplate
  virtual ~ISensorControl() = default;
  ISensorControl() = default;
  ISensorControl(const ISensorControl&) = delete;
  ISensorControl(ISensorControl&&) = delete;
  void operator=(const ISensorControl&) & = delete;
  void operator=(ISensorControl&&) & = delete;
};

/// Alias of the transport class for easier reading
using ISomeIpTransport = ::lum::platform::networking::ISomeIpTransport;

/// The base client interface
class ISensorControl::IClient
{
public:
  using Factory = std::function<ClientHandle(std::shared_ptr<ISomeIpTransport>, std::uint16_t)>;

  /// @brief a map between service ids and availability statuses
  using ServiceAvailability = std::unordered_map<std::uint16_t, bool>;

  /// @brief a list of SOME/IP addresses
  using SomeIpAddresses =
    std::unordered_set<ISomeIpTransport::Address, ISomeIpTransport::AddressHash>;

  /// @brief Get the sensor id for this client
  /// @return The id that belongs to the sensor associated with this client
  virtual std::uint16_t getSensorId() const noexcept = 0;

  /// @brief Gets the address for the service instance that belongs to this sensor client with the
  /// given service id
  /// @param[in] service_id the service id to query
  /// @return the address for the service instance
  /// @note If there is no applicable service instance associated with this client, then the
  /// instance id returned will be ANY_INSTANCE, i.e 0xFFFF
  virtual ISomeIpTransport::Address getServiceAddress(std::uint16_t service_id) const noexcept = 0;

  /// @brief Gets all the addresses of the service instances that belongs to this sensor client
  /// @return a list of service addresses belonging to this sensor client
  virtual SomeIpAddresses getServiceAddresses() const noexcept = 0;

  /// @brief Get the availability statuses of all services known to this client
  /// @return a map of service ids to availability statuses for this sensor client
  virtual ServiceAvailability getServiceAvailability() const noexcept = 0;

  /// @brief Get whether this client has at least one available service
  /// @return True if at least one service is available
  virtual bool hasAvailableService() const noexcept = 0;

  /// @brief Whether the service instance with the given service id is available for this sensor
  /// @param[in] service_id the service id to query
  /// @return True if the service is available
  virtual bool isAvailable(std::uint16_t service_id) const noexcept = 0;

  /// @brief Mark the service instance with the given address as available or unavailable
  /// Event data is only updated if available.
  /// @param[in] address the address for the service instance
  /// @param[in] is_available the availability status to set
  /// @return True if the service instance's state changed
  virtual bool setAvailable(const ISomeIpTransport::Address& address,
                            bool is_available) noexcept = 0;

  /// @brief Mark all service instances for this sensor as available or unavailable
  /// Event data is only updated if available.
  /// @param[in] is_available the availability status to set
  /// @return a list of all the services that were updated
  virtual ServiceIds setAllAvailable(bool is_available) noexcept = 0;

  /// @brief Reset all event data to default initial values
  virtual void resetEventData() noexcept = 0;

  // Polymorphic base class boilerplate
  virtual ~IClient() = default;
  IClient() = default;
  IClient(const IClient&) = delete;
  IClient(IClient&&) = delete;
  IClient& operator=(const IClient&) & = delete;
  IClient& operator=(IClient&&) & = delete;
};

/// Represents the Iris A client interface
class ISensorControl::IClientA
{
public:
  /// @brief Set the scan datum on the sensor
  /// @param[in] request The scan datum to set
  /// @return True if the request has been acknowledged by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool setScanDatum(const types::ScanDatum& request) noexcept = 0;

  /// @brief Get the scan datum from the sensor
  /// @param[out] response The obtained scan datum
  /// @return True if the response has been delivered by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool getScanDatum(types::ScanDatum& response) const noexcept = 0;

  /// @brief Get the scan datum event data
  /// @return Latest local cached scan datum data received from sensor
  virtual const types::ScanDatum& getScanDatumEvent() const noexcept = 0;

  /// @brief Set the desired scan settings on the sensor
  /// @param[in] request The desired scan settings to set
  /// @return True if the request has been acknowledged by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool setDesiredScanSettings(const types::DesiredScanSettings& request) noexcept = 0;

  /// @brief Get the desired scan settings from the sensor
  /// @param[out] response The obtained desired scan settings
  /// @return True if the response has been delivered by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool getDesiredScanSettings(types::DesiredScanSettings& response) const noexcept = 0;

  /// @brief Get the achieved scan settings from the sensor
  /// @param[out] response The obtained achieved scan settings
  /// @return True if the response has been delivered by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool getAchievedScanSettings(types::AchievedScanSettings& response) const noexcept = 0;

  /// @brief Get the achieved scan settings event data
  /// @return Latest local cached achieved scan settings data received from sensor
  virtual const types::AchievedScanSettings& getAchievedScanSettingsEvent() const noexcept = 0;

  /// @brief Set the scan pattern on the sensor
  /// @param[in] request The scan pattern to set
  /// @return True if the request has been acknowledged by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool setScanPattern(const types::ScanPattern& request) noexcept = 0;

  /// @brief Get the scan pattern command status from the sensor
  /// @param[out] response The obtained scan pattern command status
  /// @return True if the response has been delivered by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool
  getScanPatternCommandStatus(types::ScanPatternCommandStatus& response) const noexcept = 0;

  /// @brief Get the scan pattern command status event data
  /// @return Latest local cached scan pattern command status data received from sensor
  virtual const types::ScanPatternCommandStatus&
  getScanPatternCommandStatusEvent() const noexcept = 0;

  /// @brief Set the desired system mode on the sensor
  /// @param[in] request The system mode to set
  /// @return True if the request has been acknowledged by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool setSystemMode(const types::SystemModeCommand& request) noexcept = 0;

  /// @brief Get the system mode status from the sensor
  /// @param[out] response The obtained system mode status
  /// @return True if the response has been delivered by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool getSystemModeStatus(types::SystemModeStatus& response) const noexcept = 0;

  /// @brief Get the system mode status event data
  /// @return Latest local cached system mode status data received from sensor
  virtual const types::SystemModeStatus& getSystemModeStatusEvent() const noexcept = 0;

  /// @brief Get the sensor health status event data
  /// @return Latest local cached sensor health status data received from sensor
  virtual const types::SensorHealthStatus& getSensorHealthStatusEvent() const noexcept = 0;

  /// @brief Get the software version strings from the sensor
  /// @param[out] response The obtained software versions
  /// @return True if the response has been delivered by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool getSensorSoftwareVersion(types::SensorSoftwareVersion& response) const noexcept = 0;

  /// @brief Get the datapath destination info for the sensor, including the ip address and
  /// port
  /// @param[out] response The obtained datapath destination info
  /// @return True if the response has been delivered by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool
  getDatapathDestinationInfo(types::DatapathDestinationInfo& response) const noexcept = 0;

  /// @brief Set the datapath destination info for the sensor, including the ip address and
  /// port
  /// @param[out] request The desired datapath destination info
  /// @return True if the request has been acknowledged by the sensor
  /// @note The availability of this call depends on the service this client is connected to
  virtual bool
  setDatapathDestinationInfo(const types::DatapathDestinationInfo& request) noexcept = 0;

  // Polymorphic base class boilerplate
  virtual ~IClientA() = default;
  IClientA() = default;
  IClientA(const IClientA&) = delete;
  IClientA(IClientA&&) = delete;
  IClientA& operator=(const IClientA&) & = delete;
  IClientA& operator=(IClientA&&) & = delete;
};

namespace detail {
/// Factory function for creating a concrete instantiation of ISensorControl::IClient with Iris A
/// functionality.
/// @param[in] transport The SOME/IP transport to use
/// @param[in] sensor_id The unique id associated with this sensor
/// @return A concrete ISensorControl::IClient if @a transport is valid, or null
/// if not valid or if memory is exhausted
ISensorControl::ClientHandle makeSensorControlClientA(std::shared_ptr<ISomeIpTransport> transport,
                                                      std::uint16_t sensor_id) noexcept;
} // namespace detail

/// Factory function for creating a concrete instantiation of ISensorControl
/// @param[in] transport The SOME/IP transport to use
/// @param[in] client_factory The creator of the client object(s)
/// @param[in] service_ids the SOME/IP service ids that the sensor control will attempt to discover
/// @return A concrete ISensorControl if both @a transport and @a client_factory are valid, or null
/// if parameters are not valid or if memory is exhausted
std::unique_ptr<ISensorControl> makeSensorControl(std::shared_ptr<ISomeIpTransport> transport,
                                                  ISensorControl::IClient::Factory client_factory,
                                                  ISensorControl::ServiceIds service_ids) noexcept;

/// Factory function for creating a concrete instantiation of ISensorControl for discovering Iris A
/// sensors
/// @param[in] transport The SOME/IP transport to use
/// @return A concrete ISensorControl if @a transport is valid, or null if the parameter is invalid
/// or if memory is exhausted
std::unique_ptr<ISensorControl>
makeSensorControlA(std::shared_ptr<ISomeIpTransport> transport) noexcept;

/// @brief Returns a list of ids for all the SOME/IP services that the Iris A sensor provides.
/// @return the list of service ids
std::unordered_set<std::uint16_t> getIrisAServiceIds() noexcept;

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum
#endif
