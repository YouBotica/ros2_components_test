// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_VSOMEIP_VSOMEIP_TRANSPORT_H
#define LUM_DRIVERS_LIDAR_IRIS_VSOMEIP_VSOMEIP_TRANSPORT_H

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <lum_platform_networking_types/i_someip_transport.h>

namespace vsomeip_v3 {
class runtime;
class application;
class message;
} // namespace vsomeip_v3

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// The default name given to the vsomeip application instance
constexpr const char* DEFAULT_VSOMEIP_APPLICATION_NAME{"SensorControlSomeIpTransport"};

class VSomeIpTransport : public lum::platform::networking::ISomeIpTransport
{
public:
  /// @brief Constructor
  /// @param[in] runtime The SOME/IP runtime to use
  /// @param[in] application_name
  explicit VSomeIpTransport(std::shared_ptr<vsomeip_v3::runtime> runtime,
                            std::string application_name = DEFAULT_VSOMEIP_APPLICATION_NAME);

  ~VSomeIpTransport() override;
  VSomeIpTransport(const VSomeIpTransport&) = delete;
  VSomeIpTransport(VSomeIpTransport&&) = delete;
  VSomeIpTransport& operator=(const VSomeIpTransport&) & = delete;
  VSomeIpTransport& operator=(VSomeIpTransport&&) & = delete;

  /// @copydoc ISomeIpTransport::connect
  Result connect() noexcept override;

  /// @copydoc ISomeIpTransport::disconnect
  Result disconnect() noexcept override;

  /// @copydoc ISomeIpTransport::subscribe
  bool subscribe(const std::vector<std::uint16_t>& service_ids,
                 const AvailabilityHandler& handler) noexcept override;

  /// @copydoc ISomeIpTransport::unsubscribe
  bool unsubscribe() noexcept override;

  /// @copydoc ISomeIpTransport::offerService
  bool offerService(const Address& address) noexcept override;

  /// @copydoc ISomeIpTransport::stopOfferingService
  bool stopOfferingService(const Address& address) noexcept override;

  /// @copydoc ISomeIpTransport::requestService
  bool requestService(const Address& address) noexcept override;

  /// @copydoc ISomeIpTransport::releaseService
  bool releaseService(const Address& address) noexcept override;

  /// @copydoc ISomeIpTransport::call
  Result call(const Address& address,
              std::uint16_t method_id,
              const void* request_data,
              std::size_t request_size,
              void* response_data,
              std::size_t response_size) noexcept override;

  /// @copydoc ISomeIpTransport::requestEvents
  bool requestEvents(const Address& address,
                     std::uint16_t event_group_id,
                     const std::vector<std::uint16_t>& event_ids,
                     const EventHandler& handler) noexcept override;

  /// @copydoc ISomeIpTransport::offerEvent
  bool offerEvent(const Address& address,
                  std::uint16_t event_id,
                  std::uint16_t event_group_id) noexcept override;

  /// @copydoc ISomeIpTransport::stopOfferingEvent
  bool stopOfferingEvent(const Address& address, std::uint16_t event_id) noexcept override;

  /// @copydoc ISomeIpTransport::sendEvent
  bool sendEvent(const Address& address,
                 std::uint16_t event_id,
                 const void* event_data,
                 std::size_t event_size) noexcept override;

  /// @copydoc ISomeIpTransport::releaseEvents
  bool releaseEvents(const Address& address) noexcept override;

  /// @brief Configure timeout for registering with library
  /// @param[in] timeout Timeout period in milliseconds
  void setRegistrationTimeout(const std::chrono::milliseconds timeout) noexcept
  {
    registration_timeout_ = timeout;
  }

  /// @brief Configure timeout for transmitting messages
  /// @param[in] timeout Timeout period in milliseconds
  void setTransmissionTimeout(const std::chrono::milliseconds timeout) noexcept
  {
    transmission_timeout_ = timeout;
  }

private:
  // non-virtual for use in dtor
  void closeConnection() noexcept;

  void registerResponseHandler(const Address& address,
                               std::uint16_t method_id,
                               void* response_data,
                               std::size_t response_size) noexcept;
  void unregisterResponseHandler(const Address& address, std::uint16_t method_id) noexcept;

  bool waitForTransmission() noexcept;

  void onResponseReceived(void* response_data,
                          std::size_t response_size,
                          const vsomeip_v3::message* response) noexcept;

  void onEventReceived(const vsomeip_v3::message* event) noexcept;

  AvailabilityHandler handler_{};
  std::vector<std::uint16_t> subscribed_service_ids_{};

  std::shared_ptr<vsomeip_v3::runtime> runtime_;
  std::shared_ptr<vsomeip_v3::application> application_{};
  std::thread event_dispatcher_{};

  static constexpr std::chrono::milliseconds DEFAULT_LIBRARY_REACTION_TIMEOUT{1000};
  std::chrono::milliseconds registration_timeout_{DEFAULT_LIBRARY_REACTION_TIMEOUT};
  bool registration_ok_{false};

  // default: 2x sensor BSW cycle time
  static constexpr std::chrono::milliseconds DEFAULT_NETWORK_TRANSMISSION_TIMEOUT{600};
  std::chrono::milliseconds transmission_timeout_{DEFAULT_NETWORK_TRANSMISSION_TIMEOUT};
  bool transmission_received_{false};
  bool transmission_ok_{false};

  // for synchronization with library callbacks
  std::condition_variable library_operation_condition_{};
  std::mutex library_operation_mutex_{};

  struct EventGroup
  {
    std::uint16_t event_group_id{};
    std::vector<std::uint16_t> event_ids{};
    ISomeIpTransport::EventHandler handler{};
  };

  std::unordered_map<ISomeIpTransport::Address, EventGroup, ISomeIpTransport::AddressHash>
    requested_events_{};
  std::mutex requested_events_mutex_{};

  std::string application_name_; ///< The internal name given to the vsomeip application instance
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum
#endif
