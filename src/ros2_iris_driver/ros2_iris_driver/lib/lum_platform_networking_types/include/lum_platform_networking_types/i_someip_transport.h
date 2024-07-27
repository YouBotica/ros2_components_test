// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORKING_TYPES_I_SOMEIP_TRANSPORT_H
#define LUM_PLATFORM_NETWORKING_TYPES_I_SOMEIP_TRANSPORT_H

#include <cstdint>
#include <functional>
#include <vector>

namespace lum {
namespace platform {
namespace networking {

/// @brief Interface for SOME/IP transport implementations
class ISomeIpTransport
{
public:
  static constexpr std::uint16_t ANY_SERVICE{0xFFFFU};  ///< Service wildcard
  static constexpr std::uint16_t ANY_INSTANCE{0xFFFFU}; ///< Instance wildcard
  static constexpr std::uint16_t ANY_EVENT{0xFFFFU};    ///< Event wildcard

  /// @brief Details of a SOME/IP service
  struct Address
  {
    std::uint16_t service_id{ANY_SERVICE};   ///< The service id
    std::uint16_t instance_id{ANY_INSTANCE}; ///< The instance id
  };

  /// @brief Hashing functor to allow using Address as key for unordered associative STL containers
  class AddressHash
  {
  public:
    std::size_t operator()(const Address& address) const noexcept
    {
      return (static_cast<std::size_t>(address.service_id) << 16U) |
             static_cast<std::size_t>(address.instance_id);
    }
  };

  enum class Result : std::uint32_t
  {
    OK,
    TIMEOUT,
    INTERNAL_ERROR
  };

  /// @brief Default constructor initializing the SOME/IP library and claiming resources
  ISomeIpTransport() = default;
  /// @brief Destructor freeing all SOME/IP library resources
  virtual ~ISomeIpTransport() = default;

  /// @brief Deleted copy constructor
  ISomeIpTransport(const ISomeIpTransport&) = delete;
  /// @brief Deleted move constructor
  ISomeIpTransport(ISomeIpTransport&&) = delete;

  /// @brief Deleted copy assignment
  const ISomeIpTransport& operator=(const ISomeIpTransport&) & = delete;
  /// @brief Deleted move assignment
  const ISomeIpTransport& operator=(ISomeIpTransport&&) & = delete;

  /// @brief The availability callback function
  using AvailabilityHandler = std::function<void(const Address&, bool)>;

  /// @brief The event callback function
  using EventHandler =
    std::function<void(std::uint16_t event_id, const void* event_data, std::size_t event_size)>;

  /// @brief Connect to network
  /// @note This operation can block a configurable amount of time.
  /// @return Connection operation result
  virtual Result connect() noexcept = 0;

  /// @brief Disconnect from network
  /// @note This operation can block a configurable amount of time.
  /// @return Connection operation result
  virtual Result disconnect() noexcept = 0;

  /// @brief Subscribe to be notified when service availability changes
  /// @param[in] service_ids The services to subscribe to
  /// @param[in] handler The function to call
  virtual bool subscribe(const std::vector<std::uint16_t>& service_ids,
                         const AvailabilityHandler& handler) noexcept = 0;

  /// @brief Release subscription to not get notified anymore
  virtual bool unsubscribe() noexcept = 0;

  /// @brief Begin offering a service
  /// @param[in] address The service details
  virtual bool offerService(const Address& address) noexcept = 0;

  /// @brief Stop offering a service
  /// @param[in] address The service details
  virtual bool stopOfferingService(const Address& address) noexcept = 0;

  /// @brief Request a service
  /// @param[in] address The service details
  virtual bool requestService(const Address& address) noexcept = 0;

  /// @brief Release a service
  /// @param[in] address The service details
  virtual bool releaseService(const Address& address) noexcept = 0;

  /// @brief Call a method
  /// Request or response data can be null for parameter-less/void return calls.
  /// @param[in] address The service details
  /// @param[in] method_id The method to call
  /// @param[in] request_data Pointer to the method parameters
  /// @param[in] request_size Size of the method parameter structure
  /// @param[inout] response_data Pointer to the method return value
  /// @param[inout] response_size Size of the method return value structure
  /// @return Connection operation result
  virtual Result call(const Address& address,
                      std::uint16_t method_id,
                      const void* request_data,
                      std::size_t request_size,
                      void* response_data,
                      std::size_t response_size) noexcept = 0;

  /// @brief Request to be notified about events
  /// @param[in] address The service details
  /// @param[in] event_group_id The event group identifier
  /// @param[in] event_ids List of events to be notified upon
  /// @param[in] handler Event handler to call when one of the events arrives
  /// @return whether call succeeded
  virtual bool requestEvents(const Address& address,
                             std::uint16_t event_group_id,
                             const std::vector<std::uint16_t>& event_ids,
                             const EventHandler& handler) noexcept = 0;

  /// @brief offer an event
  /// @param[in] address The service details
  /// @param[in] event_id The event id
  /// @param[in] event_group_id The event group identifier
  virtual bool offerEvent(const Address& address,
                          std::uint16_t event_id,
                          std::uint16_t event_group_id) noexcept = 0;

  /// @brief Stop offering an event
  /// @param[in] address The service details
  /// @param[in] event_id The event id
  virtual bool stopOfferingEvent(const Address& address, std::uint16_t event_id) noexcept = 0;

  /// @brief Notify all parties listening to this event
  /// @param[in] address The service details
  /// @param[in] event_id The event id
  /// @param[in] event_data The event payload buffer
  /// @param[in] event_size The size of the payload
  virtual bool sendEvent(const Address& address,
                         std::uint16_t event_id,
                         const void* event_data,
                         std::size_t event_size) noexcept = 0;

  /// @brief Release event notification
  /// This will stop calls to the event handler handed over with requestEvents().
  /// @param[in] address The service details
  /// @return True if events notification has been stopped successfully
  virtual bool releaseEvents(const Address& address) noexcept = 0;
};

/// @brief Equality operator for Address
inline bool operator==(const ISomeIpTransport::Address& lhs,
                       const ISomeIpTransport::Address& rhs) noexcept
{
  return lhs.service_id == rhs.service_id && lhs.instance_id == rhs.instance_id;
}

} // namespace networking
} // namespace platform
} // namespace lum

#endif
