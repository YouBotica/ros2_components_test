// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_DATA_CLIENT_TYPES_TYPES_H
#define LUM_DRIVERS_DATA_CLIENT_TYPES_TYPES_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include <lum_common_memory/object_pool.h>
#include <lum_common_types/networking.h>
#include <lum_common_types_observable/subscription.h>

namespace lum {
namespace drivers {
namespace data_client {
namespace types {
///@namespace lum::drivers::data_client::types convenience typings for complex types

using PacketTimestamp = std::chrono::system_clock::time_point;
using EncodedIPAddress = std::uint32_t;

using UDPPacketPool =
  lum::common::memory::object_pool::ISmartPool<lum::common::types::networking::UdpPacket>;

using PooledUDPPacket =
  lum::common::memory::object_pool::SmartReference<common::types::networking::UdpPacket>;

struct DataClientData
{
  PooledUDPPacket packet{};
  PacketTimestamp timestamp{};
  EncodedIPAddress encoded_source_address{0U};
};

using PacketSubscription =
  lum::common::types::observable::SubscriptionHandle<const DataClientData&>;
using PacketSubscriptionCallback = std::function<void(const DataClientData&)>;

/// @brief a handler for a plain text message
/// @param [in] message The message to process
using PlainTextMessageCallback = std::function<void(const std::string& message)>;

} // namespace types
} // namespace data_client
} // namespace drivers
} // namespace lum

#endif
