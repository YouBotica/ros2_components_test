// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_TYPES_H
#define LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_TYPES_H

#include <string>
#include <vector>

#include <capnp/serialize.h>
#include <lum_common_casts/lum_casts.h>
#include <lum_common_memory/object_pool.h>
#include <lum_common_types/networking.h>
#include <schemas/udp.capnp.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

namespace types {

using PooledUdpPacket =
  lum::common::memory::object_pool::SmartReference<common::types::networking::UdpPacket>;

/// @brief helper class to wrap a capnp message and handle lifecycle concerns
/// This is a streamlined clone of class logic from lum_ecu_middleware
template <typename CapnpMessageType>
class Message
{
  using NewMessage = capnp::MallocMessageBuilder;
  using Builder = typename CapnpMessageType::Builder;
  using Reader = typename CapnpMessageType::Reader;

public:
  Message()
      : message_builder_(std::make_unique<NewMessage>())
      , builder_root_(message_builder_->initRoot<CapnpMessageType>())
  {
  }

  /// @brief Access to actual message builder
  /// @return capnp message builder
  inline Builder& operator()() { return builder_root_; }

  /// @brief Access to actual message builder
  /// @return capnp message builder
  inline const Builder& operator()() const { return builder_root_; }

  /// @brief use an existing buffer in this message without a copy.  This must be paired with an
  /// ".adoptXxx()" call
  /// @note the buffer must exist for the duration of the message, otherwise behavior becomes
  /// undefined
  /// @param data_pointer [in] a pointer to the first byte in the buffer
  /// @param buffer_size [in] the size of the referenced buffer, in bytes
  /// @tparam DataType type of incoming data buffer, e.g. uint8_t, ClassifiedPoint, etc
  template <typename DataType>
  capnp::Orphan<capnp::Data> referenceBuffer(const DataType* const data_pointer,
                                             std::size_t buffer_size)
  {
    using lum::common::casts::lum_reinterpret_cast;
    return message_builder_->getOrphanage().referenceExternalData(
      capnp::Data::Reader(lum_reinterpret_cast<const kj::byte*>(data_pointer), buffer_size));
  }

private:
  // MessageBuilders are not copyable or moveable.  unique_ptr so Message is moveable.
  std::unique_ptr<NewMessage> message_builder_; ///< Message builder object
  Builder builder_root_;                        ///< Message root
};

/// @brief simple struct to hold pooled memory alonside referencing structures so lifecycle is
/// managed
struct OwnedBatch
{
  Message<ReSim::Schemas::Udp> message;
  std::vector<PooledUdpPacket> owned_memory;
};

} // namespace types
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_TYPES_H
