// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_DATA_CLIENT_I_DATA_CLIENT_H
#define LUM_DRIVERS_DATA_CLIENT_I_DATA_CLIENT_H

#include <lum_drivers_data_client_types/types.h>

namespace lum {
namespace drivers {
namespace data_client {

class IPacketSource
{
public:
  // Polymorphic base class boilerplate
  virtual ~IPacketSource() = default;
  IPacketSource() = default;
  IPacketSource(const IPacketSource&) = delete;
  IPacketSource(IPacketSource&&) = delete;
  void operator=(const IPacketSource&) & = delete;
  void operator=(IPacketSource&&) & = delete;

  /// @brief subscribe to new packet data
  /// @param [in] callback function to handle new frame data
  /// @returns subscription handle
  /// @note
  /// Lifespan of the callback is determined by subscription handle
  /// When subscription handle goes out of scope, callback is removed
  virtual types::PacketSubscription
  subscribeOnPacket(const types::PacketSubscriptionCallback& callback) = 0;
};

class IListener
{
public:
  // Polymorphic base class boilerplate
  virtual ~IListener() = default;
  IListener() = default;
  IListener(const IListener&) = delete;
  IListener(IListener&&) = delete;
  void operator=(const IListener&) & = delete;
  void operator=(IListener&&) & = delete;

  /// @brief attempt to listen and process a single packet.  This must be called repeatedly for
  /// continuous processing
  /// @param packet [in] a pooled buffer for use in processing
  ///
  /// @note The packet can be held after the subscription callback returns.  This will stop
  /// receiving UDP packets if the pool is empty, either through being undersized, or from packets
  /// not getting returned
  /// @return true if this function can be called again
  virtual bool listenOnce(types::PooledUDPPacket packet) = 0;
};

/// @brief Interface for exposing a data stream for processing
class IDataClient : public IListener, public IPacketSource
{
public:
  // Polymorphic base class boilerplate
  ~IDataClient() override = default;
  IDataClient() = default;
  IDataClient(const IDataClient&) = delete;
  IDataClient(IDataClient&&) = delete;
  void operator=(const IDataClient&) & = delete;
  void operator=(IDataClient&&) & = delete;
};

} // namespace data_client
} // namespace drivers
} // namespace lum

#endif
