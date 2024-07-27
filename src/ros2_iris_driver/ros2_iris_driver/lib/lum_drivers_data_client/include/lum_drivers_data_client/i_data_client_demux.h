// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_DATA_CLIENT_I_DATA_CLIENT_DEMUX_H
#define LUM_DRIVERS_DATA_CLIENT_I_DATA_CLIENT_DEMUX_H

#include <lum_common_types_observable/subscription.h>
#include <lum_drivers_data_client/i_data_client.h>
#include <lum_drivers_data_client_types/types.h>

namespace lum {
namespace drivers {
namespace data_client {

///////////////////////////////////////////////////////////////////////////////////////////////////
namespace types {

using ProxyDataSourceSubscription = lum::common::types::observable::
  SubscriptionHandle<const std::shared_ptr<data_client::IPacketSource>&, const EncodedIPAddress>;
using ProxyDataSourceSubscriptionCallback =
  std::function<void(const std::shared_ptr<data_client::IPacketSource>&, const EncodedIPAddress)>;
} // namespace types

///////////////////////////////////////////////////////////////////////////////////////////////////
class IDataClientDemux
{
public:
  // Polymorphic base class boilerplate
  virtual ~IDataClientDemux() = default;
  IDataClientDemux() = default;
  IDataClientDemux(const IDataClientDemux&) = delete;
  IDataClientDemux(IDataClientDemux&&) = delete;
  void operator=(const IDataClientDemux&) & = delete;
  void operator=(IDataClientDemux&&) & = delete;

  virtual void addPacket(const types::DataClientData& packet) = 0;

  virtual types::ProxyDataSourceSubscription
  subscribeToSources(const types::ProxyDataSourceSubscriptionCallback& callback) = 0;
};

} // namespace data_client
} // namespace drivers
} // namespace lum

#endif
