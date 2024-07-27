// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_PACKET_BATCHER_H
#define LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_PACKET_BATCHER_H

#include <functional>

#include <lum_drivers_lidar_iris_resim_internal/types.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

class PacketBatcher
{
public:
  using Callback = std::function<void(types::OwnedBatch)>;

  explicit PacketBatcher(std::size_t batch_size);

  void setCallback(Callback callback);

  void addPacket(types::PooledUdpPacket packet);

  void reset();

private:
  bool has_received_status_{false};
  std::size_t batch_index_{0U};
  std::size_t batch_size_{1U};
  Callback callback_;
  types::OwnedBatch batch_;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
