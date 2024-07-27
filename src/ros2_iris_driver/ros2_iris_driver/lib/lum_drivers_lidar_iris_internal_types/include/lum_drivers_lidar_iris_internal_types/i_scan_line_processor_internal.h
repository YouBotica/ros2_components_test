// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_SCAN_LINE_PROCESSOR_INTERNAL_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_SCAN_LINE_PROCESSOR_INTERNAL_H

#include <lum_drivers_lidar_iris_internal_types/i_scan_line_processor.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

class IScanLineProcessorInternal : public virtual IScanLineProcessor
{
public:
  /// @brief Gets all the frame indices associated with the data contained within the packet
  /// @param packet [in] a pooled packet
  /// @return the frame indices and whether the set of frame indices is valid
  virtual types::FrameIndices getPacketFrameIndices(const PooledUdpPacket& packet) = 0;

  /// @brief Gets all the frame indices associated with the data contained within the packet
  /// @param buffer [in] a buffer containing packet data
  /// @param packet_size [in] the size of the buffer in bytes
  /// @param header [in] the associated packet header containing useful metadata
  /// @return the frame indices and whether the set of frame indices is valid
  virtual types::FrameIndices
  getPacketFrameIndices(const std::uint8_t* buffer,
                        std::size_t packet_size,
                        const common::types::networking::UdpHeader& header) = 0;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
