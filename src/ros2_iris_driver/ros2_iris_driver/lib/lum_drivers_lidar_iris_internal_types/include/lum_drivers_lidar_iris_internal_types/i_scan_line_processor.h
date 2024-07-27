// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_SCAN_LINE_PROCESSOR_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_SCAN_LINE_PROCESSOR_H

#include <cstdint>
#include <string>
#include <utility>

#include <lum_common_memory/object_pool.h>
#include <lum_common_types/networking.h>
#include <lum_drivers_lidar_iris_internal_types/data_types.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

using PooledUdpPacket =
  lum::common::memory::object_pool::SmartReference<common::types::networking::UdpPacket>;

class IScanLineProcessor
{
public:
  virtual ~IScanLineProcessor() = default;

  // Boilerplate for polymorphic base class
  IScanLineProcessor() = default;
  IScanLineProcessor(const IScanLineProcessor&) = delete;
  IScanLineProcessor(IScanLineProcessor&&) = delete;
  IScanLineProcessor& operator=(const IScanLineProcessor&) & = delete;
  IScanLineProcessor& operator=(IScanLineProcessor&&) & = delete;

  /// @brief Set sensor id
  /// @param [in] sensor_id sensor id that will be assigned to the sensor_id property of each point
  virtual void setSensorId(std::uint8_t sensor_id) = 0;

  /// @brief Get sensor id
  /// @return sensor_id sensor id option
  virtual std::uint8_t getSensorId() const = 0;

  /// @brief Returns this processor's human-readable name and semver
  virtual std::pair<std::string, std::string> getNameAndVersion() const = 0;

  /// @brief reset the internal state of this processor
  virtual void reset() = 0;

  /// @brief add a new packet for processing, transferring ownership of the data
  /// @param packet [in] a pooled packet
  /// @return true if the message was added successfully
  virtual bool addPacket(const PooledUdpPacket& packet) = 0;

  /// @brief process a new packet without any ownership transfer
  /// @param buffer [in] a buffer containing packet data
  /// @param packet_size [in] the size of the buffer in bytes
  /// @param header [in] the associated packet header containing useful metadata
  /// @return true if the message was added successfully
  virtual bool addPacket(const std::uint8_t* buffer,
                         std::size_t packet_size,
                         const common::types::networking::UdpHeader& header) = 0;

  /// @brief subscribe to new line data
  /// @param [in] callback function to handle new line data
  /// @returns subscription handle, subscription is terminated when the handle goes out of scope
  /// @note  Data is temporary, and must be used within the callback, or copied for any later
  /// reference
  virtual types::PointCloudLineSubscription
  subscribeOnScanLineSegment(const types::PointCloudLineSubscriptionCallback& callback) = 0;

  /// @brief subscribe to new line data
  /// @param [in] callback function to handle new line data
  /// @returns subscription handle, subscription is terminated when the handle goes out of scope
  /// @note  Data is temporary, and must be used within the callback, or copied for any later
  /// reference
  virtual types::PointCloudSupplementalLineSubscription subscribeOnScanLineSegment(
    const types::PointCloudSupplementalLineSubscriptionCallback& callback) = 0;

  /// @brief subscribe to new line data
  /// @param [in] callback function to handle new line data
  /// @returns subscription handle, subscription is terminated when the handle goes out of scope
  /// @note  Data is temporary, and must be used within the callback, or copied for any later
  /// reference
  virtual types::UnstructuredLayeredDataSubscription subscribeOnScanLineSegment(
    const types::UnstructuredLayeredDataSubscriptionCallback& callback) = 0;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
