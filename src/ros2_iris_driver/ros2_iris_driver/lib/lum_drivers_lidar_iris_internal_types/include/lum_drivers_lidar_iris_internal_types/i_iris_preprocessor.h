// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_IRIS_PREPROCESSOR_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_IRIS_PREPROCESSOR_H

#include <lum_drivers_data_client_types/types.h>
#include <lum_drivers_lidar_iris_internal_types/data_types.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

class IIrisPreprocessor
{
public:
  /// @brief add a new packet for processing
  /// @param data [in] a struct containing the packet to be processed
  /// @returns true if the packet was accepted as valid
  virtual bool addPacket(const data_client::types::DataClientData& data) = 0;

  /// @brief subscribe to access new frame data.
  /// @param [in] callback function to handle new frame data
  /// @returns subscription handle, subscription is terminated when the handle goes out of scope
  /// @note  Data is temporary, and must be used within the callback, or copied for any later
  /// reference
  virtual types::UnstructuredLayeredDataSubscription
  subscribeOnFrameEnd(const types::UnstructuredLayeredDataSubscriptionCallback& callback) = 0;

  /// @brief subscribe to access new line data.
  /// @param [in] callback function to handle new line data
  /// @returns subscription handle, subscription is terminated when the handle goes out of scope
  /// @note  Data is temporary, and must be used within the callback, or copied for any later
  /// reference
  virtual types::UnstructuredLayeredDataSubscription subscribeOnScanLineSegment(
    const types::UnstructuredLayeredDataSubscriptionCallback& callback) = 0;

  // Polymorphic base class boilerplate
  virtual ~IIrisPreprocessor() = default;
  IIrisPreprocessor() = default;
  IIrisPreprocessor(const IIrisPreprocessor&) = delete;
  IIrisPreprocessor(IIrisPreprocessor&&) = delete;
  IIrisPreprocessor& operator=(const IIrisPreprocessor&) & = delete;
  IIrisPreprocessor& operator=(IIrisPreprocessor&&) & = delete;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
