// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_I_RESIM_PROCESSOR_H
#define LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_I_RESIM_PROCESSOR_H

#include <functional>
#include <memory>
#include <vector>

#include <lum_drivers_lidar_iris_resim_internal/types.h>
#include <schemas/pointcloud.capnp.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief replaceable interface for internal resim processing step
class IResimProcessor
{
public:
  using Callback = std::function<void(const ReSim::Schemas::PointCloud::Reader&)>;

  /// @brief is internal processing done with current data
  /// @return true if there are no internal queues still processing
  virtual bool isProcessingDone() const = 0;

  /// @brief submit a raw UDP message for processing
  /// @param[in] msg The message
  virtual void addMessageBatch(types::OwnedBatch msg) = 0;

  /// @brief set function to be called when a new point cloud message is available
  /// @param[in] callback The callback function
  virtual void setCallback(Callback callback) = 0;

  /// @brief clear internal state
  virtual void reset() = 0;

  // Boilerplate for polymorphic base class
  IResimProcessor() = default;
  virtual ~IResimProcessor() = default;
  IResimProcessor(const IResimProcessor&) = delete;
  IResimProcessor(IResimProcessor&&) = delete;
  void operator=(const IResimProcessor&) & = delete;
  void operator=(IResimProcessor&&) & = delete;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
