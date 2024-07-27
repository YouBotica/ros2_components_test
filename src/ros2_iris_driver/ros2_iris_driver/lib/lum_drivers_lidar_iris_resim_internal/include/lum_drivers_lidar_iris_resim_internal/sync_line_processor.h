// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_SYNC_LINE_PROCESSOR_H
#define LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_SYNC_LINE_PROCESSOR_H

#include <memory>

#include <lum_drivers_lidar_iris_resim_internal/i_resim_processor.h>
#include <lum_drivers_lidar_iris_resim_internal/types.h>
#include <lum_drivers_lidar_iris_types_resim/region_of_interest.h>
#include <resimpointcloud.hpp>
#include <resimudptocompressed.hpp>
#include <schemas/compressed.capnp.h>
#include <schemas/roi_filter.capnp.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief class that listens for ReSim compressed messages and outputs lidar scan lines
class SyncResimScanLineProcessor : public virtual IResimProcessor
{
public:
  /////////////////////////////////////////////////////////////////////////////////////////////////
  SyncResimScanLineProcessor(
    std::unique_ptr<ReSim::LibReSim<ReSim::Schemas::Compressed, ReSim::Schemas::PointCloud>> resim,
    const resim::IrisRegionOfInterest& roi_filter);

  /////////////////////////////////////////////////////////////////////////////////////////////////
  bool isProcessingDone() const override;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  void setCallback(Callback callback) final;

  /////////////////////////////////////////////////////////////////////////////////////////////////
  void addMessageBatch(types::OwnedBatch msgs) override;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  void reset() override {}

protected:
  /// Converter for UDP packets to compressed messages
  ReSim::LibReSim<ReSim::Schemas::Udp, ReSim::Schemas::Compressed> resim_udp_to_compressed_{};

private:
  /// ReSim object that processes detector messages and outputs lidar rays
  std::unique_ptr<ReSim::LibReSim<ReSim::Schemas::Compressed, ReSim::Schemas::PointCloud>>
    resim_decompression_point_cloud_{};

  /// callback when processing is complete
  Callback callback_;

  // own the roi_filter, resim references this but does not copy
  types::Message<ReSim::Schemas::RoiFilter> roi_filter_{};
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
