// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_I_RESIM_SCANLINE_PROCESSOR_H
#define LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_I_RESIM_SCANLINE_PROCESSOR_H

#include <lum_drivers_lidar_iris_internal_types/i_scan_line_processor.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief class that listens for ReSim messages and outputs lidar scan lines
class IResimScanLineProcessor : public virtual IScanLineProcessor
{
public:
  /// @brief is internal processing done with current data
  /// @return true if there are no internal queues still processing
  virtual bool isProcessingDone() const = 0;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_I_RESIM_SCANLINE_PROCESSOR_H
