// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_UTILS_H
#define LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_UTILS_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <lum_drivers_lidar_iris_internal_types/data_types.h>
#include <resimpointcloud.hpp>
#include <schemas/compressed.capnp.h>
#include <schemas/pointcloud.capnp.h>

enum class PRAlgorithms;

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

///////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Convert a ReSim point cloud message to a vector of Model I lidar ray bundles
/// @note Each ray bundle contains rays that share the same preamble information
/// @param point_cloud_msg [in] ReSim point cloud message
/// @param bundles [out] a vector of Model I lidar metadata and LidarRays
void convert(const ReSim::Schemas::PointCloud::Reader& point_cloud_msg,
             std::vector<std::pair<types::LidarPacketMetadata, types::LidarRays>>& bundles);

///////////////////////////////////////////////////////////////////////////////////////////////////
std::unique_ptr<ReSim::LibReSim<ReSim::Schemas::Compressed, ReSim::Schemas::PointCloud>>
makeLibReSim(const std::string& resim_calibration_file_path);

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
