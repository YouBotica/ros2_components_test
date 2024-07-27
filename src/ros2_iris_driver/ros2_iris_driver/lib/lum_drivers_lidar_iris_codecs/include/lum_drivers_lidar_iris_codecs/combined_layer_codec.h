// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_CODECS_COMBINED_LAYER_CODEC_H
#define LUM_DRIVERS_LIDAR_IRIS_CODECS_COMBINED_LAYER_CODEC_H

#include <lum_common_ros_2_com_pointcloud/point_cloud_codec.h>
#include <lum_drivers_lidar_iris_types/point_cloud_layer.h>

LUM_COM_REGISTER_ROS_2_POINT_STRUCT(lum::drivers::lidar::iris::types::point::CombinedData)
{
  using lum::drivers::lidar::iris::types::point::CombinedData;

  static_assert(sizeof(lum::common::types::point::CommonData) == 32,
                "Common data layer has changed. Please update this ROS field spec.");
  static_assert(sizeof(lum::common::types::point::PolarData) == 12,
                "Polar data layer has changed. Please update this ROS field spec.");
  static_assert(sizeof(lum::drivers::lidar::iris::types::point::LidarData) == 8,
                "Lidar data layer has changed. Please update this ROS field spec.");

  // CommonData
  LUM_COM_DECLARE_ROS_2_POINT_ARRAY_FIELD(CombinedData, common_data.timestamp, timestamp, UINT8, 8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, common_data.x, x, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, common_data.y, y, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, common_data.z, z, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, common_data.reflectance, reflectance, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, common_data.return_index, return_index, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(
    CombinedData, common_data.last_return_index, last_return_index, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, common_data.sensor_id, sensor_id, UINT8);

  // PolarData
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, polar_data.azimuth, azimuth, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, polar_data.elevation, elevation, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, polar_data.depth, depth, FLOAT32);

  // LidarData
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, lidar_data.line_index, line_index, UINT16);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, lidar_data.frame_index, frame_index, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(
    CombinedData, lidar_data.detector_site_id, detector_site_id, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(
    CombinedData, lidar_data.scan_checkpoint, scan_checkpoint, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(
    CombinedData, lidar_data.existence_probability_percent, existence_probability_percent, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, lidar_data.data_qualifier, data_qualifier, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CombinedData, lidar_data.blockage_level, blockage_level, UINT8);
}

namespace lum {
namespace com {

using CombinedPointCloudCodec = lum::common::swc::PointCloudCodec<
  lum::drivers::lidar::iris::types::point_cloud::StructuredCombinedLayer>;

using UnstructuredCombinedPointCloudCodec = lum::common::swc::PointCloudCodec<
  lum::drivers::lidar::iris::types::point_cloud::UnstructuredCombinedLayer>;

} // namespace com
} // namespace lum

#endif
