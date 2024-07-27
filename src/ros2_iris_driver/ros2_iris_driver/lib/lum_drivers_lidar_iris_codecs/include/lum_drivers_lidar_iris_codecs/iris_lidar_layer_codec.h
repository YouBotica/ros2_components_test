// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_CODECS_IRIS_LIDAR_LAYER_CODEC_H
#define LUM_DRIVERS_LIDAR_IRIS_CODECS_IRIS_LIDAR_LAYER_CODEC_H

#include <lum_common_ros_2_com_pointcloud/point_cloud_codec.h>
#include <lum_drivers_lidar_iris_types/point_cloud_layer.h>

LUM_COM_REGISTER_ROS_2_POINT_STRUCT(lum::drivers::lidar::iris::types::point::LidarData)
{
  using lum::drivers::lidar::iris::types::point::LidarData;
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(LidarData, line_index, line_index, UINT16);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(LidarData, frame_index, frame_index, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(LidarData, detector_site_id, detector_site_id, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(LidarData, scan_checkpoint, scan_checkpoint, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(
    LidarData, existence_probability_percent, existence_probability_percent, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(LidarData, data_qualifier, data_qualifier, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(LidarData, blockage_level, blockage_level, UINT8);
}

namespace lum {
namespace com {

using IrisLidarPointCloudCodec = lum::common::swc::PointCloudCodec<
  lum::drivers::lidar::iris::types::point_cloud::StructuredLidarLayer>;
using UnstructuredIrisLidarPointCloudCodec = lum::common::swc::PointCloudCodec<
  lum::drivers::lidar::iris::types::point_cloud::UnstructuredLidarLayer>;

} // namespace com
} // namespace lum
#endif
