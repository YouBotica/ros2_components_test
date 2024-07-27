// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_ROS_2_COM_POINTCLOUD_CODECS_COMMON_LAYER_CODEC_H
#define LUM_COMMON_ROS_2_COM_POINTCLOUD_CODECS_COMMON_LAYER_CODEC_H

#include <lum_common_ros_2_com_pointcloud/point_cloud_codec.h>
#include <lum_common_types_pointcloud/point_cloud_layer.h>

LUM_COM_REGISTER_ROS_2_POINT_STRUCT(lum::common::types::point::CommonData)
{
  using lum::common::types::point::CommonData;
  LUM_COM_DECLARE_ROS_2_POINT_ARRAY_FIELD(CommonData, timestamp, timestamp, UINT8, 8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CommonData, x, x, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CommonData, y, y, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CommonData, z, z, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CommonData, reflectance, reflectance, FLOAT32);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CommonData, return_index, return_index, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CommonData, last_return_index, last_return_index, UINT8);
  LUM_COM_DECLARE_ROS_2_POINT_FIELD(CommonData, sensor_id, sensor_id, UINT8);
}

namespace lum {
namespace com {

using CommonPointCloudCodec =
  lum::common::swc::PointCloudCodec<lum::common::types::point_cloud::StructuredCommonLayer>;

using UnstructuredCommonPointCloudCodec =
  lum::common::swc::PointCloudCodec<lum::common::types::point_cloud::UnstructuredCommonLayer>;

} // namespace com
} // namespace lum
#endif
