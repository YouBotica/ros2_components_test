// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_ROS_2_COM_POINTCLOUD_CODECS_DEBUG_LAYER_CODEC_H
#define LUM_COMMON_ROS_2_COM_POINTCLOUD_CODECS_DEBUG_LAYER_CODEC_H

#include <lum_common_ros_2_com_pointcloud/point_cloud_codec.h>
#include <lum_common_types_pointcloud/point_cloud_layer.h>

LUM_COM_REGISTER_ROS_2_POINT_STRUCT(lum::common::types::point::DebugData)
{
  using common::types::point::DebugData;
  LUM_COM_DECLARE_ROS_2_POINT_ARRAY_FIELD(DebugData,
                                         reserved,
                                         reserved,
                                         UINT8,
                                         lum::common::types::point::DebugData::NUM_RESERVED_FIELDS);
}

namespace lum {
namespace com {

using DebugPointCloudCodec =
  lum::common::swc::PointCloudCodec<lum::common::types::point_cloud::StructuredDebugLayer>;

} // namespace com
} // namespace lum

#endif
