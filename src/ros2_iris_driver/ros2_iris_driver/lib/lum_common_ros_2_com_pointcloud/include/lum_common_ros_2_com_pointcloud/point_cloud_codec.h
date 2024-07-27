// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_POINTCLOUD_CODEC_H
#define LUM_COMMON_SWC_POINTCLOUD_CODEC_H

#include <lum_common_types_pointcloud/base_point_cloud.h>

#include "utils/conversions.h"
#include "utils/point_registration.h"

namespace lum {
namespace common {
namespace swc {

/// @brief PointCloudCodec is a generic codec that is used by swc::RosCommunication to provide
/// conversion between pointclouds of an arbitary point type and ROS PCL message
/// Pontcloud class must be inherited from lum::common::types::point_cloud::BasePointCloud
/// Usage:
/// @brief 1. Register point type used in your pointcloud and provide mapping to ROS fields
/// @code
/// #include <lum_common_ros_com_pointcloud/point_cloud_codec.h>
///
/// struct myPoint
/// {
///   float x;
///   std::uint8_t data[4];
/// };
///
/// LUM_COM_REGISTER_ROS_2_POINT_STRUCT( myPoint )
/// {
///   LUM_COM_DECLARE_ROS_2_POINT_FIELD( myPoint, x, x, FLOAT32 );
///   LUM_COM_DECLARE_ROS_2_POINT_ARRAY_FIELD( myPoint, data, data, UINT8, 4 );
/// }
/// @endcode
/// @note LUM_COM_REGISTER_ROS_2_POINT_STRUCT must be placed in the global namespace.
/// @note LUM_COM_DECLARE_ROS_2_POINT_FIELD maps a single struct member to ROS type. It accepts
/// datatype, accessor, ROS field name and ROS type.
/// @note LUM_COM_DECLARE_ROS_2_POINT_ARRAY_FIELD maps struct array member to ROS type. It accepts
/// datatype, accessor, ROS field name, ROS type and array size.
/// @note ROS type is enumerable from sensor_msgs::msg::PointField, see
/// https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointField.html
///
/// @brief 2. During topic registration, use pointcloud codec templated by structured or unstrutured
/// pointcloud of that point type
/// @code
/// swc::RosCommunication ros_com;
/// ros_com.registerTopic<swc::Topic, common::swc::PointCloudCodec<StructuredPointCloud<myPoint>>>(
///   "my_topic_name" );
/// @endcode

template <typename ContainerType,
          typename PointType = typename ContainerType::iterator::value_type,
          typename = typename std::enable_if<
            std::is_base_of<lum::common::types::point_cloud::BasePointCloud<PointType>,
                            ContainerType>::value>::type>
class PointCloudCodec
{
public:
  using IndependentType = ContainerType;
  using DependentType = sensor_msgs::msg::PointCloud2;

  static DependentType encode(const IndependentType& pointcloud)
  {
    DependentType msg{};
    const auto& ros_fields = point_registration::PointFields<PointType>::getFields();
    conversions::toROSMsg(pointcloud, ros_fields, msg);

    return msg;
  }

  static IndependentType decode(const DependentType& msg)
  {
    IndependentType pointcloud{};
    const auto& ros_fields = point_registration::PointFields<PointType>::getFields();
    conversions::fromROSMsg(msg, ros_fields, pointcloud);

    return pointcloud;
  }
};

} // namespace swc
} // namespace common
} // namespace lum

#endif // LUM_COMMON_SWC_POINTCLOUD_CODEC_H
