// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_UTILS_CONVERSIONS_ROS_2_H
#define LUM_COMMON_SWC_UTILS_CONVERSIONS_ROS_2_H

#include <lum_common_log/log.h>
#include <lum_common_types/common.h>
#include <lum_common_types_pointcloud/structured_point_cloud.h>
#include <lum_common_types_pointcloud/unstructured_point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lum {
namespace common {
namespace swc {
namespace conversions {

template <typename PointType>
void toROSMsg(const lum::common::types::point_cloud::BasePointCloud<PointType>& pointcloud,
              const std::uint32_t width,
              const std::uint32_t height,
              const sensor_msgs::msg::PointCloud2::_fields_type& fields,
              bool is_dense,
              sensor_msgs::msg::PointCloud2& msg)
{
  // set header
  auto& header = msg.header;
  std::int64_t nanoseconds = pointcloud.getTimestamp().count();
  header.stamp.sec = nanoseconds / std::nano::den;
  header.stamp.nanosec = nanoseconds - header.stamp.sec * std::nano::den;
  header.frame_id = "base_link";

  msg.width = width;
  msg.height = height;
  msg.fields = fields;
  msg.point_step = sizeof(PointType);
  msg.row_step = static_cast<std::uint32_t>(sizeof(PointType) * msg.width);
  msg.is_dense = is_dense;

  // copy data
  size_t data_size = sizeof(PointType) * pointcloud.size();
  msg.data.resize(data_size);
  pointcloud.serializeToBuffer(&msg.data[0]);
}

template <typename PointType>
void toROSMsg(const lum::common::types::point_cloud::StructuredPointCloud<PointType>& pointcloud,
              const sensor_msgs::msg::PointCloud2::_fields_type& fields,
              sensor_msgs::msg::PointCloud2& msg)
{
  toROSMsg(
    pointcloud, pointcloud.getNumPointsInLine(), pointcloud.getNumScanLines(), fields, false, msg);
}

template <typename PointType>
void toROSMsg(const lum::common::types::point_cloud::UnstructuredPointCloud<PointType>& pointcloud,
              const sensor_msgs::msg::PointCloud2::_fields_type& fields,
              sensor_msgs::msg::PointCloud2& msg)
{
  toROSMsg(pointcloud, pointcloud.size(), 1U, fields, true, msg);
}

inline bool areFieldTypesIdentical(const sensor_msgs::msg::PointCloud2::_fields_type& fields_a,
                                   const sensor_msgs::msg::PointCloud2::_fields_type& fields_b)
{
  if (fields_a.size() != fields_b.size())
  {
    return false;
  }

  for (std::size_t index{0U}; index < fields_a.size(); ++index)
  {
    const auto& field_a = fields_a[index];
    const auto& field_b = fields_b[index];
    if (field_a.name != field_b.name || field_a.offset != field_b.offset ||
        field_a.datatype != field_b.datatype || field_a.count != field_b.count)
    {
      return false;
    }
  }
  return true;
}

template <typename PointType>
void fromROSMsg(const sensor_msgs::msg::PointCloud2& msg,
                lum::common::types::point_cloud::BasePointCloud<PointType>& pointcloud)
{
  size_t data_size = sizeof(PointType) * pointcloud.size();
  if (data_size != msg.data.size())
  {
    LUM_LOG_ERROR << "Mismatch between ROS message size and a pointcloud.";
    return;
  }

  // get time
  const auto& header_stamp = msg.header.stamp;
  lum::common::types::Time timestamp{std::chrono::nanoseconds(
    static_cast<std::int64_t>(header_stamp.sec * std::nano::den + header_stamp.nanosec))};
  pointcloud.setTimestamp(timestamp);

  // copy data
  pointcloud.deserializeFromBuffer(&msg.data[0], msg.data.size());
}

template <typename PointType>
void fromROSMsg(const sensor_msgs::msg::PointCloud2& msg,
                const sensor_msgs::msg::PointCloud2::_fields_type& fields,
                lum::common::types::point_cloud::StructuredPointCloud<PointType>& pointcloud)
{
  (void)fields;
  if (msg.is_dense)
  {
    LUM_LOG_WARN
      << "ROS message contains unstructured pointcloud. Requested structured pointcloud.";
  }
  if (areFieldTypesIdentical(fields, msg.fields))
  {
    pointcloud.resize(msg.height, msg.width);
    fromROSMsg(msg, pointcloud);
  }
  else
  {
    LUM_LOG_ERROR << "ROS message point definition does not match the one from the pointcloud.";
  }
}

template <typename PointType>
void fromROSMsg(const sensor_msgs::msg::PointCloud2& msg,
                const sensor_msgs::msg::PointCloud2::_fields_type& fields,
                lum::common::types::point_cloud::UnstructuredPointCloud<PointType>& pointcloud)
{
  if (!msg.is_dense || msg.height != 1)
  {
    LUM_LOG_WARN
      << "ROS message contains structured pointcloud. Requested unstructured pointcloud.";
  }
  if (areFieldTypesIdentical(fields, msg.fields))
  {
    pointcloud.resize(msg.height * msg.width);
    fromROSMsg(msg, pointcloud);
  }
  else
  {
    LUM_LOG_ERROR << "ROS message point definition does not match the one from the pointcloud.";
  }
}

} // namespace conversions
} // namespace swc
} // namespace common
} // namespace lum

#endif // LUM_COMMON_SWC_UTILS_CONVERSIONS_H
