// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_BOUNDING_BOX_H
#define LUM_COMMON_UTILS_BOUNDING_BOX_H

#include <limits>
#include <type_traits>
#include <vector>

namespace lum {
namespace common {
namespace utils {

/// @brief Given a vector of axis aligned bounding boxes, compute single union bounding box as shown
/// here https://luminartech.atlassian.net/wiki/spaces/SWE/pages/876609746/Data+Detection+EVF+Sync
/// case 2.
/// @param boxes bounding boxes of type lum::pdk::common::types::geometry::BoundingBox3D that
/// required the union to be calculated
/// @return the union bounding box
template <typename BoxType>
BoxType getUnionOfAxisAlignedBoxes(const std::vector<BoxType>& boxes)
{
  // In case no boxes were provided, return an empty box
  if (boxes.empty())
  {
    return {};
  }

  using PoseType = decltype(boxes[0].pose.position.x);
  // create minimum and maximum values across x,y,z dimensions and assign them numeric_limits
  PoseType x_min{std::numeric_limits<PoseType>::max()};
  PoseType x_max{std::numeric_limits<PoseType>::min()};
  PoseType y_min{std::numeric_limits<PoseType>::max()};
  PoseType y_max{std::numeric_limits<PoseType>::min()};
  PoseType z_min{std::numeric_limits<PoseType>::max()};
  PoseType z_max{std::numeric_limits<PoseType>::min()};

  for (const auto& box : boxes)
  {
    // Compute the corners of the current box in iteration
    auto box_x_min = box.pose.position.x - box.scale.x / 2.F;
    auto box_x_max = box.pose.position.x + box.scale.x / 2.F;
    auto box_y_min = box.pose.position.y - box.scale.y / 2.F;
    auto box_y_max = box.pose.position.y + box.scale.y / 2.F;
    auto box_z_min = box.pose.position.z - box.scale.z / 2.F;
    auto box_z_max = box.pose.position.z + box.scale.z / 2.F;

    // find minimum of x dimension
    if (box_x_min < x_min)
    {
      x_min = box_x_min;
    }
    // find maximum of x dimension
    if (box_x_max > x_max)
    {
      x_max = box_x_max;
    }
    // find minimum of y dimension
    if (box_y_min < y_min)
    {
      y_min = box_y_min;
    }
    // find maximum of y dimension
    if (box_y_max > y_max)
    {
      y_max = box_y_max;
    }

    // find minimum of z dimension
    if (box_z_min < z_min)
    {
      z_min = box_z_min;
    }
    // find maximum of z dimension
    if (box_z_max > z_max)
    {
      z_max = box_z_max;
    }
  }

  BoxType union_box;

  // Compute the the center of the union box
  union_box.pose.position.x = (x_min + x_max) / 2.F;
  union_box.pose.position.y = (y_min + y_max) / 2.F;
  union_box.pose.position.z = (z_min + z_max) / 2.F;

  // Compute the the scale of the union box
  union_box.scale.x = x_max - x_min;
  union_box.scale.y = y_max - y_min;
  union_box.scale.z = z_max - z_min;

  return union_box;
}

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_BOUNDING_BOX_H
