// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_STRUCTURED_POINTCLOUD_REORDERING_H
#define LUM_COMMON_UTILS_STRUCTURED_POINTCLOUD_REORDERING_H

#include <algorithm>
#include <vector>

#include <lum_common_types_pointcloud/structured_point_cloud.h>

namespace lum {
namespace common {
namespace utils {

/// @brief This function reorders the scan lines of a structured pointcloud by sorting it based on
/// elevation angles of the scan lines. This is useful when you want to create a 2.5D range image.
/// @tparam T Point Type of Structured Pointcloud
/// @param[in] point_cloud
/// @param [out] Pointer to reordered pointcloud
template <typename T>
void reorderStructuredPointcloud(
  const types::point_cloud::StructuredPointCloud<T>& point_cloud,
  types::point_cloud::StructuredPointCloudPtr<T>& reordered_pointcloud)
{
  const auto& number_of_scan_lines = point_cloud.getNumScanLines();
  const auto& number_of_points_in_line = point_cloud.getNumPointsInLine();

  // Resize reordered pointcloud if needed
  reordered_pointcloud->resize(number_of_scan_lines, number_of_points_in_line);

  // Create vector of pairs of scan line index and elevation angle
  std::vector<std::pair<types::point_cloud::DIMENSIONAL_TYPE, float>> scan_line_order(
    number_of_scan_lines);

  for (types::point_cloud::DIMENSIONAL_TYPE scan_line_index{0};
       scan_line_index < number_of_scan_lines;
       ++scan_line_index)
  {
    const auto& scan_line_mid_point = point_cloud.at(scan_line_index, number_of_points_in_line / 2);

    scan_line_order[scan_line_index] =
      std::make_pair(scan_line_index, scan_line_mid_point.elevation);
  }

  // Sort the scan lines based on elevation angle
  std::stable_sort(scan_line_order.begin(),
                   scan_line_order.end(),
                   [](const auto& left, const auto& right) { return left.second > right.second; });

  // Populate reordered pointcloud based on sorted indices
  for (types::point_cloud::DIMENSIONAL_TYPE scan_line_index{0};
       scan_line_index < number_of_scan_lines;
       ++scan_line_index)
  {
    for (types::point_cloud::DIMENSIONAL_TYPE index_in_scan_line{0};
         index_in_scan_line < number_of_points_in_line;
         ++index_in_scan_line)
    {
      auto& point_in_reordered_cloud =
        reordered_pointcloud->at(scan_line_index, index_in_scan_line);
      auto& point_in_input_cloud =
        point_cloud.at(scan_line_order[scan_line_index].first, index_in_scan_line);

      point_in_reordered_cloud = point_in_input_cloud;
    }
  }
}

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_STRUCTURED_POINTCLOUD_REORDERING_H
