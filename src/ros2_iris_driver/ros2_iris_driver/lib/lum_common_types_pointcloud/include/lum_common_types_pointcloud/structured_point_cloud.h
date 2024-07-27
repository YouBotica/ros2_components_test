// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_STRUCTURED_POINTCLOUD_POINTCLOUD_H
#define LUM_COMMON_TYPES_STRUCTURED_POINTCLOUD_POINTCLOUD_H

#include <algorithm>
#include <cstring>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

#include <lum_common_exceptions/init.h>
#include <lum_common_types/common.h>
#include <lum_common_types_pointcloud/base_point_cloud.h>

namespace lum {
namespace common {
namespace types {
namespace point_cloud {

/// @brief common dimensional typing for StructuredPointClouds and Points
using DIMENSIONAL_TYPE = std::uint32_t;

/// @brief Class fo structured point clouds
/// @note Use this class to create a stuctured point cloud of type <T>.
template <typename T>
class StructuredPointCloud : public BasePointCloud<T>
{
private:
  /// @brief vertical resolution of the point cloud
  DIMENSIONAL_TYPE scan_lines_{0};

  /// @brief Horizontal resolution of the point cloud
  DIMENSIONAL_TYPE points_in_line_{0};

public:
  /// @brief hide standard constructor
  StructuredPointCloud() = default;

  /// @brief Constructor with horizontal and vertical resolution
  /// @param[in] scan_lines vertical resolution
  /// @param[in] points_in_line horizontal resolution
  StructuredPointCloud(const DIMENSIONAL_TYPE scan_lines, const DIMENSIONAL_TYPE points_in_line)
      : BasePointCloud<T>::BasePointCloud(scan_lines * points_in_line)
      , scan_lines_(scan_lines)
      , points_in_line_(points_in_line)
  {
    this->points_.reserve(static_cast<std::size_t>(this->scan_lines_) * this->points_in_line_);
    this->points_.resize(this->points_.capacity());
  }

  /// @brief Constructor with horizontal and vertical resolution and default point <T>
  /// @param[in] scan_lines vertical resolution
  /// @param[in] points_in_line horizontal resolution
  /// @param[in] value default point <T>
  StructuredPointCloud(const DIMENSIONAL_TYPE scan_lines,
                       const DIMENSIONAL_TYPE points_in_line,
                       const T& value)
      : StructuredPointCloud<T>::StructuredPointCloud(scan_lines, points_in_line)
  {
    this->points_.assign(this->points_.size(), value);
  }

  /// @brief resizes the point cloud
  /// @param[in] scan_lines vertical resolution
  /// @param[in] points_in_line horizontal resolution
  void resize(const DIMENSIONAL_TYPE scan_lines, const DIMENSIONAL_TYPE points_in_line)
  {
    if (scan_lines == 0)
    {
      throw exceptions::StructuredPointCloudInvalidConstructionException(
        "scan_lines", static_cast<std::size_t>(scan_lines));
    }

    if (points_in_line == 0)
    {
      throw exceptions::StructuredPointCloudInvalidConstructionException(
        "points_in_line", static_cast<std::size_t>(points_in_line));
    }

    this->points_.resize(static_cast<std::size_t>(scan_lines) * points_in_line);

    scan_lines_ = scan_lines;
    points_in_line_ = points_in_line;
  }

  /// @brief Resizes the structured point cloud to the given width and height
  /// and initializes all points with the given value T
  /// @param[in] scan_lines vertical resolution
  /// @param[in] points_in_line horizontal resolution
  /// @param[in] value default point <T>
  void
  reset(const DIMENSIONAL_TYPE scan_lines, const DIMENSIONAL_TYPE points_in_line, const T& value)
  {
    if (scan_lines == 0)
    {
      throw exceptions::StructuredPointCloudInvalidConstructionException(
        "scan_lines", static_cast<std::size_t>(scan_lines));
    }

    if (points_in_line == 0)
    {
      throw exceptions::StructuredPointCloudInvalidConstructionException(
        "points_in_line", static_cast<std::size_t>(points_in_line));
    }

    this->points_.assign(static_cast<std::size_t>(scan_lines) * points_in_line, value);

    scan_lines_ = scan_lines;
    points_in_line_ = points_in_line;
  }

  /// @brief Returns the vertical resolution of the structured point cloud
  /// @return vertical resolution
  DIMENSIONAL_TYPE getNumScanLines() const { return this->scan_lines_; }

  /// @brief Returns the horizontal resolution of the structured point cloud
  /// @return horizontal resolution
  DIMENSIONAL_TYPE getNumPointsInLine() const { return this->points_in_line_; }

  /// @brief Returns point T at the given coordinates of the structured point cloud
  /// @param[in] scan_line vertical index
  /// @param[in] point_in_line horizontal index
  /// @return Reference to point <T> at the given coordinates
  T& at(const DIMENSIONAL_TYPE scan_line, const DIMENSIONAL_TYPE point_in_line)
  {
    if (scan_line >= scan_lines_)
    {
      throw exceptions::StructuredPointCloudIndexOutOfBoundsException(
        static_cast<std::size_t>(scan_line), 0U, static_cast<std::size_t>(scan_lines_ - 1U));
    }

    if (point_in_line >= points_in_line_)
    {
      throw exceptions::StructuredPointCloudIndexOutOfBoundsException(
        static_cast<std::size_t>(point_in_line),
        0U,
        static_cast<std::size_t>(points_in_line_ - 1U));
    }

    return this->points_.at(static_cast<std::size_t>(scan_line) * points_in_line_ + point_in_line);
  }

  /// @brief Returns point T at the given coordinates of the structured point cloud
  /// @param[in] scan_line vertical index
  /// @param[in] point_in_line horizontal index
  /// @return Reference to point <T> at the given coordinates
  const T& at(const DIMENSIONAL_TYPE scan_line, const DIMENSIONAL_TYPE point_in_line) const
  {
    if (scan_line >= scan_lines_)
    {
      throw exceptions::StructuredPointCloudIndexOutOfBoundsException(
        static_cast<std::size_t>(scan_line), 0U, static_cast<std::size_t>(scan_lines_ - 1U));
    }

    if (point_in_line >= points_in_line_)
    {
      throw exceptions::StructuredPointCloudIndexOutOfBoundsException(
        static_cast<std::size_t>(point_in_line),
        0U,
        static_cast<std::size_t>(points_in_line_ - 1U));
    }

    return this->points_.at(static_cast<std::size_t>(scan_line) * points_in_line_ + point_in_line);
  }

  /// @brief Returns point T at the given coordinates of the structured point cloud
  /// @param[in] scan_line vertical index
  /// @param[in] point_in_line horizontal index
  /// @return Reference to point <T> at the given coordinates
  T& atNoCheck(const DIMENSIONAL_TYPE scan_line, const DIMENSIONAL_TYPE point_in_line)
  {
    // NOLINTNEXTLINE remove bounds checking for performance
    return this->points_[getSingleIndex(scan_line, point_in_line)];
  }

  /// @brief Returns point T at the given coordinates of the structured point cloud
  /// @param[in] scan_line vertical index
  /// @param[in] point_in_line horizontal index
  /// @return Reference to point <T> at the given coordinates
  const T& atNoCheck(const DIMENSIONAL_TYPE scan_line, const DIMENSIONAL_TYPE point_in_line) const
  {
    // NOLINTNEXTLINE remove bounds checking for performance
    return this->points_[getSingleIndex(scan_line, point_in_line)];
  }

  /// @brief get the size of the point cloud (const)
  /// @return Returns the size of the point cloud (const)
  std::size_t size() const override { return this->points_.size(); }

  /// @brief Return the unique single index for a given (row, col)
  /// @param[in] scan_line vertical index
  /// @param[in] point_in_line horizontal index
  /// @return Index for given (row, col) in row-major ordering
  inline std::size_t getSingleIndex(const DIMENSIONAL_TYPE scan_line,
                                    const DIMENSIONAL_TYPE point_in_line) const
  {
    return static_cast<std::size_t>(scan_line) * points_in_line_ + point_in_line;
  }

  using BasePointCloud<T>::at;
  using BasePointCloud<T>::atNoCheck;
};

/// @brief Shared pointer to StructuredPointCloud of type T
template <typename T>
using StructuredPointCloudPtr = std::shared_ptr<StructuredPointCloud<T>>;

/// @brief Shared const pointer to StructuredPointCloud of type T
template <typename T>
using StructuredPointCloudConstPtr = std::shared_ptr<const StructuredPointCloud<T>>;

} // namespace point_cloud
} // namespace types
} // namespace common
} // namespace lum

#endif
