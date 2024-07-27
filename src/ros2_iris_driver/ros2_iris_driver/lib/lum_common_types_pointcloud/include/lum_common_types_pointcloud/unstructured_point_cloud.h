// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_POINTCLOUD_POINTCLOUD_H
#define LUM_COMMON_TYPES_POINTCLOUD_POINTCLOUD_H

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

/// @brief Class fo unstructured point clouds
/// @note Use this class to create an unstuctured point cloud of type <T>.
template <typename T>
class UnstructuredPointCloud : public BasePointCloud<T>
{
public:
  /// @brief Standard constructor
  UnstructuredPointCloud() = default;

  /// @brief Constructor with initial vector capacity
  explicit UnstructuredPointCloud(const std::size_t capacity)
      : BasePointCloud<T>::BasePointCloud(capacity)
  {
  }

  /// @brief reference operator of the unstructured point cloud
  /// @param[in] index index of the element to be referenced
  /// @return a reference to the <T> element of the unstructured point cloud at index position
  T& operator[](const std::size_t index)
  {
    if (index >= this->points_.size())
    {
      throw exceptions::PointCloudIndexOutOfBoundsException(index, 0U, this->points_.size());
    }

    return this->points_[index];
  }

  /// @brief reference operator of the unstructured point cloud (const)
  /// @param[in] index index of the element to be referenced
  /// @return a reference to the <T> element of the unstructured point cloud at index position
  const T& operator[](const std::size_t index) const
  {
    if (index >= this->points_.size())
    {
      throw exceptions::PointCloudIndexOutOfBoundsException(index, 0U, this->points_.size());
    }

    return this->points_[index];
  }

  /// @brief resizes the point cloud
  /// @param[in] size size that the point cloud should be resized to
  void resize(const std::size_t size) { this->points_.resize(size); }

  /// @brief Adds a point of type <T> to the end of the point cloud
  /// @param[in] point typ <T> to be added
  void add(const T& point) { this->points_.emplace_back(point); }

  /// @brief Adds the series of points from the given container to the end of the point cloud
  /// @tparam ContainerType the type of container the points are being added from.
  /// @param[in] points the points to be added
  template <class ContainerType>
  void addMany(const ContainerType& points)
  {
    this->points_.insert(this->points_.end(), points.begin(), points.end());
  }

  /// @brief Adds the series of points from the given container to the end of the point cloud at the
  /// given position, pushing existing points back in the point cloud
  /// @tparam ContainerType the type of container the points are being added from.
  /// @param[in] points the points to be added
  /// @param[in] position the 0-indexed position to insert the points into
  template <class ContainerType>
  void addManyAt(const ContainerType& points, const std::size_t position)
  {
    this->points_.insert(this->points_.begin() + position, points.begin(), points.end());
  }

  /// @brief Erases points of type <T> from the beginning of the point cloud
  /// @param[in] number_of_points number of elements to be erased
  void erase(const std::size_t number_of_points)
  {
    if (number_of_points <= this->points_.size())
    {
      this->points_.erase(this->points_.begin(), this->points_.begin() + number_of_points);
    }
    else
    {
      this->points_.clear();
    }
  }

  /// @brief Erases points by a given vector of indices
  /// @param[in] erase_indices vector of indicies of points to be erased
  void eraseByListOfIndices(std::vector<std::size_t> erase_indices)
  {
    // Make sure the indices are sorted high-to-low and unique
    std::sort(std::begin(erase_indices), std::end(erase_indices), [](std::size_t a, std::size_t b) {
      return a > b;
    });
    const auto last_index = std::unique(std::begin(erase_indices), std::end(erase_indices));

    std::for_each(std::begin(erase_indices), last_index, [this](const std::size_t index) {
      const auto erasure_iter = std::begin(this->points_) + index;
      if (erasure_iter < std::end(this->points_))
      {
        this->points_.erase(erasure_iter);
      }
    });
  }

  /// @brief Erases point <T> at the given index
  /// @param[in] index index of the point <T> to be ereased
  void eraseByIndex(std::size_t index)
  {
    if (index >= this->points_.size())
    {
      throw exceptions::PointCloudIndexOutOfBoundsException(index, 0U, this->points_.size());
    }
    this->points_.erase(this->points_.begin() + index);
  }

  /// @brief Returns the size of the point cloud (const)
  std::size_t size() const override { return this->points_.size(); }

  using BasePointCloud<T>::at;
  using BasePointCloud<T>::atNoCheck;
};

/// @brief Shared pointer to UnstructuredPointCloud of type <T>
template <typename T>
using UnstructuredPointCloudPtr = std::shared_ptr<UnstructuredPointCloud<T>>;

/// @brief Shared const pointer to UnstructuredPointCloud of type <T>
template <typename T>
using UnstructuredPointCloudConstPtr = std::shared_ptr<const UnstructuredPointCloud<T>>;

} // namespace point_cloud
} // namespace types
} // namespace common
} // namespace lum

#endif
