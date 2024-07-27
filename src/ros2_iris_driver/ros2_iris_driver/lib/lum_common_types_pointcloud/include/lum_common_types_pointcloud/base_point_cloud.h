// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_BASE_POINTCLOUD_POINTCLOUD_H
#define LUM_COMMON_TYPES_BASE_POINTCLOUD_POINTCLOUD_H

#include <algorithm>
#include <cstring>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

#include <lum_common_exceptions/init.h>
#include <lum_common_types/common.h>

namespace lum {
namespace common {
namespace types {
namespace point_cloud {

namespace exceptions = lum::common::exceptions;

/// @namespace lum::common::types::point_cloud Point cloud data types

/// @brief Structure for representing a point cloud
template <typename T>
using PointCloud = std::vector<T>;

/// @brief Shared pointer to PointCloud of type T
template <typename T>
using PointCloudPtr = std::shared_ptr<PointCloud<T>>;

/// @brief Shared const pointer to PointCloud of type T
template <typename T>
using PointCloudConstPtr = std::shared_ptr<const PointCloud<T>>;

/// @brief Base class of the structured and unstructured point cloud
/// @note This class serves as base class of the structured and unstructured
/// point cloud and can't be instantiated.
template <typename T>
class BasePointCloud
{
protected:
  /// @brief points_ internal vector that holds <T> of the point cloud
  std::vector<T> points_{0};

  /// @brief Timestamp of the current pointcloud
  Time timestamp_{};

  /// @brief Standard constructor
  BasePointCloud() = default;

  /// @brief Constructor with initial vector size
  explicit BasePointCloud(const std::size_t size) : points_(size) {}

  virtual ~BasePointCloud() = default;

public:
  BasePointCloud(const BasePointCloud&) = default;
  BasePointCloud(BasePointCloud&&) noexcept = default;
  BasePointCloud& operator=(const BasePointCloud&) & = default;
  BasePointCloud& operator=(BasePointCloud&&) & noexcept = default;
  /// @brief Type alias for the iterator
  using iterator = typename std::vector<T>::iterator;

  /// @brief Type alias for the iterator for const <T>
  using const_iterator = typename std::vector<T>::const_iterator;

  /// @brief Type alias for the reverse iterator
  using reverse_iterator = typename std::vector<T>::reverse_iterator;

  /// @brief Type alias for the reverse iterator for const <T>
  using const_reverse_iterator = typename std::vector<T>::const_reverse_iterator;

  /// @brief  get the point cloud begin iterator
  /// @return Returns an iterator that points to the begin of the point cloud
  iterator begin() { return points_.begin(); }

  /// @brief get the point cloud end iterator
  /// @return Returns an iterator that points to the end of the point cloud
  iterator end() { return points_.end(); }

  /// @brief  get the point cloud begin iterator (const)
  /// @return Returns an iterator that points to the begin of the point cloud (const)
  const_iterator begin() const { return points_.cbegin(); }

  /// @brief get the point cloud end iterator (const)
  /// @return Returns an iterator that points to the end of the point cloud (const)
  const_iterator end() const { return points_.cend(); }

  /// @brief get the point cloud end reverse iterator
  /// @return Returns a reverse iterator that points to the end of the point cloud
  reverse_iterator rbegin() { return points_.rbegin(); }

  /// @brief get the point cloud begin reverse iterator
  /// @return Returns a reverse iterator that points to the begin of the point cloud
  reverse_iterator rend() { return points_.rend(); }

  /// @brief get the point cloud end reverse iterator (const)
  /// @return Returns a reverse iterator that points to the end of the point cloud (const)
  const_reverse_iterator rbegin() const { return points_.crbegin(); }

  /// @brief get the point cloud begin reverse iterator (const)
  /// @return Returns a reverse iterator that points to the begin of the point cloud (const)
  const_reverse_iterator rend() const { return points_.crend(); }

  /// @brief Returns a reference to the point at the given index of the point cloud
  /// @param[in] index index of the point to be returned
  /// @return Reference to the point at the given index of the point cloud
  T& at(const std::size_t index)
  {
    if (index >= this->points_.size())
    {
      throw exceptions::PointCloudIndexOutOfBoundsException(index, 0U, this->points_.size());
    }

    return this->points_.at(index);
  }

  /// @brief Returns a reference to the point at the given index of the point cloud (const)
  /// @param[in] index index of the point to be returned
  /// @return Reference to the point at the given index of the point cloud
  const T& at(const std::size_t index) const
  {
    if (index >= this->points_.size())
    {
      throw exceptions::PointCloudIndexOutOfBoundsException(index, 0U, this->points_.size());
    }

    return this->points_.at(index);
  }

  /// @brief Returns a reference to the point at the given index of the point cloud without
  /// performing a bounds check
  /// @param[in] index index of the point to be returned
  /// @return Reference to the point at the given index of the point cloud
  T& atNoCheck(const std::size_t index)
  {
    // NOLINTNEXTLINE remove bounds checking for performance
    return this->points_[index];
  }

  /// @brief Returns a reference to the point at the given index of the point cloud without
  /// performing a bounds check (const)
  /// @param[in] index index of the point to be returned
  /// @return Reference to the point at the given index of the point cloud
  const T& atNoCheck(const std::size_t index) const
  {
    // NOLINTNEXTLINE remove bounds checking for performance
    return this->points_[index];
  }

  /// @brief get the point cloud capacity
  /// @return Returns the capacity of the point cloud (const)
  std::size_t capacity() const { return this->points_.capacity(); }

  /// @brief size of the point cloud (const)
  /// @return Returns the size of the point cloud (const)
  virtual std::size_t size() const = 0;

  /// @brief clear the unstructured point cloud
  void clear() { this->points_.clear(); }

  /// @brief resets the unstructured point cloud with the given value
  /// @param[in] value <T> that is used to initialize the unstructured point cloud
  void reset(const T& value) { this->points_.assign(this->points_.size(), value); }

  /// @brief reserves space for the point cloud
  /// @param[in] size size of the space to be reserved
  void reserve(const std::size_t size) { this->points_.reserve(size); }

  /// @brief Check if the pointcloud is empty
  /// @return True if the pointcloud has no points, else False
  bool empty() const { return this->points_.empty(); }

  /// @brief get the capacity of the point cloud (const)
  /// @return Returns the capacity of the point cloud (const)
  std::size_t bufferSize() const { return sizeof(T) * this->points_.size(); }

  /// @brief Copies raw pointcloud data to the buffer.
  /// @note dest buffer size must be larger or equal to this->bufferSize()
  /// @param[out] dest - destination buffer (array of unsigned char)
  inline void serializeToBuffer(std::uint8_t* dest) const
  {
    std::memcpy(dest, this->points_.data(), this->bufferSize());
  }

  /// @brief Copies raw pointcloud data from buffer.
  /// @note cloud memory must be allocated beforehand
  /// @param[in] src - source buffer (array of unsigned char)
  /// @param[in] bytes_count - number of bytes to copy
  /// @param[in] dest_offset - destination pointcloud offset
  /// @param[in] src_offset - source buffer offset
  inline void deserializeFromBuffer(const std::uint8_t* src,
                                    const std::size_t bytes_count,
                                    const std::size_t dest_offset = 0U,
                                    const std::size_t src_offset = 0U)
  {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    std::memcpy(this->points_.data() + dest_offset, src + src_offset, bytes_count);
  }

  /// @brief points to first element
  /// @return return a pointer to first element
  inline T* data() { return this->points_.data(); }

  /// @brief points to first element
  /// @return return a pointer to first element
  const T* data() const { return this->points_.data(); }

  /// @brief current pointcloud timestamp
  /// @return return a pointer to first element
  Time getTimestamp() const { return this->timestamp_; }

  /// @brief set current pointcloud timestamp
  void setTimestamp(const Time& timestamp) { this->timestamp_ = timestamp; }
};

} // namespace point_cloud
} // namespace types
} // namespace common
} // namespace lum

#endif
