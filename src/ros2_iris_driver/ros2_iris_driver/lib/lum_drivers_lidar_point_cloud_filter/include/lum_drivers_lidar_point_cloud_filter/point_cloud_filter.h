// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_POINT_CLOUD_FILTER_POINT_CLOUD_FILTER_H
#define LUM_DRIVERS_LIDAR_POINT_CLOUD_FILTER_POINT_CLOUD_FILTER_H

#include <iterator>
#include <stdexcept>

#include <lum_common_types_internal/tagged_bool.h>
#include <lum_common_types_pointcloud/base_point_cloud.h>
#include <lum_common_types_pointcloud/point_cloud_layer.h>
#include <lum_drivers_lidar_calibration/calibration.h>

namespace lum {
namespace drivers {
namespace lidar {

/// @brief a utility that provides the ability to filter a layered point cloud using 2
/// different filtering options. The first is a PointTypeFilter with the options: KEEP_VALID and
/// KEEP_ALL_POINTS. The second is a MultiReturnFilter with the options: ALL, FIRST, LAST, and
/// STRONGEST.
///
/// @code
/// common::types::point_cloud::UnstructuredCommonLayer common_layer{};
/// common::types::point_cloud::UnstructuredPolarLayer polar_layer{};
/// common::types::point_cloud::UnstructuredDebugLayer debug_layer{};
/// common::types::point_cloud::UnstructuredPointCloud<LidarData> lidar_layer{};
///
/// // .. populate the cloud with points
///
/// calibration::PointTypeFilter point_type_filter;     // KEEP_VALID or KEEP_ALL_POINTS
/// calibration::MultiReturnFilter multi_return_filter; // ALL, FIRST, LAST, or STRONGEST
///
/// PointCloudFilter<LidarData> point_cloud_filter(
///   point_type_filter, multi_return_filter, common_layer, polar_layer, debug_layer,
///   lidar_layer);
///
/// PointCloudFilter<LidarData>::CommonIterator common_it;
/// PointCloudFilter<LidarData>::PolarIterator polar_it;
/// PointCloudFilter<LidarData>::DebugIterator debug_it;
/// PointCloudFilter<LidarData>::LidarIterator lidar_it;
///
/// while (point_cloud_filter.nextPoint(common_it, polar_it, debug_it, lidar_it))
/// {
///   // .. use filtered layered point data
/// }
/// @endcode
template <typename LidarLayerType>
class PointCloudFilter
{
  using ShouldIncrement = common::types::TaggedBool<class ShouldIncrementTag>;

public:
  using CommonIterator = typename common::types::point_cloud::BaseCommonLayer::const_iterator;
  using PolarIterator = typename common::types::point_cloud::BasePolarLayer::const_iterator;
  using DebugIterator = typename common::types::point_cloud::BaseDebugLayer::const_iterator;
  using LidarIterator =
    typename common::types::point_cloud::BasePointCloud<LidarLayerType>::const_iterator;

  /// @brief Constructor.
  /// @param point_type_filter how to filter at the ray level
  /// @param multi_return_filter how to filter the returns within a ray
  /// @param common_layer common layer point cloud
  /// @param polar_layer polar layer point cloud
  /// @param debug_layer debug layer point cloud
  /// @param lidar_layer lidar layer point cloud
  PointCloudFilter(const calibration::PointTypeFilter point_type_filter,
                   const calibration::MultiReturnFilter multi_return_filter,
                   const common::types::point_cloud::BaseCommonLayer& common_layer,
                   const common::types::point_cloud::BasePolarLayer& polar_layer,
                   const common::types::point_cloud::BaseDebugLayer& debug_layer,
                   const common::types::point_cloud::BasePointCloud<LidarLayerType>& lidar_layer)
      : point_type_filter_{point_type_filter}
      , multi_return_filter_{multi_return_filter}
      , common_head_{common_layer.begin()}
      , common_tail_{common_layer.end()}
      , polar_head_{polar_layer.begin()}
      , polar_tail_{polar_layer.end()}
      , debug_head_{debug_layer.begin()}
      , debug_tail_{debug_layer.end()}
      , lidar_head_{lidar_layer.begin()}
      , lidar_tail_{lidar_layer.end()}
  {
    if (!areMatching(common_layer, polar_layer) || !areMatching(common_layer, debug_layer) ||
        !areMatching(common_layer, lidar_layer))
    {
      throw std::invalid_argument("Layers are non-matching");
    }

    // initialize the iterators to the first point that passes the filter
    pickNextPoint(ShouldIncrement{false});
  }

  /// @brief get the next layered point in the point cloud with the filter applied
  /// @param [out] common_iterator points to the next point in the common layer
  /// @param [out] polar_iterator points to the next point in the polar layer
  /// @param [out] debug_iterator points to the next point in the debug layer
  /// @param [out] lidar_iterator points to the next point in the lidar layer
  /// @return true if the iterators point to valid layered data and false if the end of the point
  /// cloud has been reached
  bool nextPoint(CommonIterator& common_iterator,
                 PolarIterator& polar_iterator,
                 DebugIterator& debug_iterator,
                 LidarIterator& lidar_iterator)
  {
    // already reached the end, return false
    if (common_head_ == common_tail_)
    {
      return false;
    }

    // set the out variables based on the previously selected point
    common_iterator = common_head_;
    polar_iterator = polar_head_;
    debug_iterator = debug_head_;
    lidar_iterator = lidar_head_;

    // select the next point for a subsequent call
    pickNextPoint(ShouldIncrement{true});

    return true;
  }

private:
  calibration::PointTypeFilter point_type_filter_;     ///< how we are filtering types of rays
  calibration::MultiReturnFilter multi_return_filter_; ///< how we are filtering ray returns

  CommonIterator common_head_; ///< initial start to the iterable set of the common layer
  CommonIterator common_tail_; ///< the end of the iterable set of the common layer

  PolarIterator polar_head_; ///< initial start to the iterable set of the polar layer
  PolarIterator polar_tail_; ///< the end of the iterable set of the polar layer

  DebugIterator debug_head_; ///< initial start to the iterable set of the debug layer
  DebugIterator debug_tail_; ///< the end of the iterable set of the debug layer

  LidarIterator lidar_head_; ///< initial start to the iterable set of the lidar layer
  LidarIterator lidar_tail_; ///< the end of the iterable set of the lidar layer

  /// @brief pick the next point
  /// @param should_increment whether to increment the iterator at the beginning of the function
  void pickNextPoint(const ShouldIncrement should_increment)
  {
    // keep track of where the common iterator is at the beginning so we can advance the other
    // iterators the same amount as we advance the common iterator when applying the filter.
    auto common_start = common_head_;

    // increment the iterator; we should never be at the end of the point cloud here, so no check is
    // required
    if (should_increment)
    {
      ++common_head_;
    }

    // point selection is a loop as we may need to skip and reselect points based on configuration
    // and input data
    while (common_head_ != common_tail_)
    {
      // select next candidate point
      switch (multi_return_filter_)
      {
      case calibration::MultiReturnFilter::STRONGEST:
        pickStrongest();
        break;

      case calibration::MultiReturnFilter::FIRST:
        pickFirst();
        break;

      case calibration::MultiReturnFilter::LAST:
        pickLast();
        break;

      case calibration::MultiReturnFilter::ALL:
      default:
        // no point filtering, we return them all
        break;
      }

      // the return filtering halts on empty
      // if we are filtering empty increment and re-select
      if (point_type_filter_ == calibration::PointTypeFilter::KEEP_VALID &&
          common_head_ != common_tail_ && common::types::point::isEmpty(*common_head_))
      {
        ++common_head_;
      }
      else
      {
        // not filtering empty, reached end, or head is not empty, stop looking
        break;
      }
    }

    // advance the other iterators by the same distance as common was advanced.
    const auto distance = std::distance(common_start, common_head_);
    std::advance(polar_head_, distance);
    std::advance(debug_head_, distance);
    std::advance(lidar_head_, distance);
  }

  /// @brief function holding complex strongest point logic, selects next empty or strongest point
  void pickStrongest()
  {
    // middle of a set of points; move head to beginning of the next set of points
    if (common_head_->return_index > 1U)
    {
      const auto current_index = common_head_->return_index;
      ++common_head_;
      while (common_head_ != common_tail_ && common_head_->return_index > current_index)
      {
        ++common_head_;
      }
    }

    // If we've reached the end or the next point is empty, return.
    if (common_head_ == common_tail_ || common_head_->return_index == 0U)
    {
      return;
    }

    // start of a set of points; find strongest
    auto itr = std::next(common_head_);
    while (itr != common_tail_ && itr->return_index > common_head_->return_index)
    {
      if (itr->reflectance > common_head_->reflectance)
      {
        common_head_ = itr;
      }
      ++itr;
    }
  }

  /// @brief function holding logic for the first return only filter
  void pickFirst()
  {
    // iterate until we reach next empty point or first return
    while (common_head_ != common_tail_ && !common::types::point::isEmpty(*common_head_) &&
           common_head_->return_index != 1U)
    {
      ++common_head_;
    }
  }

  /// @brief function holding logic for the last return only filter
  void pickLast()
  {
    // iterate until we reach next empty point or last return
    while (common_head_ != common_tail_ && !common::types::point::isEmpty(*common_head_) &&
           common_head_->return_index != common_head_->last_return_index)
    {
      ++common_head_;
    }
  }
};

} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_POINT_CLOUD_FILTER_POINT_CLOUD_FILTER_H
