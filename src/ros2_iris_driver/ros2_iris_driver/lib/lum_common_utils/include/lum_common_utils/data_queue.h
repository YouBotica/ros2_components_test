// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_DATA_QUEUE_H
#define LUM_COMMON_UTILS_DATA_QUEUE_H

#include <cstdint>
#include <deque>
#include <vector>

namespace lum {
namespace common {
namespace utils {

/// @brief Class to store a detections which will be used to get fused horizon output
template <class T>
class DataQueue
{
public:
  /// @brief Constructor
  /// @@param [in] capacity Capacity of the queue
  explicit DataQueue(const std::size_t capacity = 10) { queue_capacity_ = capacity; }

  /// @brief Method to feed new data to the queue
  /// This method is not thread safe so make to lock the respective thread before using this method
  /// @param [in] data New data
  void feedNewData(const T& data)
  {
    if (queue_.size() == queue_capacity_)
    {
      queue_.pop_front();
    }
    queue_.push_back(data);
  }

  /// @brief Method to get measurements in the buffer
  /// This method is not thread safe so make to lock the respective thread before using this method
  /// @return returns a vector of measurements
  std::vector<T> getQueueData() const
  {
    // return measurements;
    return {queue_.begin(), queue_.end()};
  }

  /// @brief Method to check if queue is empty
  /// @return returns true if queue is empty
  bool empty() const { return queue_.empty(); }

  /// @brief Method to get size of horizon queue
  /// @return returns the size of the horizon queue
  std::size_t size() const { return queue_.size(); }

  /// @brief Method to set queue capacity
  /// @param [in] cpapcity of the horizon queue
  void setCapacity(std::size_t capacity) { queue_capacity_ = capacity; }

  /// @brief Method to get queue capacity
  /// @return cpapcity of the horizon queue
  std::size_t getCapacity() const { return queue_capacity_; }

  /// @brief Method to reset the horizon queue
  void reset() { queue_.clear(); }

private:
  std::size_t queue_capacity_{10}; ///< max capacity of the queue
  std::deque<T> queue_;            ///< Horizon queue
};
} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_DATA_QUEUE_H
