// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_MESSAGE_QUEUE_H
#define LUM_COMMON_UTILS_MESSAGE_QUEUE_H

#include <mutex>
#include <queue>

#include <lum_common_types/result.h>
#include <lum_common_utils/i_message_queue.h>

namespace lum {
namespace common {
namespace utils {

/// @brief A thread-safe message queue
/// @tparam T type to be queued
template <class T>
class MessageQueue : public IMessageQueue
{
public:
  template <class A, class B>
  using Result = lum::common::types::Result<A, B>;

  /// @brief Adds a value to the queue
  /// @param data is the input to be stored
  /// @return true in case of success
  bool push(T data)
  {
    std::lock_guard<std::mutex> guard{mutex_};
    queue_.emplace(std::move(data));
    return true;
  }

  /// @brief This function removes the current front from the queue and returns it within a result
  /// @return returns <T, true> if queue is empty; <T, false> in case of success
  Result<T, bool> pop()
  {
    std::lock_guard<std::mutex> guard{mutex_};
    if (queue_.empty())
    {
      return {{}, true};
    }
    Result<T, bool> result{std::move(queue_.front()), false};
    queue_.pop();
    return result;
  }

private:
  std::queue<T> queue_{};
  std::mutex mutex_{};
};

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_MESSAGE_QUEUE_H
