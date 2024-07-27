// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MULTI_THREAD_WORK_QUEUE_H
#define LUM_COMMON_MULTI_THREAD_WORK_QUEUE_H

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

#include <lum_common_exceptions/init.h>

namespace lum {
namespace common {
namespace multi_thread {
namespace detail {

constexpr std::chrono::milliseconds DEFAULT_TIMEOUT{100U};
constexpr std::chrono::milliseconds NO_WAIT{0};

template <typename ItemType>
class ThreadSafeQueueBase
{
public:
  bool isEmpty() const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  std::pair<ItemType, bool> pop() { return pop(NO_WAIT); }

  std::pair<ItemType, bool> waitAndPop(const std::chrono::milliseconds timeout_ms = DEFAULT_TIMEOUT)
  {
    return pop(timeout_ms);
  }

  /// Removes all elements from the queue.
  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // std::queue doesn't have a clear method, so just swap with an empty queue.
    std::queue<ItemType>().swap(queue_);
  }

  // Not copyable or movable
  ThreadSafeQueueBase(const ThreadSafeQueueBase&) = delete;
  ThreadSafeQueueBase(ThreadSafeQueueBase&&) = delete;
  void operator=(const ThreadSafeQueueBase&) & = delete;
  void operator=(ThreadSafeQueueBase&&) & = delete;

protected:
  std::condition_variable data_available_;
  mutable std::mutex mutex_;
  std::queue<ItemType> queue_;

  // Creation and destruction is protected since this is not a polymorphic base class
  ThreadSafeQueueBase() = default;
  ~ThreadSafeQueueBase() = default;

private:
  std::pair<ItemType, bool> pop(const std::chrono::milliseconds timeout)
  {
    ItemType oldest_item{};

    {
      std::unique_lock<std::mutex> lock(mutex_);

      // Wait until the work queue is not empty or a timeout occurs. If a timeout occurs, return an
      // empty item.
      if (queue_.empty())
      {
        // wait_for() will return false if a timeout occurs and the queue is still empty.
        if (timeout == NO_WAIT ||
            !data_available_.wait_for(lock, timeout, [this] { return !queue_.empty(); }))
        {
          return {std::move(oldest_item), false};
        }
      }

      // pop the oldest element off the queue
      oldest_item = std::move(queue_.front());
      queue_.pop();
    }

    return {std::move(oldest_item), true};
  }
};
} // namespace detail

/// @brief A thread-safe queue.
/// @tparam ItemType The type stored in the queue. Must be default-constructible.
/// @code
/// ThreadSafeQueue<int> q;
///
/// // start a thread that processes items on the queue
/// std::thread t([&q](){
///   while (true) {
///      const auto pop = q.waitAndPop();
///      if( pop.second )
///      {
///         // ... do something with pop.first
///      }
///   }
/// });
///
///
/// q.push(5);
/// q.push(10);
/// @endcode
template <typename ItemType>
class ThreadSafeQueue final : public detail::ThreadSafeQueueBase<ItemType>
{
public:
  ThreadSafeQueue() = default;

  /// Adds @a item to the back of the queue
  void push(ItemType item)
  {
    bool was_empty{};

    {
      std::lock_guard<std::mutex> lock{this->mutex_};
      was_empty = this->queue_.empty();
      this->queue_.push(std::move(item));
    }

    if (was_empty)
    {
      // Alert waiting threads that a new item is available
      this->data_available_.notify_all();
    }
  }
};

/// A queue with a maximum capacity. New elements are dropped if full.
template <typename ItemType>
class ThreadSafeCapacityQueue final : public detail::ThreadSafeQueueBase<ItemType>
{
public:
  /// @brief Constructs a queue with max capacity of @a capacity items
  /// @throw InvalidParameterException if capacity is zero
  explicit ThreadSafeCapacityQueue(std::size_t capacity) : capacity_{capacity}
  {
    if (capacity == 0U)
    {
      throw exceptions::InvalidParameterException{
        "ThreadSafeCapacityQueue constructor: capacity cannot be zero"};
    }
  }

  /// Adds @a item to the back of the queue
  /// @return true if added, false if failed because queue was full
  bool push(ItemType item)
  {
    bool was_empty{};

    {
      std::lock_guard<std::mutex> lock(this->mutex_);

      // if there is no space just return, dropping this item
      if (this->queue_.size() == capacity_)
      {
        return false;
      }

      was_empty = this->queue_.empty();
      this->queue_.push(std::move(item));
    }

    if (was_empty)
    {
      // Alert waiting threads that a new item is available
      this->data_available_.notify_all();
    }

    return true;
  }

  /// Returns whether queue is currently at capacity
  bool isFull() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->queue_.size() == capacity_;
  }

private:
  const std::size_t capacity_;
};

} // namespace multi_thread
} // namespace common
} // namespace lum

#endif
