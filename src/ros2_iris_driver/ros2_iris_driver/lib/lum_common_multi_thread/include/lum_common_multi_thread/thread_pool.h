// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MULTI_THREAD_THREAD_POOL_H
#define LUM_COMMON_MULTI_THREAD_THREAD_POOL_H

#include <atomic>
#include <functional>
#include <list>
#include <system_error>
#include <thread>

namespace lum {
namespace common {
namespace multi_thread {
namespace thread_pool {

/// @brief Thread pool will run a single given function N times over N threads
/// @code
/// const auto worker = []( const std::size_t index, std::atomic_bool& active ){ <do-something> };
/// Pool p(worker, 4);
/// @endcode
class Pool
{
  /// @brief The threadpool function should take as input a size_t which will equal the index of the
  /// given thread in the pool. The threadpool function should return void.
  /// @param [in] index The thread index
  /// @param [in] active Whether the thread pool is active. Functions should quickly exit when this
  /// flag is false.
  using ThreadPoolFunction = std::function<void(const std::size_t index, std::atomic_bool& active)>;

public:
  /// @brief Pool default constructor. No thread function has been provided so a call to
  /// initThreadPool will be subsequently required
  /// @return Pool object
  Pool() = default;

  /// @brief Destructor.
  /// @details Deactivates the pool and waits for all threads to complete.
  ~Pool() { shutdown(); }

  /// @brief Pool constructor
  /// @param [in] func inner function to run in every thread
  /// @param [in] num_threads number of threads in the pool
  /// @return Pool object
  Pool(const ThreadPoolFunction& func, const std::size_t num_threads)
  {
    // init N threads to run func
    initThreadPool(func, num_threads);
  }

  /// @brief Initialize the pool with N threads running func
  /// @note any threads currently in the pool will be terminated and destroyed
  /// @post The thread pool will be active
  /// @param [in] func inner function to run in every thread
  /// @param [in] num_threads number of threads in the pool
  void initThreadPool(const ThreadPoolFunction& func, const std::size_t num_threads)
  {
    if (!pool_.empty())
    {
      shutdown();
    }

    alive_ = true;

    for (std::size_t thread_index{0}; thread_index < num_threads; ++thread_index)
    {
      // Add new thread, passing thread_index as argument to thread function
      pool_.emplace_back(func, thread_index, std::ref(alive_));
    }
  }

  /// @brief Terminates and removes all threads from the pool.
  /// @post The thread pool will be inactive, call #initThreadPool to start it again.
  void shutdown() noexcept
  {
    alive_ = false;

    for (auto& thread : pool_)
    {
      if (thread.joinable())
      {
        try
        {
          thread.join();
        }
        catch (const std::system_error& e)
        {
          // Catch only the exception type expected from std::thread::join
        }
      }
    }

    pool_.clear();
  }

private:
  std::list<std::thread> pool_;  ///< List of threads
  std::atomic_bool alive_{true}; ///< Whether the thread pool is active
};

} // namespace thread_pool
} // namespace multi_thread
} // namespace common
} // namespace lum

#endif
