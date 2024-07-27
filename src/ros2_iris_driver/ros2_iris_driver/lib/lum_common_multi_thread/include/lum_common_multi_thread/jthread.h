// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MULTI_THREAD_JTHREAD_H
#define LUM_COMMON_MULTI_THREAD_JTHREAD_H

#include <thread>
#include <utility>

namespace lum {
namespace common {
namespace multi_thread {
namespace detail {

///////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Simulates C++20's std::jthread by joining a Thread instance on destruction.
/// @tparam The thread type, injectable for testing.
template <class Thread>
class JThreadImpl
{
public:
  /// @brief Constructs a thread running the callable object and passing the arguments to it
  /// @tparam Function A callable type
  /// @tparam Args The optional argument types
  /// @param function The callable object
  /// @param args The arguments to be passed to the callable object
  template <class Function, class... Args>
  explicit JThreadImpl(Function&& function, Args&&... args)
      : thread_{std::forward<Function>(function), std::forward<Args>(args)...}
  {
  }

  /// @brief Destructor -- joins the thread if needed
  ~JThreadImpl()
  {
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  /// @brief Get a reference to the thread used within the wrapper
  Thread& getThread() { return thread_; }

  /// @brief no copying
  JThreadImpl(const JThreadImpl&) = delete;
  /// @brief no copying
  JThreadImpl& operator=(const JThreadImpl&) = delete;

  /// @brief Move a JThreadImpl
  JThreadImpl(JThreadImpl&&) noexcept = default;
  /// @brief Move a JThreadImpl
  JThreadImpl& operator=(JThreadImpl&&) noexcept = default;

private:
  Thread thread_;
};
} // namespace detail

///////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Simulates C++20's std::jthread by joining a std::thread on destruction.
using JThread = detail::JThreadImpl<std::thread>;

} // namespace multi_thread
} // namespace common
} // namespace lum
#endif
