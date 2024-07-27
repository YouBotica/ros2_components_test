// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_RUNTIME_H
#define LUM_COMMON_SWC_RUNTIME_H

#include <atomic>
#include <chrono>
#include <thread>

#include <lum_common_swc/i_clock.h>
#include <lum_common_swc/i_software_component.h>

namespace lum {
namespace common {

namespace multi_thread {
namespace detail {
template <typename>
class JThreadImpl;
} // namespace detail
using JThread = detail::JThreadImpl<std::thread>;
} // namespace multi_thread

namespace swc {

/// @brief Software component runtime
namespace runtime {

/// @page swc_runtime Runtime policies for software components
///
/// The policy should differentiate the runtime behavior of the
/// main loop of the software component @c run() method so it can be
/// used for several application cases.
///
/// Semantics:
///
/// - @c init(): setup any necessary signals or conditions for threading,
///   health watchdogs, etc. If it returns true, it's safe to execute the
///   @c run() method of the software component. Else the software component
///   cannot be stepped and should thus be reconfigured or destroyed by
///   the module.
/// - @c shutdown(): stop any threading. If it returns true, the @c run() method
///   of the software component has exited and @c start() must be called again
///   before next call to @c run(). Else, the runtime cannot be terminated
///   properly and the module must destroy the software component.
/// - @c sleep(): waits for some time. The main loop calls @c sleep() after each
///   call to @c step() of the software component. Use this method to implement
///   throttled cyclic stepping or to cause stepping to wait on an external
///   signal for example.
/// - @c setDelay(): set the amount of time @c sleep() should pause. The argument
///   allows for the maximum supported time resolution, but might be limited by the
///   actual clock used scheduling.
/// - @c setClock(): set the clock to use for timed operations like the @c sleep()
///   call. If this has not been called, the native system clock is used.
/// - @c shouldExit(): must return true if the main loop should exit normally
///   on the next @c step() cycle.
/// - @c execute(): perform the main loop. This possibly starts a separate thread.

/// @brief Module stepping runtime policy
///
/// This policy will perform a single @c step() only for each call to the @c run() method of the
/// software component using the current processing thread. It is thus up to the module-level to
/// schedule repeated stepping.
/// @sa @ref swc_runtime
class ModuleStepping final
{
public:
  static bool init() noexcept { return true; }
  static bool shutdown() noexcept { return true; }
  static void sleep() noexcept
  {
    // empty by design
  }
  static void setDelay(types::Time) noexcept {}
  static void setClock(const IClock&) noexcept {}
  static bool shouldExit() noexcept { return true; }
  static void execute(ISoftwareComponent& component);
};

/// @brief Threaded runtime policy
///
/// This policy will make the main loop spin in an independent thread when the
/// @c run() method of the software component is called. The @c run() method will return
/// immediately and the @c step() method of the software component is called while
/// sleeping with the configured delay in between.
/// @sa @ref swc_runtime
class RunThreaded final
{
public:
  RunThreaded() noexcept;
  ~RunThreaded() noexcept;

  RunThreaded(const RunThreaded&) = delete;
  RunThreaded(RunThreaded&&) = delete;
  RunThreaded& operator=(const RunThreaded&) & = delete;
  RunThreaded& operator=(RunThreaded&&) & = delete;

  bool init();
  bool shutdown() noexcept;
  void sleep() const;
  void setDelay(const types::Time delay) noexcept { delay_ = delay; }
  void setClock(IClock& runtime_clock) noexcept { clock_ = &runtime_clock; }
  bool shouldExit() const noexcept { return should_exit_; }
  void execute(ISoftwareComponent& component);

private:
  std::unique_ptr<multi_thread::JThread> execution_thread_;
  types::Time delay_{types::Time::zero()};
  IClock* clock_{nullptr};
  std::atomic<bool> should_exit_{false};
};

/// @private
class ScheduleWithMaxPoolSize
{
public:
  explicit ScheduleWithMaxPoolSize(std::size_t max_num_workers) noexcept;
  ~ScheduleWithMaxPoolSize() noexcept;

  ScheduleWithMaxPoolSize(const ScheduleWithMaxPoolSize&) = delete;
  ScheduleWithMaxPoolSize(ScheduleWithMaxPoolSize&&) = delete;
  ScheduleWithMaxPoolSize& operator=(const ScheduleWithMaxPoolSize&) & = delete;
  ScheduleWithMaxPoolSize& operator=(ScheduleWithMaxPoolSize&&) & = delete;

  bool init();
  bool shutdown() noexcept;
  void sleep() const;
  void setDelay(const types::Time delay) noexcept { delay_ = delay; }
  void setClock(IClock& runtime_clock) noexcept { clock_ = &runtime_clock; }
  bool shouldExit() const noexcept { return should_exit_; }
  void execute(ISoftwareComponent& component);

private:
  std::unique_ptr<multi_thread::JThread> scheduler_thread_;
  types::Time delay_{types::Time::zero()};
  IClock* clock_{nullptr};
  const std::size_t max_num_workers_;
  std::atomic<bool> should_exit_{false};
};

/// @brief Threaded scheduled runtime policy
///
/// This policy will install an independent scheduler thread when the
/// @c run() method of the software component is called. The @c run() method will return
/// immediately and the scheduler calls the @c step() method of the software component at
/// the configured delay without waiting for it's completion. Therefore, if @c step() takes
/// longer than the configured delay period, more than one @c step() call can be in flight.
/// Thus you need to make sure that @c step() is reentrant.
/// @tparam max_num_workers The maximum number of concurrent @c step() calls (= thread pool size)
/// @sa @ref swc_runtime
template <std::size_t max_num_workers>
class ScheduleThreaded final : public ScheduleWithMaxPoolSize
{
public:
  ScheduleThreaded() noexcept : ScheduleWithMaxPoolSize{max_num_workers} {}
};

} // namespace runtime
} // namespace swc
} // namespace common
} // namespace lum
#endif
