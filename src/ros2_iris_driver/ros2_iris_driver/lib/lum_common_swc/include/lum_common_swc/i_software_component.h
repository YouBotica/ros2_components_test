// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_I_SOFTWARE_COMPONENT_H
#define LUM_COMMON_SWC_I_SOFTWARE_COMPONENT_H

#include <cstddef>
#include <memory>
#include <string>

#include <lum_common_types/common.h>

namespace lum {
namespace common {

/// @brief Software layering framework
namespace swc {

/// @brief Software component interface.
///
/// A software component is a functional entity performing a reusable service to a module.
/// It uses resources which are handed in by the module. It is the creator and owner of
/// software units. It is supposed to be complete in functionality, so it can be tested
/// by using mocks of the resources or other software component pointers handed in.
///
/// The methods of this interface are operated in three nested pairs by the software
/// module. It expects the following to happen:
///
/// - Ctor / dtor: acquire/release resources from pointers handed in, create owned units
/// - init() / shutdown(): configure/reset the component by applying settings, setting up
///   owned units, verifying consistency or connections
/// - start() / stop(): launch/terminate controlled processing, get everything ready for
///   executing the actual task
///
/// The component can expect that the module operates the methods in sequence:
/// init() - start() - run() - stop() - shutdown()
/// with run() calling step() when scheduled by the component runtime or by a module-level
/// scheduler. It is considered wrong module behavior to not call stop() or shutdown() before
/// destroying the component:
///
/// - If init() returns false, the module can destroy the component immediately
/// - If start() returns false, the module needs to call shutdown() before destroying
/// - If start() returns true, the module needs to call stop() and shutdown()
///
/// The module can perform repeated partial state transition cycles:
///
/// - to suspend execution: start() - stop() - start() - stop()...
/// - to reconfigure the component: init() - start() - stop() - shutdown() - init() - start()...
///
/// The component implementation needs to be prepared for this, especially internal state or
/// the state of owned units must be handled correctly.
///
/// @sa SoftwareComponentBase @ref swc_runtime
class ISoftwareComponent
{
public:
  /// @brief A handle to a software component which maintains the ownership
  using Handle = std::unique_ptr<ISoftwareComponent>;

  /// @brief Performance statistics data
  struct Stats
  {
    /// Number of bytes sent via SWC communication
    std::size_t bytes_sent{0U};
    /// Number of bytes received from SWC communication
    std::size_t bytes_received{0U};
    /// Wall clock duration of the step() call
    types::Time wall_clock_step_duration{types::Time::zero()};
    /// CPU time duration of the step() call
    types::Time cpu_time_step_duration{types::Time::zero()};
  };

  /// @brief Acquires external resources from pointers handed in, creates owned units.
  ISoftwareComponent() = default;

  /// @brief Releases acquired external resources, destroys owned units.
  virtual ~ISoftwareComponent() = default;

  ISoftwareComponent(const ISoftwareComponent&) = delete;
  ISoftwareComponent(ISoftwareComponent&&) = delete;
  ISoftwareComponent& operator=(const ISoftwareComponent&) & = delete;
  ISoftwareComponent& operator=(ISoftwareComponent&&) & = delete;

  /// @brief Initialize the component by applying settings, setting up
  /// owned units, verifying consistency or connections.
  /// @return True if initialization succeeds, implying shutdown() is required in the future.
  virtual bool init() = 0;

  /// @brief Shutdown the component.
  ///
  /// Releases everything that has been configured by init().
  /// @return True if shutdown succeeds. If false, the module should not call init() again,
  /// but rather should destroy this object.
  virtual bool shutdown() = 0;

  /// @brief Start the component by launching controlled processing,
  /// and getting everything ready so step() can be called.
  /// @return True if starting succeeds
  virtual bool start() = 0;

  /// @brief Stop the component by terminating controlled processing.
  ///
  /// After stopping, step() cannot be called anymore.
  /// @return True if stopping succeeds. If false, start() should not be called again.
  /// Rather, shutdown() / init() should be called to re-initialize or the object should
  /// be destroyed after calling shutdown().
  virtual bool stop() = 0;

  /// @brief Step the component, i.e., perform the task of the component.
  ///
  /// This function will be called repeatedly to perform processing.
  /// Normally, this function is not called directly, but through the run() method or by a
  /// module-level scheduler.
  /// @return True if step succeeds and the component can be stepped again
  virtual bool step() = 0;

  /// @brief Run the component.
  ///
  /// The actual behavior is dependent on the runtime's properties
  /// but usually calls step() repeatedly until it returns false or processing is stopped.
  virtual void run() = 0;

  /// @brief Get the software component name.
  /// @return The software component name. Mostly useful for debugging and logging purposes.
  virtual const std::string& getName() const = 0;

  /// @brief Get current performance data
  /// @return Reference to current performance data
  virtual const Stats& getStats() const = 0;
  /// @brief Zero current performance data
  virtual void resetStats() = 0;

  /// @brief Get peak performance data
  /// @return Reference to peak performance data
  virtual const Stats& getPeakStats() const = 0;
  /// @brief Zero peak performance data
  virtual void resetPeakStats() = 0;

  /// @brief Set step() performance statistics
  /// @param[in] wall_clock_duration Time of the step() call as wall clock duration
  /// @param[in] cpu_duration Time of the step() call as CPU time duration
  virtual void setStepTimeStats(const types::Time& wall_clock_duration,
                                const types::Time& cpu_duration) = 0;
};

} // namespace swc
} // namespace common
} // namespace lum
#endif
