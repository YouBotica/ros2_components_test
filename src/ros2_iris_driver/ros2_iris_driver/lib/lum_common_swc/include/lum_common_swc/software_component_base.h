// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_SOFTWARE_COMPONENT_BASE_H
#define LUM_COMMON_SWC_SOFTWARE_COMPONENT_BASE_H

#include <algorithm>
#include <string>
#include <type_traits>
#include <utility>

#include <lum_common_swc/i_clock.h>
#include <lum_common_swc/i_software_component.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Software component default implementation.
template <class Runtime>
class SoftwareComponentBase : public ISoftwareComponent
{
public:
  /// @brief Create a SWC with basic functionality.
  /// @tparam Runtime The SWC runtime to use.
  /// @param[in] name Name of the SWC, most useful for reporting.
  explicit SoftwareComponentBase<Runtime>(std::string name) noexcept
      : ISoftwareComponent{}, name_{std::move(name)}
  {
    static_assert(std::is_final<Runtime>::value, "Runtime policy class must be final");
    static_assert(std::is_default_constructible<Runtime>::value,
                  "Runtime policy class must be default constructible");
    static_assert(std::is_standard_layout<Runtime>::value,
                  "Runtime policy class must be standard layout");
  }

  /// @copydoc ISoftwareComponent::init
  bool init() override { return true; }

  /// @copydoc ISoftwareComponent::shutdown
  bool shutdown() override { return true; }

  /// @copydoc ISoftwareComponent::start
  bool start() override { return runtime_.init(); }

  /// @copydoc ISoftwareComponent::stop
  bool stop() override { return runtime_.shutdown(); }

  /// @copydoc ISoftwareComponent::step
  bool step() override { return true; }

  /// @copydoc ISoftwareComponent::run
  void run() override { runtime_.execute(*this); }

  /// @copydoc ISoftwareComponent::getName
  const std::string& getName() const noexcept override { return name_; }

  /// @copydoc ISoftwareComponent::getStats
  const Stats& getStats() const noexcept override { return stats_; }
  /// @copydoc ISoftwareComponent::resetStats
  void resetStats() noexcept override { stats_ = Stats{}; }

  /// @copydoc ISoftwareComponent::getPeakStats
  const Stats& getPeakStats() const noexcept override { return peak_stats_; }
  /// @copydoc ISoftwareComponent::resetPeakStats
  void resetPeakStats() noexcept override { peak_stats_ = Stats{}; }

  /// @copydoc ISoftwareComponent::setStepTimeStats
  void setStepTimeStats(const types::Time& wall_clock_duration,
                        const types::Time& cpu_duration) noexcept override
  {
    stats_.wall_clock_step_duration = wall_clock_duration;
    stats_.cpu_time_step_duration = cpu_duration;
    peak_stats_.wall_clock_step_duration =
      std::max(peak_stats_.wall_clock_step_duration, wall_clock_duration);
    peak_stats_.cpu_time_step_duration = std::max(peak_stats_.cpu_time_step_duration, cpu_duration);
  }

protected:
  /// @brief Allow to check for cancellation while doing work in step()
  /// @returns true when cancellation is requested otherwise false
  bool shouldExitStep() { return runtime_.shouldExit(); }

  /// @brief Configure runtime main loop delay
  /// @param[in] delay Time value to delay
  void setRuntimeDelay(types::Time delay) { runtime_.setDelay(delay); }

  /// @brief Set clock the runtime should use for time-related operations
  /// @param[in] clock The clock to use
  void setRuntimeClock(IClock& clock) { runtime_.setClock(clock); }

  /// @brief Accumulate Sender performance statistics
  /// @param[in] sender The sender
  template <template <typename> class S, typename T>
  void addSenderStats(const S<T>& sender)
  {
    stats_.bytes_sent += sender.getBytesSent();
    peak_stats_.bytes_sent = std::max(peak_stats_.bytes_sent, stats_.bytes_sent);
  }

  /// @brief Accumulate Receiver performance statistics
  /// @param[in] receiver The receiver
  template <template <typename> class R, typename T>
  void addReceiverStats(const R<T>& receiver)
  {
    stats_.bytes_received += receiver.getBytesReceived();
    peak_stats_.bytes_received = std::max(peak_stats_.bytes_received, stats_.bytes_received);
  }

private:
  Runtime runtime_{};
  Stats stats_{};
  Stats peak_stats_{};
  std::string name_;
};

} // namespace swc
} // namespace common
} // namespace lum
#endif
