// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_TEE_LOGGER_H
#define LUM_COMMON_LOG_TEE_LOGGER_H

#include <lum_common_log/i_logger.h>

namespace lum {
namespace common {
namespace log {

/// @brief Tee logger
///
///	Given two loggers, this class owns them and forwards the log messages to both while handling
/// message ownership correctly.
class TeeLogger : public ILogger
{
public:
  /// @brief Create forwarding logger by taking ownership of the two loggers.
  /// @param[in] one First logger
  /// @param[in] two Second logger
  TeeLogger(std::unique_ptr<ILogger> one, std::unique_ptr<ILogger> two) noexcept;

  /// @copydoc ILogger::sendLogMessage
  void sendLogMessage(LogRecord log_record) override;
  /// @copydoc ILogger::colorizeLogMessage
  void colorizeLogMessage(bool on) noexcept override;
  /// @copydoc ILogger::setDiscardLevel
  void setDiscardLevel(LogLevel level) noexcept override;

  /// @return Pointer to first managed logger
  ILogger* getLogger1() const noexcept;
  /// @return Pointer to second managed logger
  ILogger* getLogger2() const noexcept;

private:
  std::unique_ptr<ILogger> logger1_;
  std::unique_ptr<ILogger> logger2_;
};

} // namespace log
} // namespace common
} // namespace lum

#endif
