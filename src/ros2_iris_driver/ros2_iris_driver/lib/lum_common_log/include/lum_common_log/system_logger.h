// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_SYSTEM_LOGGER_H
#define LUM_COMMON_LOG_SYSTEM_LOGGER_H

#include <lum_common_log/log_message.h>
#include <lum_common_log/stream_logger.h>
#include <lum_common_log/tee_logger.h>
#include <lum_common_types/singleton.h>

#ifdef _WIN32
#include <lum_common_log/default_windows_logger.h>
#else
#include <lum_common_log/default_linux_logger.h>
#endif

namespace lum {
namespace common {
namespace log {

/// @brief Global system logger ("syslog")
/// @note It starts off with a DefaultStreamLogger but one can exchange it for some custom logger.
class SystemLogger
{
public:
  /// @brief Get the current logger in use
  /// @return Reference to the current logger
  ILogger& get() const { return *logger_; }
  /// @brief Replace the current logger with another one
  /// @note Keep a reference to the returned logger to be able to switch back!
  /// @param[in] logger Custom logger to use
  /// @return Current logger with transferring ownership
  std::shared_ptr<ILogger> replace(std::shared_ptr<ILogger> logger)
  {
    auto prev = logger_;
    logger_ = std::move(logger);
    return prev;
  }

private:
  std::shared_ptr<ILogger> logger_{std::make_shared<DefaultStreamLogger>()};
};

/// @private
namespace erase {

struct None
{
};

template <typename List>
struct LogStream
{
  List list;
};

template <typename Rest, typename Item>
LogStream<std::pair<Rest, const Item&>> operator<<(LogStream<Rest> rest, const Item& item)
{
  return {{rest.list, item}};
}

} // namespace erase

/// @brief The global system logger (syslog)
using SysLog = types::Singleton<SystemLogger>;
/// @brief A logger which discards the stream operators passed in
using NullLog = erase::LogStream<erase::None>;

} // namespace log
} // namespace common
} // namespace lum

#endif
