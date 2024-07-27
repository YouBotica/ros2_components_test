// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_STREAM_LOGGER_H
#define LUM_COMMON_LOG_STREAM_LOGGER_H

#include <fstream>
#include <mutex>
#include <sstream>

#include <lum_common_log/i_logger.h>

namespace lum {
namespace common {
namespace log {

/// @brief Generic stream logger
///
/// Output is formatted like: LEVEL: message
class OstreamLogger : public ILogger
{
public:
  /// @brief Create a logger writing to a stream
  /// @param[in] out The stream to write to
  explicit OstreamLogger(std::ostream& out) noexcept;

  /// @copydoc ILogger::initLogMessage
  void initLogMessage(const LogRecord& log_record) override;
  /// @copydoc ILogger::sendLogMessage
  void sendLogMessage(LogRecord log_record) override;

protected:
  std::ostream& log_stream_;
  std::mutex log_stream_mutex_{};
};

/// @brief Simple stream logger writing to stderr
/// @note Log messages are separated by newline
class StdErrLogger : public OstreamLogger
{
public:
  StdErrLogger() noexcept;
};

/// @brief Simple logger writing to a string
/// @note Log messages are separated by newline
class StringLogger : public OstreamLogger
{
public:
  StringLogger() noexcept;

  /// @brief Get the formatted output as string
  /// @return The string
  std::string getString() const;
  /// @brief Clear the contained string
  void clear();

private:
  std::ostringstream stream_{};
};

/// @brief Simple logger to write to a text file
/// @note Log messages are separated by newline
class FileLogger : public OstreamLogger
{
public:
  /// @brief Create a text file logger
  /// @param[in] log_file Full path of the file to write to
  /// @param[in] append If true, the file is opened for append, else it is re-written.
  explicit FileLogger(const std::string& log_file, bool append = false);

private:
  std::ofstream file_stream_;
};

} // namespace log
} // namespace common
} // namespace lum

#endif
