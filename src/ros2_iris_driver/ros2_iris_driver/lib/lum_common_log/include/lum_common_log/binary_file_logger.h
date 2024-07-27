// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_BINARY_FILE_LOGGER_H
#define LUM_COMMON_LOG_BINARY_FILE_LOGGER_H

#include <fstream>
#include <mutex>

#include <lum_common_log/i_logger.h>

namespace lum {
namespace common {
namespace log {

/// @brief Log to binary file.
///
/// The log records have the following format:
///
/// | Field          | Value                 |
/// | -------------- | --------------------- |
/// | Time           | int64                 |
/// | ProcessId      | uint32                |
/// | ThreadId       | uint32                |
/// | Line           | uint16                |
/// | LogLevel       | uint8                 |
/// | FileNameLength | uint8                 |
/// | MessageLength  | uint16                |
/// | FileName       | FileNameLength * char |
/// | Message        | MessageLength * char  |
class BinaryFileLogger : public ILogger
{
public:
  /// @brief Create a binary file logger
  /// @param[in] log_file Full path of the file to write to
  /// @param[in] append If true, the file is opened for append, else it is re-written.
  explicit BinaryFileLogger(const std::string& log_file, bool append = false);

  /// @copydoc ILogger::sendLogMessage
  void sendLogMessage(LogRecord log_record) override;

private:
  std::ofstream stream_;
  std::mutex stream_mutex_;
};

} // namespace log
} // namespace common
} // namespace lum

#endif
