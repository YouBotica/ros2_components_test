// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_CSV_FILE_LOGGER_H
#define LUM_COMMON_LOG_CSV_FILE_LOGGER_H

#include <fstream>
#include <mutex>

#include <lum_common_log/i_logger.h>

namespace lum {
namespace common {
namespace log {

/// @brief Log to CSV file
///
/// This logger writes to a text file with comma separated column entries:
///
/// Time, ProcessId, ThreadId, File, Line, LogLevel, Message
class CsvFileLogger : public ILogger
{
public:
  /// @brief Create a CSV file logger
  /// @param[in] log_file Full path of the file to write to
  /// @param[in] append If true, the file is opened for append, else it is re-written.
  explicit CsvFileLogger(const std::string& log_file, bool append = false);

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
