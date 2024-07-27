// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_DEFAULT_LINUX_LOGGER_H
#define LUM_COMMON_LOG_DEFAULT_LINUX_LOGGER_H

#include <lum_common_log/stream_logger.h>

namespace lum {
namespace common {
namespace log {

/// @brief The simple stream logger writing to stderr
using DefaultStreamLogger = StdErrLogger;

} // namespace log
} // namespace common
} // namespace lum
#endif
