// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_LOG_H
#define LUM_COMMON_LOG_LOG_H

//
// GENERATED FILE - DO NOT EDIT
//

#include <lum_common_log/system_logger.h>

#define COMPILED_WITH_LUM_LOG_TRACE
#define LUM_LOG_TRACE lum::common::log::LogMessage(__FILE__, static_cast<std::uint32_t>(__LINE__), lum::common::log::LogLevel::LEVEL_TRACE, lum::common::log::SysLog::getInstance().get()).getStream()

#define COMPILED_WITH_LUM_LOG_DEBUG
#define LUM_LOG_DEBUG lum::common::log::LogMessage(__FILE__, static_cast<std::uint32_t>(__LINE__), lum::common::log::LogLevel::LEVEL_DEBUG, lum::common::log::SysLog::getInstance().get()).getStream()

#define COMPILED_WITH_LUM_LOG_INFO
#define LUM_LOG_INFO lum::common::log::LogMessage(__FILE__, static_cast<std::uint32_t>(__LINE__), lum::common::log::LogLevel::LEVEL_INFO, lum::common::log::SysLog::getInstance().get()).getStream()

#define COMPILED_WITH_LUM_LOG_WARN
#define LUM_LOG_WARN lum::common::log::LogMessage(__FILE__, static_cast<std::uint32_t>(__LINE__), lum::common::log::LogLevel::LEVEL_WARN, lum::common::log::SysLog::getInstance().get()).getStream()

#define COMPILED_WITH_LUM_LOG_ERROR
#define LUM_LOG_ERROR lum::common::log::LogMessage(__FILE__, static_cast<std::uint32_t>(__LINE__), lum::common::log::LogLevel::LEVEL_ERROR, lum::common::log::SysLog::getInstance().get()).getStream()

#define COMPILED_WITH_LUM_LOG_FATAL
#define LUM_LOG_FATAL lum::common::log::LogMessage(__FILE__, static_cast<std::uint32_t>(__LINE__), lum::common::log::LogLevel::LEVEL_FATAL, lum::common::log::SysLog::getInstance().get()).getStream()

#endif
