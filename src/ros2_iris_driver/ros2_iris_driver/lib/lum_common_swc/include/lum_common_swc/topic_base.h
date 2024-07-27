// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_TOPIC_BASE_H
#define LUM_COMMON_SWC_TOPIC_BASE_H

#include <lum_common_swc/i_topic.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Base class for topics.
///
/// @note Convenient to derive from instead of the interface.
template <typename MessageType>
class TopicBase : public ITopic
{
public:
  /// @brief The exposed type of the payload.
  using Type = MessageType;
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
