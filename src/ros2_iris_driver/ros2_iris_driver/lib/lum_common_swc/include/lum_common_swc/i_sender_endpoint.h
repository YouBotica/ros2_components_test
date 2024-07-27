// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_I_SENDER_ENDPOINT_H
#define LUM_COMMON_SWC_I_SENDER_ENDPOINT_H

#include <memory>
#include <vector>

#include <lum_common_swc/i_message.h>
#include <lum_common_swc/i_topic.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Sending communication endpoint
class ISenderEndpoint
{
public:
  /// @brief A handle to endpoint which maintains the ownership
  using Handle = std::unique_ptr<ISenderEndpoint>;

  ISenderEndpoint() = default;
  virtual ~ISenderEndpoint() noexcept = default;

  ISenderEndpoint(const ISenderEndpoint&) = delete;
  ISenderEndpoint& operator=(const ISenderEndpoint&) & = delete;
  ISenderEndpoint(ISenderEndpoint&&) = delete;
  ISenderEndpoint& operator=(ISenderEndpoint&&) & = delete;

  /// @brief Enqueues messages to be sent on next step of the communication SWC
  /// @param[in] messages List of messages to be sent
  virtual void pushMessages(std::vector<IMessage::Handle> messages) = 0;

  /// @brief Get the topic identifier this endpoint is for
  /// @return The unique identifier
  virtual ITopic::Id getTopicId() const = 0;
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
