// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_I_RECEIVER_ENDPOINT_H
#define LUM_COMMON_SWC_I_RECEIVER_ENDPOINT_H

#include <memory>
#include <vector>

#include <lum_common_swc/i_message.h>
#include <lum_common_swc/i_topic.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Receiving communication endpoint
class IReceiverEndpoint
{
public:
  /// @brief A handle to endpoint which maintains the ownership
  using Handle = std::unique_ptr<IReceiverEndpoint>;

  IReceiverEndpoint() = default;
  virtual ~IReceiverEndpoint() noexcept = default;

  IReceiverEndpoint(const IReceiverEndpoint&) = delete;
  IReceiverEndpoint& operator=(const IReceiverEndpoint&) & = delete;
  IReceiverEndpoint(IReceiverEndpoint&&) = delete;
  IReceiverEndpoint& operator=(IReceiverEndpoint&&) & = delete;

  /// @brief Get all messages received since last call to this function
  /// @return List of received messages
  virtual std::vector<IMessage::Handle> popMessages() = 0;

  /// @brief Get the topic identifier this endpoint is for
  /// @return The unique identifier
  virtual ITopic::Id getTopicId() const = 0;
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
