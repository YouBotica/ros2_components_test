// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_MESSAGE_H
#define LUM_COMMON_SWC_MESSAGE_H

#include <lum_common_swc/i_message.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Base class for communication messages.
///
/// @note Convenient to derive from instead of the interface.
template <typename MessageType>
class Message : public IMessage
{
public:
  /// @brief Create a communication message
  /// @param[in] payload Message content gets taken over
  explicit Message(MessageType payload) : payload_{std::move(payload)} {}

  /// @copydoc IMessage::clone
  IMessage::Handle clone() const override
  {
    return std::make_unique<Message<MessageType>>(payload_);
  }

  /// @brief Get the payload data stored in self
  /// @return Read-only reference to the stored data
  const MessageType& getPayload() const { return payload_; }

private:
  MessageType payload_;
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
