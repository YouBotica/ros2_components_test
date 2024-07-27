// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_SENDER_H
#define LUM_COMMON_SWC_SENDER_H

#include <vector>

#include <lum_common_swc/i_sender_endpoint.h>
#include <lum_common_swc/message.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Class for simple handling of sender communication endpoints.
///
/// This class wraps the opaque communication subsystem specific endpoint handle. The sender submits
/// messages to be transferred in the next @c step() call to the communication SWC. They do not get
/// sent immediately, nor is their delivery guaranteed.
/// @sa TsmCommunication RosCommunication ShmCommunication
template <typename Topic>
class Sender
{
public:
  /// @brief Create a typed sender.
  /// @param[in] sender The opaque endpoint of the communication subsystem to use.
  explicit Sender(ISenderEndpoint::Handle sender) : sender_{std::move(sender)} {}

  /// @brief Exposed type of transported message.
  using Type = typename Topic::Type;

  /// @brief Submit several messages for sending by the communication subsystem.
  /// @param[in] messages List of message messages to be sent.
  void submit(std::vector<Type> messages)
  {
    auto wrapped_messages = std::vector<IMessage::Handle>{};
    wrapped_messages.reserve(messages.size());
    for (auto&& message : messages)
    {
      wrapped_messages.push_back(std::make_unique<Message<Type>>(std::move(message)));
    }
    sender_->pushMessages(std::move(wrapped_messages));
    num_bytes_sent_ += sizeof(Type) * messages.size();
  }

  /// @brief Submit a single message for sending by the communication subsystem.
  /// @param[in] message Message to be sent.
  void submit(Type message)
  {
    auto wrapped_message = std::vector<IMessage::Handle>{};
    wrapped_message.push_back(std::make_unique<Message<Type>>(std::move(message)));
    sender_->pushMessages(std::move(wrapped_message));
    num_bytes_sent_ += sizeof(Type);
  }

  /// @brief Size of messages submitted to the communication subsystem.
  /// @return Number of bytes sent since last reset.
  std::size_t getBytesSent() const noexcept { return num_bytes_sent_; }
  /// @brief Zero the bytes sent counter.
  void resetBytesSent() noexcept { num_bytes_sent_ = 0U; }

private:
  ISenderEndpoint::Handle sender_;
  std::size_t num_bytes_sent_{0U};
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
