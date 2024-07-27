// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_RECEIVER_H
#define LUM_COMMON_SWC_RECEIVER_H

#include <vector>

#include <lum_common_swc/i_receiver_endpoint.h>
#include <lum_common_swc/message.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Class for simple handling of receiver communication endpoints.
///
/// This class wraps the opaque communication subsystem specific endpoint handle. The receiver will
/// return the messages obtained in the @c step() call of the communication SWC predating the
/// retrieve() call. Messages which have physically arrived since the last communication step will
/// only be available here after the next communication step.
/// @sa TsmCommunication RosCommunication ShmCommunication
template <typename Topic>
class Receiver
{
public:
  /// @brief Create a typed receiver.
  /// @param[in] receiver The opaque endpoint of the communication subsystem to use.
  explicit Receiver(IReceiverEndpoint::Handle receiver) : receiver_{std::move(receiver)} {}

  /// @brief Exposed type of transported message.
  using Type = typename Topic::Type;

  /// @brief Get all messages received since last call to this function.
  /// @return List of received messages.
  std::vector<Type> retrieve()
  {
    const auto messages = receiver_->popMessages();
    auto payloads = std::vector<Type>{};
    payloads.reserve(messages.size());
    for (const auto& message : messages)
    {
      payloads.push_back(static_cast<Message<Type>&>(*message).getPayload());
    }
    num_bytes_received_ += sizeof(Type) * messages.size();
    return payloads;
  }

  /// @brief Size of messages retrieved from the communication subsystem.
  /// @return Number of bytes received since last reset.
  std::size_t getBytesReceived() const noexcept { return num_bytes_received_; }
  /// @brief Zero the bytes received counter.
  void resetBytesReceived() noexcept { num_bytes_received_ = 0U; }

private:
  IReceiverEndpoint::Handle receiver_;
  std::size_t num_bytes_received_{0U};
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
