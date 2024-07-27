// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_TSM_COMMUNICATION_H
#define LUM_COMMON_SWC_TSM_COMMUNICATION_H

#include <map>
#include <mutex>
#include <queue>

#include <lum_common_swc/i_communication.h>
#include <lum_common_swc/runtime.h>
#include <lum_common_swc/software_component_base.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Communication implementation using thread shared memory transport.
///
/// This SWC implements a communication subsystem which moves the sent messages to the receivers by
/// pointing to shared memory. This means all Sender and Receiver instances must be in the same
/// process context. It does not support inter-process communication.
/// @sa RosCommunication ShmCommunication
class TsmCommunication : public SoftwareComponentBase<runtime::ModuleStepping>,
                         public ICommunication
{
public:
  TsmCommunication();

  /// @copydoc ISoftwareComponent::step
  bool step() override;

  /// @copydoc ICommunication::createSenderEndpoint
  ISenderEndpoint::Handle createSenderEndpoint(ITopic::Id id, std::size_t payload_size) override;
  /// @copydoc ICommunication::createReceiverEndpoint
  IReceiverEndpoint::Handle
  createReceiverEndpoint(ITopic::Id id, std::size_t payload_size, std::size_t queue_size) override;

  /// @copydoc ICommunication::getTotalBytesSent
  std::size_t getTotalBytesSent() const noexcept override { return bytes_sent_; }
  /// @copydoc ICommunication::getTotalBytesReceived
  std::size_t getTotalBytesReceived() const noexcept override { return bytes_received_; }
  /// @copydoc ICommunication::resetTotalBytesCounters
  void resetTotalBytesCounters() noexcept override
  {
    bytes_sent_ = 0U;
    bytes_received_ = 0U;
  }

private:
  using BaseClass = SoftwareComponentBase<runtime::ModuleStepping>;
  class TsmSenderEndpoint;
  class TsmReceiverEndpoint;

  void querySenders();
  void distributeMessages();
  void unregister(const TsmSenderEndpoint& sender);
  void unregister(const TsmReceiverEndpoint& receiver);

  std::map<ITopic::Id, std::queue<IMessage::Handle>> message_queues_;
  std::map<ITopic::Id, std::size_t> message_sizes_;
  std::multimap<ITopic::Id, TsmSenderEndpoint*> senders_;
  std::multimap<ITopic::Id, TsmReceiverEndpoint*> receivers_;
  std::mutex map_access_mutex_{};
  std::size_t bytes_sent_{0U};
  std::size_t bytes_received_{0U};
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
