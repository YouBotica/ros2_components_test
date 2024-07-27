// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_I_COMMUNICATION_H
#define LUM_COMMON_SWC_I_COMMUNICATION_H

#include <lum_common_swc/i_topic.h>
#include <lum_common_swc/receiver.h>
#include <lum_common_swc/sender.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Interface of SWC communication subsystem.
///
/// It allows moving data between software components (SWCs).
/// The communication subsystem is typically also an SWC itself (i.e., it implements both
/// ISoftwareComponent and ICommunication interfaces). This communication subsystem SWC is owned by
/// the module, and thus the module is able to select the appropriate transport for the platform.
/// Due to interface segregation, however, the SWCs _using_ the communication interface do not need
/// to know that (or if) the communication subsystem is also an SWC.
///
/// @note This interface acts as a factory and the implementation does not own the created endpoint
/// instances.
///
/// @note The endpoint handles are opaque, use the typed Sender and Receiver templates to work with
/// them.
///
/// @note Use the convenience helper functions makeSender() and makeReceiver() to create a typed
/// Sender or Receiver from a topic type, no need to create the unique topic id manually.
///
/// @sa Sender Receiver TsmCommunication
class ICommunication
{
public:
  ICommunication() = default;
  virtual ~ICommunication() = default;

  ICommunication(const ICommunication&) = delete;
  ICommunication& operator=(const ICommunication&) & = delete;
  ICommunication(ICommunication&&) = delete;
  ICommunication& operator=(ICommunication&&) & = delete;

  /// @brief Create a sending communication endpoint for a specific topic.
  /// @param[in] id Unique topic identifier
  /// @param[in] payload_size Size of the message payload in bytes
  /// @return Handle to the created Sender
  virtual ISenderEndpoint::Handle createSenderEndpoint(ITopic::Id id, std::size_t payload_size) = 0;

  /// @brief Create a receiving communication endpoint for a specific topic with specific buffer
  /// size.
  /// @param[in] id Unique topic identifier
  /// @param[in] payload_size Size of the message payload in bytes
  /// @param[in] queue_size Size of the receive buffer (FIFO queue) in number of messages
  /// @return Handle to the created Receiver
  virtual IReceiverEndpoint::Handle
  createReceiverEndpoint(ITopic::Id id, std::size_t payload_size, std::size_t queue_size) = 0;

  /// @brief Get sum of message sizes transported for all senders.
  /// @return Number of bytes in total since last reset.
  virtual std::size_t getTotalBytesSent() const = 0;
  /// @brief Get sum of message sizes transported for all receivers.
  /// @return Number of bytes in total since last reset.
  virtual std::size_t getTotalBytesReceived() const = 0;
  /// @brief Zero the total send and total receive counters.
  virtual void resetTotalBytesCounters() = 0;
};

/// @brief Convenience helper to create a Sender from a topic type.
/// @tparam Topic The topic type
/// @param[in] factory The communication subsystem to create the Sender for
/// @return Sender instance with ownership transfer
template <typename Topic>
inline Sender<Topic> makeSender(ICommunication& factory)
{
  return Sender<Topic>(
    factory.createSenderEndpoint(getTopicIdentifier<Topic>(), sizeof(typename Topic::Type)));
}

/// @brief Convenience helper to create a Receiver from a topic type.
/// @tparam Topic The topic type
/// @param[in] factory The communication subsystem to create the Receiver for
/// @param[in] queue_size The receive buffer (FIFO queue) size in number of messages
/// @return Receiver instance with ownership transfer
template <typename Topic>
inline Receiver<Topic> makeReceiver(ICommunication& factory, std::size_t queue_size = 1U)
{
  return Receiver<Topic>(factory.createReceiverEndpoint(
    getTopicIdentifier<Topic>(), sizeof(typename Topic::Type), queue_size));
}

} // namespace swc
} // namespace common
} // namespace lum

#endif
