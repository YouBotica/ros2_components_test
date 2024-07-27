// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_ASYNC_DATA_CLIENT_READER_ASYNC_DATA_CLIENT_READER_H
#define LUM_DRIVERS_LIDAR_ASYNC_DATA_CLIENT_READER_ASYNC_DATA_CLIENT_READER_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include <lum_common_memory/object_pool_heap.h>
#include <lum_common_types/networking.h>
#include <lum_common_types_observable/observable.h>
#include <lum_drivers_data_client/i_data_client.h>
#include <lum_drivers_data_client_types/types.h>

namespace lum {
namespace drivers {
namespace data_client {

/// @brief a data client reader class that operates asynchronously.  This class contains 2 threads:
/// 1 thread to read the data client and queue the read packets, and a second thread to deque
/// packets and call the callback for the packets.  This implements the producer-consumer pattern.
/// @throws std::invalid_argument if source is not a valid IDataClient
/// @note if you subscribe to the source before providing it to this class, your callback will be
/// called on the producer thread instead of the consumer thread.
class AsyncDataClientReader
{
  using ObjectPool =
    lum::common::memory::object_pool::SmartPoolHeap<common::types::networking::UdpPacket>;

public:
  /// @brief constructor
  /// @param [in] pool_size the number of packets to preallocate and limit to in memory
  /// @param [in] source an IDataClient implementation to be threaded and read from
  AsyncDataClientReader(std::size_t pool_size,
                        std::unique_ptr<drivers::data_client::IDataClient> source);

  ~AsyncDataClientReader();

  // Rule of 5 boilerplate
  AsyncDataClientReader(const AsyncDataClientReader&) = delete;
  AsyncDataClientReader(AsyncDataClientReader&&) = delete;
  void operator=(const AsyncDataClientReader&) & = delete;
  void operator=(AsyncDataClientReader&&) & = delete;

  /// @brief get if this reader is still processing.  This can be false when it is getting
  /// destroyed, or if the data client that this is reading from halts.
  /// @return true if this class is still processing
  bool isProcessing() const { return is_processing_; }

  /// @brief subscribe to packets from the DataClient, called on the consumer thread
  /// @param [in] callback a lambda function to receive packets when they are ready for processing
  /// @return a subscription handle.  Keep the handle in scope to continue to be called back, or
  /// destroy it to unsubscribe
  data_client::types::PacketSubscription
  subscribeOnPacket(const data_client::types::PacketSubscriptionCallback& callback);

private:
  ObjectPool pool_;
  std::size_t pool_size_{}; ///< memory pool size
  std::atomic_bool is_processing_{true};
  data_client::types::PacketSubscription packet_sub_{};

  std::unique_ptr<std::thread> data_client_thread_{};
  std::unique_ptr<std::thread> consumer_thread_{};

  /// observable for forwarding new packets
  common::types::observable::Observable<const data_client::types::DataClientData&> packets_;

  // sensor_data_source is declared after the threads so it is destroyed first
  std::unique_ptr<drivers::data_client::IDataClient> sensor_data_source_;

  std::deque<data_client::types::DataClientData> packet_queue_;
  std::condition_variable packet_cond_;
  std::mutex packet_cond_mutex_;

  void waitForOnePacket();
  void processDataClient();
  void processConsumerCallback();
};

} // namespace data_client
} // namespace drivers
} // namespace lum

#endif
