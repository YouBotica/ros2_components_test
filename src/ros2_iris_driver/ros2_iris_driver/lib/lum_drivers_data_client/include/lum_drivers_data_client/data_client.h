// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_DATA_CLIENT_DATA_CLIENT_H
#define LUM_DRIVERS_DATA_CLIENT_DATA_CLIENT_H

#include <cmath>
#include <cstdint>
#include <memory>

#include <lum_drivers_data_client/i_data_client.h>
#include <lum_drivers_data_client/i_data_client_demux.h>
#include <lum_drivers_pcap/i_pcap_reader.h>
#include <lum_platform_networking_types/i_udp_socket.h>
#include <lum_platform_networking_types/types.h>

namespace lum {
namespace drivers {
namespace data_client {

/// @brief Construct an data_client::DataClient backed by a network socket
/// @param local_port the local port the incoming data is targeted at
/// @param reuse_address allow socket to be reused (true)
/// @param socket a UDP socket
/// @return a unique pointer to the data client
std::unique_ptr<IDataClient>
makeNetworkDataClient(std::uint16_t local_port,
                      bool reuse_address = true,
                      std::unique_ptr<platform::networking::IUDPSocket> socket = nullptr);

/// @brief Construct an data_client::DataClient backed by a network socket with multicast
/// @param local_port the local port the incoming data is targeted at
/// @param multicast_config multicast configuration
/// @param reuse_address allow socket to be reused (true)
/// @param socket a UDP socket
/// @return a unique pointer to the data client
std::unique_ptr<IDataClient>
makeNetworkDataClient(std::uint16_t local_port,
                      const platform::networking::MulticastConfig& multicast_config,
                      bool reuse_address = true,
                      std::unique_ptr<platform::networking::IUDPSocket> socket = nullptr);

/// @brief Construct an data_client::DataClient backed by a pcap. Returns nullptr if the file does
/// not exist, or if the PCAPDataClient could not be constructed
/// @param pcap_reader An interface to read the backing pcap
/// @param sleep_at_packet_timestamps attempt to preserve inherent pcap timing
/// @param loop continuously play the pcap
/// @param playback_speed playback rate
/// @param start_time_offset offset into the pcap
/// @param playback_duration how long to play
/// @return a unique pointer to the data client, or nullptr if the file does not exist
std::unique_ptr<IDataClient>
makePcapDataClient(std::unique_ptr<drivers::pcap::IPcapReader> pcap_reader,
                   const bool sleep_at_packet_timestamps = true,
                   const bool loop = false,
                   const std::float_t playback_speed = 1.0F,
                   const std::double_t start_time_offset = 0.0,
                   const std::double_t playback_duration = -1.0);

/// @brief Construct a demux data client for dividing mixed data streams
/// @return a unique pointer to an instance
std::unique_ptr<IDataClientDemux> makeDataClientDemux();

} // namespace data_client
} // namespace drivers
} // namespace lum

#endif
