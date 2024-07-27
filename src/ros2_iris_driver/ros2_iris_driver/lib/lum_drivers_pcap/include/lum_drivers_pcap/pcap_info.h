// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_PCAP_PCAP_INFO_H
#define LUM_DRIVERS_PCAP_PCAP_INFO_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <lum_drivers_pcap/i_pcap_lib.h>

namespace lum {
namespace drivers {
namespace pcap {
namespace types {

/// @brief PCAP info struct
struct PcapInfo
{
  std::size_t num_packets{0}; ///< total number of packets in file
  std::size_t data_size{0};   ///< total length of all packets in the file (in bytes)
  double capture_duration{
    std::numeric_limits<double>::quiet_NaN()}; ///< capture duration (in seconds.microseconds)
  std::chrono::microseconds earliest_timestamp{
    std::chrono::microseconds::max()}; ///< earliest timestamp in the file
                                       ///< (not necessarily the first)
  std::chrono::microseconds latest_timestamp{
    std::chrono::microseconds::min()}; ///< latest timestamp in the file
                                       ///< (not necessarily the last)
};

} // namespace types

types::PcapInfo getPcapInfo(const std::string& file_path, std::shared_ptr<IPcapLibrary> pcap = {});

} // namespace pcap
} // namespace drivers
} // namespace lum

#endif
