// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_PCAP_PCAP_READER_H
#define LUM_DRIVERS_PCAP_PCAP_READER_H

#include <memory>
#include <string>

#include <lum_drivers_pcap/i_pcap_reader.h>

namespace lum {
namespace drivers {
namespace pcap {

/// @brief Creates a concrete IPcapReader object
/// @param filename The PCAP filename to open
/// @return The reader if all was successful, or null if creation failed (check the log for
/// errors)
std::unique_ptr<IPcapReader> makePcapReader(const std::string& filename) noexcept;

} // namespace pcap
} // namespace drivers
} // namespace lum

#endif
