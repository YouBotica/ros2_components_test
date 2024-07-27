// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_PCAP_PCAP_WRITER_H
#define LUM_DRIVERS_PCAP_PCAP_WRITER_H

#include <memory>
#include <string>

#include <lum_drivers_pcap/i_pcap_writer.h>

namespace lum {
namespace drivers {
namespace pcap {

/// @brief Creates a concrete IPcapWriter object with a single filename
/// @param filename The PCAP filename to write
/// @return The writer if all was successful, or null if creation failed (check the log for
/// errors)
std::unique_ptr<IPcapWriter> makePcapWriter(std::string filename) noexcept;

/// @brief Creates a concrete IPcapWriter object to generate files in the specified directory
/// @param directory The directory to write pcap files to
/// @param output_prefix The prefix for all pcaps that get generated
/// @param minimum_duration_seconds The minimum number of seconds to include in a frame
/// @return The writer if all was successful, or null if creation failed (check the log for
/// errors)
std::unique_ptr<IPcapWriter> makePcapWriter(std::string directory,
                                            std::string output_prefix,
                                            float minimum_duration_seconds) noexcept;

} // namespace pcap
} // namespace drivers
} // namespace lum

#endif
