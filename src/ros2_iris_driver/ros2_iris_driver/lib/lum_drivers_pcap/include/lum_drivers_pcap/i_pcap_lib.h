// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_PCAP_I_PCAP_LIB_H
#define LUM_DRIVERS_PCAP_I_PCAP_LIB_H

#include <cstddef>
#include <cstdint>

#include <pcap.h>

namespace lum {
namespace drivers {
namespace pcap {

// Encapsulates PCAP library
class IPcapLibrary
{
public:
  IPcapLibrary() = default;
  virtual ~IPcapLibrary() = default;

  IPcapLibrary(const IPcapLibrary&) = delete;
  IPcapLibrary(IPcapLibrary&&) = delete;
  IPcapLibrary& operator=(const IPcapLibrary&) & = delete;
  IPcapLibrary& operator=(IPcapLibrary&&) & = delete;

  virtual int pcapCompile(pcap_t* pcap_file,
                          bpf_program* pcap_filter,
                          const char* proto,
                          int flag,
                          bpf_u_int32 mask) = 0;

  virtual int pcapNextEx(pcap_t* pcap_file, pcap_pkthdr** header, const u_char** packet_data) = 0;

  virtual FILE* pcapFile(pcap_t* file) = 0;

  virtual pcap_t* pcapFopenOffline(FILE* raw_file, char* error_buffer) = 0;

  virtual pcap_t* pcapOpenDead(int link_type, int snap_len) = 0;

  virtual char* pcapGeterr(pcap_t* pcap_file) = 0;

  virtual int pcapSetfilter(pcap_t* pcap_file, bpf_program* pcap_filter) = 0;

  virtual void pcapFreecode(bpf_program*) = 0;

  virtual pcap_dumper_t* pcapDumpOpen(pcap_t* pcap, const char* file_name) = 0;

  virtual void pcapDump(u_char* dumper, const pcap_pkthdr* header, const u_char* packet_data) = 0;

  virtual void pcapDumpClose(pcap_dumper_t* dumper) = 0;

  virtual void pcapClose(pcap_t* pcap_file) = 0;
};

} // namespace pcap
} // namespace drivers
} // namespace lum

#endif
