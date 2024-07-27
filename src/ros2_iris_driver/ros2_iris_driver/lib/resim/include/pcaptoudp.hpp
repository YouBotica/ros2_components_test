#pragma once
#include "resim.hpp"
#include "schemas/udp.capnp.h"
#include <capnp/message.h>
#include <cstdio>
#include <iostream>

// Example usage:
// capnp::MallocMessageBuilder udp_message;
// auto udp_builder = pcap_to_udp("in.pcap", udp_message);
//
// capnp::MallocMessageBuilder  compressed_message;
// Schemas::Compressed::Builder compressed_builder = compressed_message.initRoot<Schemas::Compressed>();
// LibReSim<Schemas::Udp, Schemas::Compressed> udp_to_compressed{};
// udp_to_compressed.Process(udp_builder, compressed_builder);
ReSim::Schemas::Udp::Reader pcap_to_udp(FILE *file, capnp::MallocMessageBuilder &udp_message);
