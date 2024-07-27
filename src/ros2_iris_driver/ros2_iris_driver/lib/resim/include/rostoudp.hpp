#pragma once
#include "lum_optional.hpp"
#include "resim.hpp"
#include "schemas/udp.capnp.h"
#include <capnp/message.h>
#include <cstdio>
#include <iostream>

ReSim::Schemas::Udp::Reader ros_to_udp(std::string const &filename, capnp::MallocMessageBuilder &udp_message);
