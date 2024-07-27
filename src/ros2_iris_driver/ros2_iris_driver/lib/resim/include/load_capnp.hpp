#pragma once
#include "capnp/any.h"
#include "capnp/compat/json.h"
#include "kj/exception.h"
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <cstdio>
#include <exception>
#include <fstream>
#include <sstream>
#include <stdexcept>

void capnp_loader_error(const kj::Exception &e)
{
	std::stringstream ss;
	ss << "Could not open and parse the Capnp file:\n" << e.getDescription().cStr() << '\n';
	std::string error_message{ss.str()};
	throw std::runtime_error{error_message};
}

#ifndef CAPNP_TRAVERSAL_LIMIT
constexpr auto CAPNP_TRAVERSAL_LIMIT = (8 * 1024 * 1024 * 100);
#endif
constexpr auto CAPNP_READER_OPTIONS = capnp::ReaderOptions{CAPNP_TRAVERSAL_LIMIT};

template <typename Schema> class CapnpLoader
{
public:
	CapnpLoader(FILE *file) : message{fileno(file), CAPNP_READER_OPTIONS} {}
	                             operator typename Schema::Reader() { return message.getRoot<Schema>(); }
	capnp::PackedFdMessageReader message;

private:
};

template <typename Schema>
typename Schema::Reader load_capnp_from_json(const std::string &filename, capnp::MallocMessageBuilder &message)
{
	std::ifstream file(filename, std::ios::in);

	if (!file.is_open())
	{
		throw std::runtime_error{"Could not open the json file."};
	}

	std::ostringstream ss;
	ss << file.rdbuf();
	std::string   s = ss.str();
	kj::StringPtr kj_string(s);

	auto             builder = message.initRoot<Schema>();
	capnp::JsonCodec json;
	json.decode(kj_string, builder);
	return builder;
}