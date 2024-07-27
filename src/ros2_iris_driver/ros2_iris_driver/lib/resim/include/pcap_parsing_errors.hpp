#ifndef PCAP_PARSING_ERRORS__HPP
#define PCAP_PARSING_ERRORS__HPP

#include <exception>
#include <string>

struct MalformedMipiFrame : public std::exception
{
	char const *what() const noexcept override { return "MipiFrame is malformed!"; }
};

#endif // PCAP_PARSING_ERRORS__HPP
