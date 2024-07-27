// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORK_UTILS_H
#define LUM_PLATFORM_NETWORK_UTILS_H

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <lum_platform_networking/types.h>
#include <lum_platform_networking_types/types.h>

namespace lum {
namespace platform {
namespace networking {

/// @brief Returns a string representation of the sockaddr's IP
/// @param [in] sa pointer to sockaddr struct
/// @return a string representation of the sockaddr's IP
std::string sockAddrToString(const sockaddr* sa);

/// @brief Returns a string representation of the sockaddr's IP
/// @param [in] sa pointer to sockaddr struct
/// @param [out] source_port reference to source_port
/// @return a string representation of the sockaddr's IP
std::string sockAddrToString(const sockaddr* sa, std::uint16_t& source_port);

/// @brief generate a string broadcast address given a string ip, and a string mask
/// @param [in] host_ip the ip address we want a broadcast address for
/// @param [in] netmask the network mask of the host_ip
/// @return the broadcast address, or the host_ip again on failure
std::string getBroadcast(const std::string& host_ip, const std::string& netmask);

/// @brief utility function to get the ip addresses, and their broadcast addresses, of the local
/// system
/// @note this function does not throw, if it cannot get the broadcast address, it simply returns
/// the host address again
/// @return A vector of string pairs, where the first is the interface name, and the second is the
/// broadcast address
std::vector<std::pair<std::string, std::string>> getSystemIPsAndBroadcasts();

/// @brief get the local ip address that *should* be reachable from the provided address
/// @param remote_address [in] the (remote) address that we want to match locally
/// @return the ip address of a local interface that enables socket connections with the given
/// address
std::string getMatchingInterface(const std::string& remote_address);

/// @brief Returns the last network error for printing
/// @return  A string representation of the last network error
std::string getLastNetworkError();

/// @brief Converts a byte-encoded IP address to a string representation
/// @param [in] addr the byte-encoded address
/// @return A string representation of the address
///
/// @attention The caller/calling function must assure that \c addr is passed in as a <em>network
///            byte order</em> address structure in the \c af <em>address family</em>
///
std::string ipAddressToString(const std::uint32_t addr);

/// @brief Converts a string representation to a byte-encoded IP address
/// @param [in] address the string address
/// @param [out] addr the byte-encoded address
/// @return A bool indicating success or failure
bool stringToIPAddress(const std::string& address, std::uint32_t& addr);

/// @brief Returns the port from the given \a socket_address.
/// @return The source port, or 0 if the address family is not recognized.
std::uint16_t getSourcePort(const sockaddr& socket_address);

/// @brief Converts a 2-byte unsigned integer from network byte order to host byte order
/// @param [in] net_bytes the network bytes to be converted
/// @return the 2-byte unsigned integer in host byte order
std::uint16_t toHostByteOrder(const std::uint16_t net_bytes);

/// @brief Converts a 4-byte unsigned integer from network byte order to host byte order
/// @param [in] net_bytes the network bytes to be converted
/// @return the 4-byte unsigned integer in host byte order
std::uint32_t toHostByteOrder(const std::uint32_t net_bytes);

/// @brief Converts a 2-byte unsigned integer from host byte order to network byte order
/// @param [in] host_bytes the host bytes to be converted
/// @return the 2-byte unsigned integer in network byte order
std::uint16_t toNetworkByteOrder(const std::uint16_t host_bytes);

/// @brief Converts a 4-byte unsigned integer from host byte order to network byte order
/// @param [in] host_bytes the host bytes to be converted
/// @return the 4-byte unsigned integer in network byte order
std::uint32_t toNetworkByteOrder(const std::uint32_t host_bytes);

/// @brief Converts a binary representation of float into int32 in network byte order.
/// @param [in] float_value the host bytes to be converted
/// @return the 4-byte unsigned integer in network byte order
std::uint32_t floatToNetworkByteOrder(float float_value);

/// @brief Converts a 4-byte binary representation of float from network byte order to host byte
/// order.
/// @param [in] uint_value_network the network bytes to be converted
/// @return the float number in host byte order
float floatToHostByteOrder(std::uint32_t uint_value_network);

/// @brief Attempts to close the socket. Callees should set `socket_` into an
///        appropriate default state afterwards.
/// @param [in] socket_ the socket to close
/// @return a SocketResponse indicating success or failure
SocketResponse closeSocket(PlatformSocket socket_);

namespace tcp {

/// @brief Opens a TCP socket
/// @param [out] socket_ will contain a representation of the open socket
/// @return a SocketResponse indicating success or failure
SocketResponse openSocket(PlatformSocket& socket_);

/// @brief Sends data through a socket
/// @param [in] socket_ the socket to write to
/// @param [in] payload the data to write
/// @param [in] the size of the payload
/// @return a SocketResponse indicating success or failure
SocketResponse
sendSocket(PlatformSocket socket_, const void* payload, const std::size_t payload_size);

/// @brief Attempts to read data from the socket
/// @param [in] socket_ the socket to attempt the read on
/// @param [in] buffer  the buffer to read data into
/// @param [in] buffer_size the size of the read buffer
/// @param [out] bytes_read the actual number of bytes read
/// @return a SocketResponse indicating success or failure
SocketResponse tryReadSocket(PlatformSocket socket_,
                             void* buffer,
                             const std::size_t buffer_size,
                             std::size_t& bytes_read);

/// @brief Attempts to `accept` on listener_socket
/// @param [out] accept_socket the socket representing the actual connection
/// @param [in] listener_socket the socket being listened to
/// @param [out] addr the address of the socket connected to by accept_socket
/// @param [out] addrlen the size of addr
/// @return a SocketResponse indicating success or failure
SocketResponse acceptSocket(PlatformSocket& accept_socket,
                            const PlatformSocket listener_socket,
                            sockaddr* addr,
                            std::uint32_t* addrlen);

/// @brief Attempts to connect socket_ to address:port
/// @param [in] socket_ the socket to connect
/// @param [in] address the address to connect with
/// @param [in] port the associated port
/// @param [in] timeout how long to attempt the connection
/// @return a SocketResponse indicating success or failure
SocketResponse connectSocket(PlatformSocket socket_,
                             const std::string& address,
                             const std::uint16_t port,
                             std::int32_t timeout);

/// @brief Attempts to set or unset the no delay option on the socket based on 'no_delay'
/// @param [in] socket_ the socket to apply the change to
/// @param [in] no_delay whether to set, or unset, the mode
/// @return a SocketResponse indicating success or failure
SocketResponse setNoDelayOption(PlatformSocket socket_, const bool no_delay);

/// @brief Attempts to set the poll timeout on the socket
/// @param [in] socket_ the socket to set the timeout on
/// @param [in] timeout the timeout to apply
/// return a SocketResponse indicating success or failure
SocketResponse setSocketPollTimeout(PlatformSocket socket_,
                                    const std::chrono::milliseconds& timeout);
} // namespace tcp

namespace udp {

/// @brief Opens a UDP socket
/// @param [out] socket_ will contain a representation of the open socket
/// @return a SocketResponse indicating success or failure
SocketResponse openSocket(PlatformSocket& socket_);

/// @brief Attempts to bind a socket to a port and address
/// @param [in] socket_ the socket to bind
/// @param [in] address the address to bind to
/// @param [in] port the port to bind to
/// @return a SocketResponse indicating success or failure
SocketResponse
bindSocket(PlatformSocket socket_, const std::string& address, const std::uint16_t port);

/// @brief Attempts to bind the socket with the given port on any local address
/// @param [in] socket_ the socket to bind
/// @param [in] port the port to bind
/// @return a SocketResponse indicating success or failure
SocketResponse bindSocket(PlatformSocket socket_, const std::uint16_t port);

/// @brief Attempts to bind a socket to a random port on an address
/// @param [in] socket_ the socket to bind
/// @param [in] address the address to bind to
/// @param [out] port the port to bound to
/// @return a SocketResponse indicating success or failure
SocketResponse
bindSocketRandomPort(PlatformSocket socket_, const std::string& address, std::uint16_t& port);

/// @brief Attempt to send data to the specified address
/// @param [in] socket_ the socket to send on
/// @param [in] dest_address the destination address
/// @param [in] dest_port the destination port
/// @param [in] payload the data payload to send
/// @param [in] payload_size the payload size
/// @return a SocketResponse indicating success or failure
SocketResponse sendToAddress(PlatformSocket socket_,
                             const std::string& dest_address,
                             const std::uint16_t dest_port,
                             const void* payload,
                             const std::size_t payload_size);

/// @brief Set or unset that we are reusing an address on the socket,
///        possibly before the OS has released it
/// @param [in] socket_ the socket to set the flag on
/// @param [in] reuse set / unset based on this flag
/// @return a SocketResponse indicating success or failure
SocketResponse setReuseAddress(PlatformSocket socket_, const bool reuse);

/// @brief Set the socket to broadcast
/// @param [in] socket_ the socket to broadcast on
/// @return a SocketResponse indicating success or failure
SocketResponse setSocketBroadcast(PlatformSocket socket_);

/// @brief Set the socket poll timeout
/// @param [in] socket_ the socket to set the timeout on
/// @param [in] timeout the poll timeout
/// @return a SocketResponse indicating success or failure
SocketResponse setSocketPollTimeout(PlatformSocket socket_,
                                    const std::chrono::milliseconds& timeout);

/// @brief Join multicast group
/// @param [in] socket_ the socket to receive multicast traffic on
/// @param [in] config the multicast configuration
/// @return a SocketResponse indicating success or failure
SocketResponse joinMulticastGroup(PlatformSocket socket_, const MulticastConfig& config);

/// @brief Attempt to read from the given socket
/// @param [in] socket_ the socket to read from
/// @param [in] buffer the read buffer
/// @param [in] buffer_size the size of the read buffer
/// @param [out] bytes_read the number of bytes read
/// @return a SocketResponse indicating success or failure
SocketResponse tryRead(PlatformSocket socket_,
                       void* buffer,
                       const std::size_t buffer_size,
                       std::size_t& bytes_read);

/// @brief Attempt to read from the socket,
///        recording the port we read from
/// @param [in] socket_ the socket to read from
/// @param [in] buffer the read buffer
/// @param [in] buffer_size the size of the read buffer
/// @param [out] bytes_read the number of bytes read
/// @param [out] from_port the remote port
SocketResponse tryReadFrom(PlatformSocket socket_,
                           void* buffer,
                           const std::size_t buffer_size,
                           std::size_t& bytes_read,
                           std::uint16_t& from_port);

/// @brief Attempt to read from the socket,
///        recording the address we read from
/// @param [in] socket_ the socket to read from
/// @param [in] buffer the read buffer
/// @param [in] buffer_size the size of the read buffer
/// @param [out] bytes_read the number of bytes read
/// @param [out] from_address the remote address
SocketResponse tryReadFrom(PlatformSocket socket_,
                           void* buffer,
                           const std::size_t buffer_size,
                           std::size_t& bytes_read,
                           std::string& from_address);

/// @brief Attempt to read from the socket,
///        recording the address and port we read from
/// @param [in] socket_ the socket to read from
/// @param [in] buffer the read buffer
/// @param [in] buffer_size the size of the read buffer
/// @param [out] bytes_read the number of bytes read
/// @param [out] from_address the remote address
/// @param [out] from_port the remote port
SocketResponse tryReadFrom(PlatformSocket socket_,
                           void* buffer,
                           const std::size_t buffer_size,
                           std::size_t& bytes_read,
                           std::string& from_address,
                           std::uint16_t& from_port);

/// @brief Attempt to read from the socket,
///        recording the address and port we read from
/// @param [in] socket_ the socket to read from
/// @param [in] buffer the read buffer
/// @param [in] buffer_size the size of the read buffer
/// @param [out] bytes_read the number of bytes read
/// @param [out] from_address the remote address
/// @param [out] from_port the remote port
SocketResponse tryReadFrom(PlatformSocket socket_,
                           void* buffer,
                           const std::size_t buffer_size,
                           std::size_t& bytes_read,
                           std::uint32_t& from_address,
                           std::uint16_t& from_port);
} // namespace udp

} // namespace networking
} // namespace platform
} // namespace lum

#endif
