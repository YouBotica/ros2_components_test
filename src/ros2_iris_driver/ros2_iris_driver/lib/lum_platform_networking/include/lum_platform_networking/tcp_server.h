// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORKING_TCP_SERVER_H
#define LUM_PLATFORM_NETWORKING_TCP_SERVER_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include <lum_platform_networking/types.h>
#include <lum_platform_networking_types/types.h>

namespace lum {
namespace platform {
namespace networking {

///@brief a simple TCP server.  It will accepts a single connection, but does not support more than
/// one client connection
/// @code
/// lum::platform::networking::TCPServer server;
/// server.open();
/// server.bind( "127.0.0.1", 50000 );
/// server.listen();
/// server.start();
///
/// // connect to server from a client socket
///
/// // perform TCP communication between client/server
///
/// server.shutdown();
/// @endcode
/// @code
/// lum::platform::networking::TCPServer server( "127.0.0.1", 50000 );
/// server.start();
///
/// // connect to server from a client socket
///
/// // perform TCP communication between client/server
///
/// server.shutdown();
/// @endcode
class TCPServer
{
public:
  ///@brief Construct a new TCPServer object; performs no socket operations
  TCPServer();

  /// @brief Construct a new TCPServer object; sets socket up to listen on provided local
  /// address, and binds to a random port
  /// @param server_ip [in] server IP address
  explicit TCPServer(const std::string& server_ip);

  ///@brief Construct a new TCPServer object; sets socket up to listen on provided local
  /// endpoint
  ///@param server_ip [in] server IP address
  ///@param server_port [in] server port number
  TCPServer(const std::string& server_ip, const std::uint16_t server_port);

  ///@brief Destroy the TCPServer object
  virtual ~TCPServer();

  TCPServer(const TCPServer&) = delete;
  TCPServer& operator=(const TCPServer&) & = delete;
  TCPServer(TCPServer&& other) = delete;
  TCPServer& operator=(TCPServer&&) & = delete;

  ///@brief opens TCP socket
  TCPServerSocketResponse open();

  /// @brief sets the timeout used during a tryRead() operation
  /// @param [in] timeout duration to block on socket before returning with no message; a value of 0
  /// shall be used if the desired behavior is to not timeout.
  TCPServerSocketResponse setSocketPollTimeout(const std::chrono::milliseconds& timeout);

  ///@brief binds TCP socket to provided local endpoint
  ///@param server_ip [in] server IP address
  ///@param server_port [in] server port number
  TCPServerSocketResponse bind(const std::string& server_ip, const std::uint16_t server_port);

  ///@brief binds TCP socket to random local endpoint
  ///@param server_ip [in] server IP address
  TCPServerSocketResponse bind(const std::string& server_ip);

  /// @brief get the bound port
  /// @return port value for local endpoint
  std::uint16_t getPort() const { return port_.load(); }

  ///@brief sets up socket to listen for one incoming TCP connection.
  TCPServerSocketResponse listen();

  ///@brief kicks off a separate thread that executes run()
  TCPServerSocketResponse start();

  /// @brief try to read from the socket, blocks forever, or until timeout expires
  /// @param buffer [in] raw pointer to the bytes to send
  /// @param buffer_size [in] number of bytes to read and send from the buffer
  /// @return the number of bytes read, will be zero for no data
  std::size_t tryRead(void* buffer, const std::size_t buffer_size);

  /// @brief send a packet
  /// @param payload [in] pointer to the buffer containing the message bytes
  /// @param payload_size [in] number of bytes from the payload buffer to read and send
  TCPServerSocketResponse send(const void* payload, const std::size_t payload_size);

  ///@brief stops run thread, but leaves TCP listening socket open.
  void stop();

  ///@brief shuts down the TCP server; this includes stopping the run thread and closing the
  /// listening socket.
  TCPServerSocketResponse shutdown();

  ///@brief whether the TCP server socket is open
  ///@return whether TCP server socket is open or not
  bool isOpen() const;

  ///@brief whether the TCP server socket is bound to a local endpoint
  ///@return whether the TCP server is bound to a local endpoint or not
  bool isBound() const;

  ///@brief whether the TCP server is setup to listen for an incoming connection
  ///@return whether the TCP server is listening
  bool isListening() const;

  ///@brief whether the run thread is active
  ///@return whether the run thread is active
  bool isRunning() const;

  ///@brief whether the TCP server is connected to a client socket.
  ///@return whether the TCP server is connected to a client socket.
  bool isConnected() const;

protected:
  std::atomic<PlatformSocket> connection_{0};

private:
  ///@brief accepts an incoming connection attempt from a client and runs until it is stopped
  void run();

  ///@brief accepts an incoming connection attempt from a client
  TCPServerSocketResponse accept();

  ///@brief this function represents what the server should do while running.
  virtual void process();

  /// atomic wrapper around platform socket
  std::atomic<PlatformSocket> socket_{DEFAULT_PLATFORM_SOCKET};

  std::atomic<bool> is_open_{false};
  std::atomic<bool> is_bound_{false};
  std::atomic<bool> is_listening_{false};
  std::atomic<bool> is_connected_{false};
  std::atomic<bool> is_running_{false};
  std::atomic<std::uint16_t> port_{0U};

  std::thread run_thread_;
};

} // namespace networking
} // namespace platform
} // namespace lum

#endif
