// Offline loopback regression for the UR Client Library's framed TCP reader.
// This program binds only to 127.0.0.1 in its own process/container. It does
// not connect to, publish to, or otherwise control a robot.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ur_client_library/comm/bin_parser.h>
#include <ur_client_library/comm/stream.h>
#include <ur_client_library/primary/primary_package.h>

using namespace std::chrono_literals;

namespace
{
using Stream = urcl::comm::URStream<urcl::primary_interface::PrimaryPackage>;

void require(bool condition, const std::string& message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

bool send_all(int fd, const uint8_t* bytes, size_t count)
{
  while (count > 0)
  {
    const ssize_t sent = ::send(fd, bytes, count, MSG_NOSIGNAL);
    if (sent <= 0)
    {
      return false;
    }
    bytes += sent;
    count -= static_cast<size_t>(sent);
  }
  return true;
}

std::vector<uint8_t> frame(size_t length, uint8_t payload = 0x42)
{
  require(length >= 4 && length <= UINT32_MAX, "invalid test frame size");
  std::vector<uint8_t> bytes(length, payload);
  const uint32_t network_length = htonl(static_cast<uint32_t>(length));
  std::memcpy(bytes.data(), &network_length, sizeof(network_length));
  return bytes;
}

class LoopbackServer
{
public:
  explicit LoopbackServer(std::function<void(int)> serve)
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    require(listen_fd_ >= 0, "socket failed");

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    require(::bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0, "bind failed");
    require(::listen(listen_fd_, 1) == 0, "listen failed");
    socklen_t address_size = sizeof(address);
    require(::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&address), &address_size) == 0,
            "getsockname failed");
    port_ = ntohs(address.sin_port);

    worker_ = std::thread([this, serve = std::move(serve)]() {
      const int client_fd = ::accept(listen_fd_, nullptr, nullptr);
      if (client_fd < 0)
      {
        worker_error_ = std::make_exception_ptr(std::runtime_error("accept failed"));
        return;
      }
      try
      {
        serve(client_fd);
      }
      catch (...)
      {
        worker_error_ = std::current_exception();
      }
      ::shutdown(client_fd, SHUT_RDWR);
      ::close(client_fd);
    });
  }

  ~LoopbackServer()
  {
    if (worker_.joinable())
    {
      worker_.join();
    }
    if (listen_fd_ >= 0)
    {
      ::close(listen_fd_);
    }
  }

  int port() const
  {
    return port_;
  }

  void finish()
  {
    if (worker_.joinable())
    {
      worker_.join();
    }
    if (worker_error_)
    {
      std::rethrow_exception(worker_error_);
    }
  }

private:
  int listen_fd_ = -1;
  int port_ = 0;
  std::thread worker_;
  std::exception_ptr worker_error_;
};

void receive_timeout(Stream& stream, long milliseconds)
{
  timeval timeout{};
  timeout.tv_sec = milliseconds / 1000;
  timeout.tv_usec = (milliseconds % 1000) * 1000;
  stream.setReceiveTimeout(timeout);
}

void test_fragmented_header()
{
  const auto expected = frame(8, 0x51);
  std::promise<void> first_byte_sent;
  auto first_byte_future = first_byte_sent.get_future();
  std::promise<void> release_remaining;
  auto release_future = release_remaining.get_future();
  LoopbackServer server([&](int fd) {
    require(send_all(fd, expected.data(), 1), "could not send first header byte");
    first_byte_sent.set_value();
    require(release_future.wait_for(2s) == std::future_status::ready, "test did not release remaining bytes");
    send_all(fd, expected.data() + 1, expected.size() - 1);
  });

  Stream stream("127.0.0.1", server.port());
  require(stream.connect(1, 1ms), "client did not connect to loopback server");
  receive_timeout(stream, 1000);
  require(first_byte_future.wait_for(2s) == std::future_status::ready, "first header byte was not sent");

  std::array<uint8_t, 32> buffer{};
  buffer.fill(0xA5);  // An incomplete 4-byte header must never be parsed.
  auto reading = std::async(std::launch::async, [&]() {
    size_t bytes_read = 0;
    const bool ok = stream.read(buffer.data(), buffer.size(), bytes_read);
    return std::make_pair(ok, bytes_read);
  });
  const bool returned_before_complete_header = reading.wait_for(200ms) == std::future_status::ready;
  release_remaining.set_value();
  require(reading.wait_for(2s) == std::future_status::ready, "read did not finish after complete frame arrived");
  const auto [ok, bytes_read] = reading.get();
  stream.disconnect();
  server.finish();

  require(!returned_before_complete_header, "read returned before all four header bytes arrived");
  require(ok, "read rejected a valid frame with a fragmented header");
  require(bytes_read == expected.size(), "read returned the wrong frame length");
  require(std::equal(expected.begin(), expected.end(), buffer.begin()), "frame bytes differ from the wire bytes");
}

void test_midbody_timeout_resume()
{
  const auto first = frame(8, 0x61);
  const auto second = frame(7, 0x62);
  std::promise<void> first_fragment_sent;
  auto first_fragment_future = first_fragment_sent.get_future();
  std::promise<void> release_remaining;
  auto release_future = release_remaining.get_future();
  LoopbackServer server([&](int fd) {
    require(send_all(fd, first.data(), 6), "could not send header and partial body");
    first_fragment_sent.set_value();
    require(release_future.wait_for(2s) == std::future_status::ready, "test did not release remaining bytes");
    if (send_all(fd, first.data() + 6, first.size() - 6))
    {
      send_all(fd, second.data(), second.size());
    }
  });

  Stream stream("127.0.0.1", server.port());
  require(stream.connect(1, 1ms), "client did not connect to loopback server");
  receive_timeout(stream, 150);
  require(first_fragment_future.wait_for(2s) == std::future_status::ready, "partial body was not sent");

  std::array<uint8_t, 32> first_buffer{};
  first_buffer.fill(0xA5);
  size_t first_attempt_bytes = 0;
  const bool first_attempt_ok = stream.read(first_buffer.data(), first_buffer.size(), first_attempt_bytes);
  const bool connected_after_timeout = stream.getState() == urcl::comm::SocketState::Connected;
  release_remaining.set_value();
  receive_timeout(stream, 1000);

  std::array<uint8_t, 32> resumed_buffer{};
  resumed_buffer.fill(0xA5);
  size_t resumed_bytes = 0;
  const bool resumed_ok = stream.read(resumed_buffer.data(), resumed_buffer.size(), resumed_bytes);
  std::array<uint8_t, 32> next_buffer{};
  next_buffer.fill(0xA5);
  size_t next_bytes = 0;
  const bool next_ok = stream.read(next_buffer.data(), next_buffer.size(), next_bytes);
  stream.disconnect();
  server.finish();

  require(!first_attempt_ok, "initial read unexpectedly completed before remaining body was released");
  require(connected_after_timeout, "recv timeout disconnected the TCP socket; this test requires a recoverable timeout");
  require(resumed_ok, "read did not resume the partial first frame after a recv timeout");
  require(resumed_bytes == first.size(), "resumed first frame has incorrect length");
  require(std::equal(first.begin(), first.end(), resumed_buffer.begin()), "resumed first frame is corrupted");
  require(next_ok && next_bytes == second.size(), "read did not preserve the boundary of the next frame");
  require(std::equal(second.begin(), second.end(), next_buffer.begin()), "next frame is corrupted");
}

void test_exact_buffer_fit()
{
  const auto expected = frame(4096, 0x73);
  LoopbackServer server([&](int fd) { send_all(fd, expected.data(), expected.size()); });
  Stream stream("127.0.0.1", server.port());
  require(stream.connect(1, 1ms), "client did not connect to loopback server");
  receive_timeout(stream, 1000);
  std::vector<uint8_t> buffer(expected.size(), 0xA5);
  size_t bytes_read = 0;
  const bool ok = stream.read(buffer.data(), buffer.size(), bytes_read);
  stream.disconnect();
  server.finish();
  require(ok, "read rejected a valid frame that exactly fits the supplied buffer");
  require(bytes_read == expected.size(), "exact-fit frame has incorrect length");
  require(buffer == expected, "exact-fit frame is corrupted");
}

void test_invalid_length_fails_closed()
{
  std::array<uint8_t, 4> invalid_length{};
  const uint32_t network_length = htonl(4097);
  std::memcpy(invalid_length.data(), &network_length, sizeof(network_length));
  LoopbackServer server([&](int fd) { send_all(fd, invalid_length.data(), invalid_length.size()); });
  Stream stream("127.0.0.1", server.port());
  require(stream.connect(1, 1ms), "client did not connect to loopback server");
  receive_timeout(stream, 1000);
  std::array<uint8_t, 4096> buffer{};
  size_t bytes_read = 0;
  const bool ok = stream.read(buffer.data(), buffer.size(), bytes_read);
  const bool still_connected = stream.getState() == urcl::comm::SocketState::Connected;
  stream.disconnect();
  server.finish();
  require(!ok, "read accepted a frame length larger than its buffer");
  require(!still_connected, "invalid frame length left the stream connected and desynchronized");
}

void test_bin_parser_oversized_string()
{
  // Keep a guard byte in the allocation so the old unchecked read is defined
  // for this regression; the parser's declared view is only the first 2 bytes.
  std::array<uint8_t, 4> bytes{ 'A', 'B', 'X', 'Y' };
  urcl::comm::BinParser parser(bytes.data(), 2);
  std::string text;
  bool rejected = false;
  try
  {
    parser.parse(text, 3);
  }
  catch (const std::exception&)
  {
    rejected = true;
  }
  require(rejected, "BinParser accepted a string longer than its remaining view");
}

void test_bin_parser_consume_beyond_remaining()
{
  // The extra allocation keeps old pointer arithmetic within a real object;
  // crossing the parser's logical end still must be rejected.
  std::array<uint8_t, 4> bytes{ 'A', 'B', 'X', 'Y' };
  urcl::comm::BinParser parser(bytes.data(), 2);
  bool rejected = false;
  try
  {
    parser.consume(3);
  }
  catch (const std::exception&)
  {
    rejected = true;
  }
  require(rejected, "BinParser consumed beyond its remaining view");
}

void test_bin_parser_short_peek()
{
  std::array<uint8_t, 2> bytes{ 'A', 'B' };
  urcl::comm::BinParser parser(bytes.data(), 1);
  bool rejected = false;
  try
  {
    (void)parser.peek<uint16_t>();
  }
  catch (const std::exception&)
  {
    rejected = true;
  }
  require(rejected, "BinParser read a uint16_t from a one-byte view");
}
}  // namespace

int main(int argc, char** argv)
{
  const std::vector<std::pair<std::string, std::function<void()>>> tests = {
    { "fragmented_header", test_fragmented_header },
    { "midbody_timeout_resume", test_midbody_timeout_resume },
    { "exact_buffer_fit", test_exact_buffer_fit },
    { "invalid_length_fails_closed", test_invalid_length_fails_closed },
    { "bin_parser_oversized_string", test_bin_parser_oversized_string },
    { "bin_parser_consume_beyond_remaining", test_bin_parser_consume_beyond_remaining },
    { "bin_parser_short_peek", test_bin_parser_short_peek },
  };
  const std::string selected = argc > 1 ? argv[1] : "all";
  bool ran = false;
  bool passed = true;
  for (const auto& [name, run] : tests)
  {
    if (selected != "all" && selected != name)
    {
      continue;
    }
    ran = true;
    try
    {
      run();
      std::cout << "PASS " << name << '\n';
    }
    catch (const std::exception& error)
    {
      std::cerr << "FAIL " << name << ": " << error.what() << '\n';
      passed = false;
    }
  }
  if (!ran)
  {
    std::cerr << "Unknown test: " << selected << '\n';
    return 2;
  }
  return passed ? 0 : 1;
}
