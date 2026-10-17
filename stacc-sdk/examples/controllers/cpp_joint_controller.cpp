// Example C++ controller: drive a joint through the STACC runtime API.
//
// This is the C++ binding to the SAME versioned /v1/runtime/* contract the
// Python controller uses (spec §4, acceptance criterion 6): a standalone
// process that speaks loopback HTTP JSON, with no native ABI and no physics
// engine linked in. It commands one joint, advances the simulation, then prints
// the runtime telemetry JSON to stdout so a caller (or the Studio) can read it.
//
// Build (no third-party dependencies, POSIX sockets only):
//   g++ -O2 -std=c++17 -o cpp_joint_controller cpp_joint_controller.cpp
//
// Run against a live daemon:
//   ./cpp_joint_controller --host 127.0.0.1 --port 8766 --joint J2 --angle 0.5 --steps 400
//
// Exit codes: 0 = telemetry printed, 1 = transport/HTTP error.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

namespace {

struct Response {
  int status = 0;
  std::string body;
};

// Minimal HTTP/1.1 request over a blocking loopback socket. The STACC daemon
// answers with Content-Length, so we read headers then exactly that many bytes.
Response http_request(const std::string& host, int port, const std::string& method,
                      const std::string& path, const std::string& body) {
  Response out;
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    out.status = -1;
    return out;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    ::close(fd);
    out.status = -1;
    return out;
  }
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    out.status = -1;
    return out;
  }

  std::ostringstream req;
  req << method << " " << path << " HTTP/1.1\r\n";
  req << "Host: " << host << ":" << port << "\r\n";
  req << "Accept: application/json\r\n";
  req << "Connection: close\r\n";
  if (!body.empty()) {
    req << "Content-Type: application/json\r\n";
    req << "Content-Length: " << body.size() << "\r\n";
  }
  req << "\r\n";
  req << body;
  const std::string payload = req.str();

  size_t sent = 0;
  while (sent < payload.size()) {
    ssize_t n = ::send(fd, payload.data() + sent, payload.size() - sent, 0);
    if (n <= 0) {
      ::close(fd);
      out.status = -1;
      return out;
    }
    sent += static_cast<size_t>(n);
  }

  std::string raw;
  char buf[4096];
  ssize_t n;
  while ((n = ::recv(fd, buf, sizeof(buf), 0)) > 0) {
    raw.append(buf, static_cast<size_t>(n));
  }
  ::close(fd);

  size_t header_end = raw.find("\r\n\r\n");
  if (header_end == std::string::npos) {
    out.status = -1;
    return out;
  }
  std::string headers = raw.substr(0, header_end);
  out.body = raw.substr(header_end + 4);

  size_t sp = headers.find(' ');
  if (sp != std::string::npos) {
    out.status = std::atoi(headers.c_str() + sp + 1);
  }
  return out;
}

std::string arg_value(int argc, char** argv, const std::string& flag, const std::string& fallback) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) return argv[i + 1];
  }
  return fallback;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string host = arg_value(argc, argv, "--host", "127.0.0.1");
  const int port = std::atoi(arg_value(argc, argv, "--port", "8766").c_str());
  const std::string joint = arg_value(argc, argv, "--joint", "J2");
  const std::string angle = arg_value(argc, argv, "--angle", "0.5");
  const std::string steps = arg_value(argc, argv, "--steps", "400");

  // 1. Command the joint (radians) — the same endpoint the Python controller calls.
  Response command = http_request(host, port, "POST", "/v1/runtime/joints/command",
                                  "{\"joint\":\"" + joint + "\",\"angle_rad\":" + angle + "}");
  if (command.status != 200) {
    std::cerr << "[cpp-controller] joint command failed: HTTP " << command.status
              << " " << command.body << "\n";
    return 1;
  }

  // 2. Advance the simulation so the servo settles.
  Response stepped = http_request(host, port, "POST", "/v1/runtime/step",
                                  "{\"count\":" + steps + "}");
  if (stepped.status != 200) {
    std::cerr << "[cpp-controller] step failed: HTTP " << stepped.status
              << " " << stepped.body << "\n";
    return 1;
  }

  // 3. Read telemetry and print it verbatim for the caller to parse.
  Response telemetry = http_request(host, port, "GET", "/v1/runtime/telemetry", "");
  if (telemetry.status != 200) {
    std::cerr << "[cpp-controller] telemetry failed: HTTP " << telemetry.status
              << " " << telemetry.body << "\n";
    return 1;
  }
  std::cout << telemetry.body << std::endl;
  return 0;
}
