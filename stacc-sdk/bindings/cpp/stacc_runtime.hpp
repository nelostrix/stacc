// stacc_runtime.hpp — C++17 client for the STACC controller runtime API.
//
// Single header, no dependencies beyond POSIX sockets. It speaks the same
// versioned /v1/runtime/* JSON contract over loopback HTTP as the Python
// client (stacc/server/runtime_client.py), so a controller is portable between
// the two. Units are SI: radians, seconds, newton-metres, metres.
//
//   /* <<<RSS>>>
//   robot arm { body { shape: box; } arm main { joints: 3; reach: 600mm; } }
//   <<</RSS>>> */
//   #include "stacc_runtime.hpp"
//   int main() {
//     stacc::Runtime rt;            // 127.0.0.1:8766
//     rt.load_file(__FILE__);       // the robot described in this file's RSS block
//     rt.start();
//     rt.command_joint("arm_main_2_joint", 0.5);
//     for (auto& j : rt.joints()) std::printf("%s %.3f\n", j.name.c_str(), j.position);
//   }
#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace stacc {

// ───────────────────────────── minimal JSON value ─────────────────────────────

struct Json {
  enum class Type { Null, Bool, Number, String, Array, Object };
  Type type = Type::Null;
  bool boolean = false;
  double number = 0.0;
  std::string string;
  std::vector<Json> array;
  std::vector<std::pair<std::string, Json>> object;

  bool is_null() const { return type == Type::Null; }

  /** Object member, or a null value when missing / not an object. */
  const Json& operator[](const std::string& key) const {
    static const Json null_value;
    if (type == Type::Object)
      for (const auto& kv : object)
        if (kv.first == key) return kv.second;
    return null_value;
  }

  /** Number as double; `fallback` when not a number (e.g. null). */
  double num(double fallback = NAN) const { return type == Type::Number ? number : fallback; }
  std::string str(const std::string& fallback = "") const { return type == Type::String ? string : fallback; }

  static Json parse(const std::string& text) {
    size_t i = 0;
    Json v = parse_value(text, i);
    skip_ws(text, i);
    if (i != text.size()) throw std::runtime_error("stacc: trailing data in JSON response");
    return v;
  }

 private:
  static void skip_ws(const std::string& s, size_t& i) {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) ++i;
  }

  static Json parse_value(const std::string& s, size_t& i) {
    skip_ws(s, i);
    if (i >= s.size()) throw std::runtime_error("stacc: unexpected end of JSON");
    Json v;
    char c = s[i];
    if (c == '{') {
      v.type = Type::Object;
      ++i;
      skip_ws(s, i);
      if (s[i] == '}') { ++i; return v; }
      while (true) {
        skip_ws(s, i);
        std::string key = parse_string(s, i);
        skip_ws(s, i);
        if (s[i++] != ':') throw std::runtime_error("stacc: expected ':' in JSON");
        v.object.emplace_back(key, parse_value(s, i));
        skip_ws(s, i);
        if (s[i] == ',') { ++i; continue; }
        if (s[i] == '}') { ++i; return v; }
        throw std::runtime_error("stacc: expected ',' or '}' in JSON");
      }
    }
    if (c == '[') {
      v.type = Type::Array;
      ++i;
      skip_ws(s, i);
      if (s[i] == ']') { ++i; return v; }
      while (true) {
        v.array.push_back(parse_value(s, i));
        skip_ws(s, i);
        if (s[i] == ',') { ++i; continue; }
        if (s[i] == ']') { ++i; return v; }
        throw std::runtime_error("stacc: expected ',' or ']' in JSON");
      }
    }
    if (c == '"') { v.type = Type::String; v.string = parse_string(s, i); return v; }
    if (s.compare(i, 4, "true") == 0) { v.type = Type::Bool; v.boolean = true; i += 4; return v; }
    if (s.compare(i, 5, "false") == 0) { v.type = Type::Bool; i += 5; return v; }
    if (s.compare(i, 4, "null") == 0) { i += 4; return v; }
    char* end = nullptr;
    v.number = std::strtod(s.c_str() + i, &end);
    if (end == s.c_str() + i) throw std::runtime_error("stacc: invalid JSON value");
    v.type = Type::Number;
    i = static_cast<size_t>(end - s.c_str());
    return v;
  }

  static std::string parse_string(const std::string& s, size_t& i) {
    if (s[i] != '"') throw std::runtime_error("stacc: expected string in JSON");
    ++i;
    std::string out;
    while (i < s.size() && s[i] != '"') {
      char c = s[i++];
      if (c != '\\') { out += c; continue; }
      char e = s[i++];
      switch (e) {
        case 'n': out += '\n'; break;
        case 't': out += '\t'; break;
        case 'r': out += '\r'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'u': {  // BMP code point -> UTF-8
          unsigned cp = static_cast<unsigned>(std::stoul(s.substr(i, 4), nullptr, 16));
          i += 4;
          if (cp < 0x80) out += static_cast<char>(cp);
          else if (cp < 0x800) { out += static_cast<char>(0xC0 | (cp >> 6)); out += static_cast<char>(0x80 | (cp & 0x3F)); }
          else { out += static_cast<char>(0xE0 | (cp >> 12)); out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); out += static_cast<char>(0x80 | (cp & 0x3F)); }
          break;
        }
        default: out += e;  // \" \\ \/
      }
    }
    ++i;  // closing quote
    return out;
  }
};

inline std::string json_escape(const std::string& in) {
  std::string out = "\"";
  for (unsigned char c : in) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) { char buf[8]; std::snprintf(buf, sizeof buf, "\\u%04x", c); out += buf; }
        else out += static_cast<char>(c);
    }
  }
  return out + "\"";
}

inline std::string json_number_array(const std::vector<double>& values) {
  std::ostringstream os;
  os.precision(17);
  os << '[';
  for (size_t k = 0; k < values.size(); ++k) os << (k ? "," : "") << values[k];
  os << ']';
  return os.str();
}

// ───────────────────────────────── the client ─────────────────────────────────

struct JointState {
  std::string id;
  std::string name;
  double position = NAN;  // rad (m for prismatic)
  double velocity = NAN;  // rad/s
  double torque = NAN;    // N·m actually applied
};

class RuntimeError : public std::runtime_error {
 public:
  RuntimeError(const std::string& message, int status) : std::runtime_error(message), status(status) {}
  int status;
};

class Runtime {
 public:
  explicit Runtime(std::string host = "127.0.0.1", int port = 8766, double timeout_s = 10.0)
      : host_(std::move(host)), port_(port), timeout_s_(timeout_s) {}

  // ── lifecycle ──
  // `file`: where the source came from, so `use "parts.rss:x";` resolves next to it.
  Json load_source(const std::string& source, const std::string& format = "rss", const std::string& file = "") {
    std::string body = "{\"format\":" + json_escape(format) + ",\"source\":" + json_escape(source);
    if (!file.empty()) body += ",\"file\":" + json_escape(file);
    return post("/v1/runtime/load_project", body + "}");
  }

  /** Load a `.rss` file, or a C++/Rust/Python file with an RSS block (pass __FILE__). */
  Json load_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("stacc: cannot read " + path);
    std::stringstream buf;
    buf << in.rdbuf();
    size_t dot = path.find_last_of('.');
    return load_source(buf.str(), dot == std::string::npos ? "rss" : path.substr(dot + 1),
                       std::filesystem::absolute(path).string());
  }

  Json start() { return post("/v1/runtime/start", "{}"); }
  Json pause() { return post("/v1/runtime/pause", "{}"); }
  Json resume() { return post("/v1/runtime/resume", "{}"); }
  Json reset() { return post("/v1/runtime/reset", "{}"); }
  Json stop() { return post("/v1/runtime/stop", "{}"); }
  Json step(int count = 1) { return post("/v1/runtime/step", "{\"count\":" + std::to_string(count) + "}"); }

  // ── state ──
  Json telemetry() { return get("/v1/runtime/telemetry"); }
  Json capabilities() { return get("/v1/runtime/capabilities"); }

  std::vector<JointState> joints() {
    std::vector<JointState> out;
    // Keep the response alive: looping over a member of a temporary dangles.
    const Json data = get("/v1/runtime/joints");
    for (const auto& j : data["joints"].array) {
      JointState s;
      s.id = j["id"].str();
      s.name = j["name"].str();
      s.position = j["position"].num();
      s.velocity = j["velocity"].num();
      s.torque = j["torque"].num();
      out.push_back(s);
    }
    return out;
  }

  /** Base pose [x, y, z, roll, pitch, yaw] (mobile robots move; fixed ones stay at 0). */
  std::vector<double> pose() {
    std::vector<double> out;
    const Json data = telemetry();
    for (const auto& v : data["pose"].array) out.push_back(v.num(0.0));
    return out;
  }

  // ── commands ──
  Json command_joint(const std::string& joint, double angle_rad) {
    std::ostringstream os;
    os.precision(17);
    os << "{\"joint\":" << json_escape(joint) << ",\"angle_rad\":" << angle_rad << "}";
    return post("/v1/runtime/joints/command", os.str());
  }
  Json command_positions(const std::vector<double>& positions_rad) {
    return post("/v1/runtime/joints/command", "{\"positions\":" + json_number_array(positions_rad) + "}");
  }
  /** Direct joint torques (N·m); clears any position command. */
  Json command_torques(const std::vector<double>& torques_nm) {
    return post("/v1/runtime/joints/command", "{\"torques\":" + json_number_array(torques_nm) + "}");
  }
  /** Rotor thrust (N) in the robot's rotor order; clamped to each rotor's maximum. */
  Json command_thrusts(const std::vector<double>& thrusts_n) {
    return post("/v1/runtime/joints/command", "{\"thrusts\":" + json_number_array(thrusts_n) + "}");
  }
  // ── shared-memory control channel (the data plane) ──

  /** Enable the channel ("realtime" or "lockstep") and return its /dev/shm path. */
  std::string enable_shm(const std::string& mode = "realtime", double timeout_ms = 1000) {
    std::ostringstream os;
    os << "{\"enable\":true,\"mode\":" << json_escape(mode) << ",\"timeout_ms\":" << timeout_ms << "}";
    return post("/v1/runtime/shm", os.str())["path"].str();
  }
  Json shm_status() { return get("/v1/runtime/shm"); }
  Json disable_shm() { return post("/v1/runtime/shm", "{\"enable\":false}"); }

  Json gains(double kp, double kd) {
    std::ostringstream os;
    os << "{\"kp\":" << kp << ",\"kd\":" << kd << "}";
    return post("/v1/runtime/control/gains", os.str());
  }

 private:
  Json get(const std::string& path) { return request("GET", path, ""); }
  Json post(const std::string& path, const std::string& body) { return request("POST", path, body); }

  Json request(const std::string& method, const std::string& path, const std::string& body) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) throw std::runtime_error("stacc: socket() failed");
    std::unique_ptr<int, void (*)(int*)> guard(&fd, [](int* f) { ::close(*f); });
    timeval tv{};
    tv.tv_sec = static_cast<long>(timeout_s_);
    tv.tv_usec = static_cast<long>((timeout_s_ - static_cast<long>(timeout_s_)) * 1e6);
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) throw std::runtime_error("stacc: bad host " + host_);
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0)
      throw RuntimeError("stacc: daemon unreachable on " + host_ + ":" + std::to_string(port_), 0);

    std::string req = method + " " + path + " HTTP/1.0\r\nHost: " + host_ + "\r\n";
    if (method == "POST") req += "Content-Type: application/json\r\nContent-Length: " + std::to_string(body.size()) + "\r\n";
    req += "\r\n" + body;
    for (size_t sent = 0; sent < req.size();) {
      ssize_t n = ::send(fd, req.data() + sent, req.size() - sent, 0);
      if (n <= 0) throw std::runtime_error("stacc: send failed");
      sent += static_cast<size_t>(n);
    }

    std::string resp;
    char chunk[16384];
    ssize_t n;
    while ((n = ::recv(fd, chunk, sizeof chunk, 0)) > 0) resp.append(chunk, static_cast<size_t>(n));
    size_t header_end = resp.find("\r\n\r\n");
    if (header_end == std::string::npos) throw std::runtime_error("stacc: malformed HTTP response");
    int status = std::atoi(resp.c_str() + resp.find(' ') + 1);
    Json data = Json::parse(resp.substr(header_end + 4));
    if (status >= 400 || !data["error"].is_null())
      throw RuntimeError("stacc: " + data["error"].str("HTTP " + std::to_string(status)), status);
    return data;
  }

  std::string host_;
  int port_;
  double timeout_s_;
};

/**
 * Shared-memory control channel (layout v1; see stacc/server/rt_channel.py).
 * The runtime writes the state block every control cycle, you write the
 * command block; both are seqlocks, so neither side ever blocks the other and
 * a reader never sees a half-written frame.
 *
 *   stacc::ControlChannel ch(rt.enable_shm("lockstep"));
 *   stacc::ControlChannel::State s;
 *   ch.wait_state(-1, s);
 *   for (;;) {
 *     ch.command_positions(target_for(s), s.step);   // answer this step
 *     if (!ch.wait_state(s.step, s)) break;          // next step's state
 *   }
 */
class ControlChannel {
 public:
  static constexpr uint32_t kMagic = 0x54525453;  // 'STRT'
  static constexpr uint32_t kVersion = 1;
  enum Kind : uint32_t { kNone = 0, kPosition = 1, kTorque = 2 };

  struct State {
    uint64_t step = 0;
    double sim_time = 0;
    double pose[7] = {0, 0, 0, 1, 0, 0, 0};  // x y z qw qx qy qz
    std::vector<double> position, velocity, torque, thrust;
  };

  explicit ControlChannel(const std::string& path) {
    fd_ = ::open(path.c_str(), O_RDWR);
    if (fd_ < 0) throw std::runtime_error("stacc: cannot open control channel " + path);
    struct stat st {};
    ::fstat(fd_, &st);
    size_ = static_cast<size_t>(st.st_size);
    void* p = ::mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (p == MAP_FAILED) { ::close(fd_); throw std::runtime_error("stacc: mmap failed for " + path); }
    base_ = static_cast<unsigned char*>(p);
    uint32_t header[6];
    std::memcpy(header, base_, sizeof header);
    if (header[0] != kMagic) throw std::runtime_error("stacc: " + path + " is not a STACC control channel");
    if (header[1] != kVersion) throw std::runtime_error("stacc: unsupported control channel layout");
    n_joints_ = header[2];
    n_thrusters_ = header[3];
    mode_ = header[4];
    std::memcpy(&state_off_, base_ + 24, 8);
    std::memcpy(&cmd_off_, base_ + 32, 8);
  }
  ~ControlChannel() {
    if (base_) ::munmap(base_, size_);
    if (fd_ >= 0) ::close(fd_);
  }
  ControlChannel(const ControlChannel&) = delete;
  ControlChannel& operator=(const ControlChannel&) = delete;

  uint32_t n_joints() const { return n_joints_; }
  uint32_t n_thrusters() const { return n_thrusters_; }
  bool lockstep() const { return mode_ == 1; }

  /** Latest consistent state; false only if the writer stayed mid-update. */
  bool read_state(State& out) const {
    const size_t n = 7 + 3 * n_joints_ + n_thrusters_;
    std::vector<double> v(n);
    for (int attempt = 0; attempt < 10000; ++attempt) {
      uint64_t s1 = seq(state_off_).load(std::memory_order_acquire);
      if (s1 & 1) continue;
      std::memcpy(&out.step, base_ + state_off_ + 8, 8);
      std::memcpy(&out.sim_time, base_ + state_off_ + 16, 8);
      std::memcpy(v.data(), base_ + state_off_ + 24, n * 8);
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq(state_off_).load(std::memory_order_relaxed) != s1) continue;
      std::memcpy(out.pose, v.data(), 7 * 8);
      const double* p = v.data() + 7;
      out.position.assign(p, p + n_joints_);
      out.velocity.assign(p + n_joints_, p + 2 * n_joints_);
      out.torque.assign(p + 2 * n_joints_, p + 3 * n_joints_);
      out.thrust.assign(p + 3 * n_joints_, p + 3 * n_joints_ + n_thrusters_);
      return true;
    }
    return false;
  }

  /** Spin until the runtime publishes a step newer than `after_step` (pass -1 for any). */
  bool wait_state(int64_t after_step, State& out, double timeout_s = 1.0) const {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      if (read_state(out) && static_cast<int64_t>(out.step) > after_step) return true;
    }
    return false;
  }

  uint64_t command_positions(const std::vector<double>& rad, uint64_t answers_step, const std::vector<double>* thrust = nullptr) {
    return write_command(kPosition, rad, answers_step, thrust);
  }
  uint64_t command_torques(const std::vector<double>& nm, uint64_t answers_step, const std::vector<double>* thrust = nullptr) {
    return write_command(kTorque, nm, answers_step, thrust);
  }

 private:
  std::atomic<uint64_t>& seq(uint64_t off) const { return *reinterpret_cast<std::atomic<uint64_t>*>(base_ + off); }

  uint64_t write_command(uint32_t kind, const std::vector<double>& values, uint64_t answers_step, const std::vector<double>* thrust) {
    if (values.size() != n_joints_) throw std::runtime_error("stacc: wrong number of joint values for this robot");
    if (thrust && thrust->size() != n_thrusters_) throw std::runtime_error("stacc: wrong number of thrust values for this robot");
    unsigned char* c = base_ + cmd_off_;
    uint64_t last_id;
    std::memcpy(&last_id, c + 8, 8);
    const uint64_t id = (next_id_ > last_id ? next_id_ : last_id) + 1;
    next_id_ = id;
    auto& s = seq(cmd_off_);
    const uint64_t start = s.load(std::memory_order_relaxed) + 1;  // odd: writing
    s.store(start, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    const uint32_t flags = thrust ? 1u : 0u;
    std::memcpy(c + 8, &id, 8);
    std::memcpy(c + 16, &answers_step, 8);
    std::memcpy(c + 24, &kind, 4);
    std::memcpy(c + 28, &flags, 4);
    std::memcpy(c + 32, values.data(), n_joints_ * 8);
    if (thrust) std::memcpy(c + 32 + n_joints_ * 8, thrust->data(), n_thrusters_ * 8);
    s.store(start + 1, std::memory_order_release);  // even: consistent
    return id;
  }

  int fd_ = -1;
  size_t size_ = 0;
  unsigned char* base_ = nullptr;
  uint32_t n_joints_ = 0, n_thrusters_ = 0, mode_ = 0;
  uint64_t state_off_ = 0, cmd_off_ = 0, next_id_ = 0;
};

}  // namespace stacc
