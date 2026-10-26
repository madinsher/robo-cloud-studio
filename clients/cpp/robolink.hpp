// VerticalBot Studio - minimal C++17 client over the TCP JSON-lines API (port 20501).
// Header-only; needs nlohmann/json (single header json.hpp, https://github.com/nlohmann/json, MIT) on the include
// path. Sockets: Winsock2 on Windows (link ws2_32), BSD sockets elsewhere.
//   #include "robolink.hpp"
//   vbs::Robolink rdk("127.0.0.1", 20501);
//   auto robot = rdk.item("", 2);            // ITEM_TYPE_ROBOT
//   rdk.call("MoveJ", robot, {{0,-90,90,0,90,0}});
//   auto pose = rdk.call("Pose", robot);     // {"$pose": [[r0],[r1],[r2],[r3]]}
// The official RoboDK C++ API (RoboDK API for C++) also works unmodified when the studio server runs the RoboDK
// binary protocol (STUDIO_RDK_PORT, see docs/api.md).
#pragma once
#include <string>
#include <stdexcept>
#include <cstring>
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "ws2_32.lib")
#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#endif
#include "json.hpp"

namespace vbs {
using json = nlohmann::json;

class Robolink {
 public:
  Robolink(const std::string& host = "127.0.0.1", int port = 20501) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) throw std::runtime_error("WSAStartup failed");
#endif
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(static_cast<unsigned short>(port));
    inet_pton(AF_INET, host.c_str(), &a.sin_addr);
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) throw std::runtime_error("cannot connect to the studio server");
  }
  ~Robolink() {
#ifdef _WIN32
    if (fd_ != invalid()) ::closesocket(fd_);
    WSACleanup();
#else
    if (fd_ != invalid()) ::close(fd_);
#endif
  }
  Robolink(const Robolink&) = delete;
  Robolink& operator=(const Robolink&) = delete;

  /// Generic call. `target` is an item object returned by item()/add* (or json() for Robolink methods).
  json call(const std::string& method, const json& target = json(), const json& params = json::array()) {
    json req = {{"id", ++seq_}, {"method", method}, {"params", params}};
    if (!target.is_null() && target.contains("$item")) req["target"] = target["$item"];
    const std::string line = req.dump() + "\n";
    size_t sent = 0;
    while (sent < line.size()) {
      const int n = ::send(fd_, line.data() + sent, static_cast<int>(line.size() - sent), 0);
      if (n <= 0) throw std::runtime_error("send failed");
      sent += static_cast<size_t>(n);
    }
    const json res = json::parse(readLine());
    if (res.contains("error") && !res["error"].is_null()) throw std::runtime_error(res["error"].get<std::string>());
    return res.contains("result") ? res["result"] : json();
  }
  json item(const std::string& name, int type = -1) { return call("Item", json(), {name, type}); }
  json addFrame(const std::string& name) { return call("AddFrame", json(), {name}); }
  json addTarget(const std::string& name, const json& parent, const json& robot) { return call("AddTarget", json(), {name, parent, robot}); }
  json addProgram(const std::string& name, const json& robot) { return call("AddProgram", json(), {name, robot}); }
  /// 4x4 row-major pose -> wire format.
  static json pose(const double m[4][4]) {
    json rows = json::array();
    for (int r = 0; r < 4; r++) { json row = json::array(); for (int c = 0; c < 4; c++) row.push_back(m[r][c]); rows.push_back(row); }
    return {{"$pose", rows}};
  }

 private:
#ifdef _WIN32
  using socket_t = SOCKET;
  static socket_t invalid() { return INVALID_SOCKET; }
#else
  using socket_t = int;
  static socket_t invalid() { return -1; }
#endif
  std::string readLine() {
    for (;;) {
      const size_t i = buf_.find('\n');
      if (i != std::string::npos) { std::string line = buf_.substr(0, i); buf_.erase(0, i + 1); return line; }
      char chunk[65536];
      const int n = ::recv(fd_, chunk, sizeof(chunk), 0);
      if (n <= 0) throw std::runtime_error("connection closed by the studio server");
      buf_.append(chunk, static_cast<size_t>(n));
    }
  }
  socket_t fd_ = invalid();
  int seq_ = 0;
  std::string buf_;
};
}  // namespace vbs
