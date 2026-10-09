// VerticalBot Studio — minimal C++17 client over the TCP JSON-lines API (port 20501).
// Header-only; requires a JSON library (nlohmann/json single header). POSIX sockets.
//   #include "robolink.hpp"
//   vbs::Robolink rdk("127.0.0.1", 20501);
//   auto robot = rdk.item("", 2);            // ITEM_TYPE_ROBOT
//   rdk.call("MoveJ", robot, {{0,-90,90,0,90,0}});
#pragma once
#include <string>
#include <stdexcept>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "json.hpp"

namespace vbs {
using json = nlohmann::json;

class Robolink {
 public:
  Robolink(const std::string& host = "127.0.0.1", int port = 20501) {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(port); inet_pton(AF_INET, host.c_str(), &a.sin_addr);
    if (::connect(fd_, (sockaddr*)&a, sizeof(a)) != 0) throw std::runtime_error("cannot connect to studio server");
  }
  ~Robolink() { if (fd_ >= 0) ::close(fd_); }

  /// Generic call. `target` is an item object returned by item()/add* (or json() for Robolink methods).
  json call(const std::string& method, const json& target = json(), const json& params = json::array()) {
    json req = {{"id", ++seq_}, {"method", method}, {"params", params}};
    if (!target.is_null() && target.contains("$item")) req["target"] = target["$item"];
    std::string line = req.dump() + "\n";
    if (::send(fd_, line.data(), line.size(), 0) < 0) throw std::runtime_error("send failed");
    std::string buf; char c;
    while (::recv(fd_, &c, 1, 0) == 1) { if (c == '\n') break; buf += c; }
    json res = json::parse(buf);
    if (res.contains("error") && !res["error"].is_null()) throw std::runtime_error(res["error"].get<std::string>());
    return res["result"];
  }
  json item(const std::string& name, int type = -1) { return call("Item", json(), {name, type}); }
  json addFrame(const std::string& name) { return call("AddFrame", json(), {name}); }
  json addTarget(const std::string& name, const json& parent, const json& robot) { return call("AddTarget", json(), {name, parent, robot}); }
  json addProgram(const std::string& name, const json& robot) { return call("AddProgram", json(), {name, robot}); }
  static json pose(const double m[4][4]) { json rows = json::array(); for (int r = 0; r < 4; r++) { json row = json::array(); for (int c = 0; c < 4; c++) row.push_back(m[r][c]); rows.push_back(row); } return {{"$pose", rows}}; }
 private:
  int fd_ = -1; int seq_ = 0;
};
}  // namespace vbs
