#include "pnd_hand_intf.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include "nlohmann/json/json.hpp"
#include "pnd_hand.h"
#include "pnd_hand_old.h"

enum class PndHandVersion { Unknown = -1, HandOld = 0, Hand_0_2_31 = 1 };

std::vector<int> parseVersion(const std::string& versionStr) {
  std::vector<int> versionParts;
  std::stringstream ss(versionStr);
  std::string segment;
  while (std::getline(ss, segment, '.')) {
    versionParts.push_back(std::stoi(segment));
  }
  return versionParts;
}

bool isVersionLater(const std::string& versionToCheck, const std::string& baseVersion) {
  std::vector<int> v1 = parseVersion(versionToCheck);
  std::vector<int> v2 = parseVersion(baseVersion);

  size_t len = std::max(v1.size(), v2.size());

  for (size_t i = 0; i < len; ++i) {
    int part1 = (i < v1.size()) ? v1[i] : 0;
    int part2 = (i < v2.size()) ? v2[i] : 0;

    if (part1 > part2) {
      return true;
    }
    if (part1 < part2) {
      return false;
    }
  }

  return false;
}

class HandInfoClient {
 private:
  int udp_socket;
  struct sockaddr_in server_addr;
  static const int remote_port = 2561;
  static const int timeout_sec = 1;

 public:
  HandInfoClient() {
    // 创建UDP socket
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
      std::cerr << "Failed to create socket" << std::endl;
      return;
    }

    // 设置超时
    struct timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;
    setsockopt(udp_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    // 设置广播选项
    int broadcast = 1;
    setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
  }

  ~HandInfoClient() {
    if (udp_socket >= 0) {
      close(udp_socket);
    }
  }

  PndHandVersion get_device_info(const std::string& addr) {
    PndHandVersion version = PndHandVersion::Unknown;
    // 创建JSON字符串
    std::ostringstream json_stream;
    json_stream << "{\"id\":0,\"method\":\"device.info\"}";
    std::string json_str = json_stream.str();

    // 设置服务器地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(remote_port);
    inet_pton(AF_INET, addr.c_str(), &server_addr.sin_addr);

    // 发送数据
    ssize_t sent_bytes =
        sendto(udp_socket, json_str.c_str(), json_str.length(), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

    if (sent_bytes < 0) {
      std::cerr << "Failed to send data to " << addr << std::endl;
      return version;
    }

    // 接收响应
    char buffer[1024];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    ssize_t received_bytes =
        recvfrom(udp_socket, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&from_addr, &from_len);

    if (received_bytes > 0) {
      buffer[received_bytes] = '\0';
      std::cout << addr << " device info received: " << buffer << std::endl;
      try {
        auto json_response = nlohmann::json::parse(buffer);
        if (json_response.contains("result") && json_response["result"].contains("fw_version")) {
          std::string version_str = json_response["result"]["fw_version"];
          if (isVersionLater(version_str, "0.2.31")) {
            version = PndHandVersion::Unknown;
          } else if (version_str == "0.2.31") {
            version = PndHandVersion::Hand_0_2_31;
          } else {
            version = PndHandVersion::HandOld;
          }
          std::cout << "Device version: " << static_cast<int>(version) << std::endl;
        } else {
          std::cerr << "fw_version info not found in response" << std::endl;
        }
      } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
      }
    } else {
      std::cerr << "Failed to receive data from " << addr << " or timeout occurred" << std::endl;
    }
    return version;
  }
};

PndHandInterface* getPndHandInterface(const std::vector<int>& values) {
  HandInfoClient client;
  PndHandVersion version1 = client.get_device_info("10.10.10.18");
  PndHandVersion version2 = client.get_device_info("10.10.10.38");
  assert(version1 == version2 && version1 != PndHandVersion::Unknown);

  if (version1 == PndHandVersion::HandOld) {
    static PND_HandController instance(values);
    return &instance;
  } else if (version1 == PndHandVersion::Hand_0_2_31) {
    std::vector<int> lr_values;
    lr_values.insert(lr_values.end(), values.begin(), values.end());
    lr_values.insert(lr_values.end(), values.begin(), values.end());
    static PND_NewHandController instance(lr_values);
    return &instance;
  } else {
    std::cerr << "Unsupported PND hand version detected." << std::endl;
    throw std::runtime_error("Unsupported PND hand version");
  }
}
