#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <imu.hpp>
#include <iostream>
#include <string>

#include "nlohmann/json/json.hpp"

using json = nlohmann::json;

#define REMOTE_PORT 8085
#define BUFFER_SIZE 1024
#define TIMEOUT_SEC 1
#define TIMEOUT_USEC 0

int create_udp_socket() {
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    perror("socket creation failed");
    exit(EXIT_FAILURE);
  }

  int broadcast = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast))) {
    perror("setsockopt (SO_BROADCAST) failed");
    close(sockfd);
    exit(EXIT_FAILURE);
  }

  struct timeval tv;
  tv.tv_sec = TIMEOUT_SEC;
  tv.tv_usec = TIMEOUT_USEC;
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    perror("setsockopt (SO_RCVTIMEO) failed");
    close(sockfd);
    exit(EXIT_FAILURE);
  }

  return sockfd;
}

void get_device_info(const std::string& addr, json& results) {
  int sockfd = create_udp_socket();

  json data = {{"id", 1}, {"method", "device.info"}};
  std::string json_str = data.dump();

  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(REMOTE_PORT);
  servaddr.sin_addr.s_addr = inet_addr(addr.c_str());

  ssize_t sent_bytes =
      sendto(sockfd, json_str.c_str(), json_str.length(), 0, (const struct sockaddr*)&servaddr, sizeof(servaddr));
  if (sent_bytes < 0) {
    perror("sendto failed");
    close(sockfd);
    return;
  }

  char buffer[BUFFER_SIZE];
  struct sockaddr_in from_addr;
  socklen_t from_len = sizeof(from_addr);

  ssize_t recv_bytes = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&from_addr, &from_len);
  if (recv_bytes < 0) {
    perror("recvfrom failed or timeout");
    close(sockfd);
    return;
  }

  buffer[recv_bytes] = '\0';
  // std::cout << addr << " device info received " << buffer << std::endl;

  try {
    results = json::parse(buffer);
    std::cout << "Received JSON: " << results.dump(4) << std::endl;
  } catch (const json::parse_error& e) {
    std::cerr << "JSON parse error: " << e.what() << std::endl;
  }

  close(sockfd);
}

ImuType checkImuType() {
  json results;
  get_device_info("127.0.0.1", results);
  ImuType imu_type = ImuType::VN100_IMU;
  if (results.contains("imu") && results["imu"].size() > 0) {
    auto imu_info = results["imu"].begin()->get<json>();
    if (imu_info.contains("model")) {
      std::string model = imu_info["model"];
      if (model == "PND_IMU_01") {
        std::cout << "[INFO] Detected VN100 IMU" << std::endl;
        imu_type = ImuType::VN100_IMU;
      } else if (model == "PND_IMU_02") {
        std::cout << "[INFO] Detected CH IMU" << std::endl;
        imu_type = ImuType::CH_IMU;
      } else {
        std::cerr << "[ERROR] Unsupported IMU model: " << model << std::endl;
      }
    } else {
      std::cerr << "[ERROR] IMU model not found in device info" << std::endl;
    }
  } else {
    std::cerr << "[ERROR] No IMU devices found in device info" << std::endl;
  }
  return imu_type;
}

// int main() {
//   get_device_info("255.255.255.255");  // 使用广播地址
//   return 0;
// }