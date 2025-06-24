#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <imu.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

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

  // 设置广播选项
  int broadcast = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast))) {
    perror("setsockopt (SO_BROADCAST) failed");
    close(sockfd);
    exit(EXIT_FAILURE);
  }

  // 设置超时
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

void get_device_info(const std::string& addr) {
  int sockfd = create_udp_socket();

  // 准备JSON数据
  json data = {{"id", 1}, {"method", "device.info"}};
  std::string json_str = data.dump();

  // 设置目标地址
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(REMOTE_PORT);
  servaddr.sin_addr.s_addr = inet_addr(addr.c_str());

  // 发送数据
  ssize_t sent_bytes =
      sendto(sockfd, json_str.c_str(), json_str.length(), 0, (const struct sockaddr*)&servaddr, sizeof(servaddr));
  if (sent_bytes < 0) {
    perror("sendto failed");
    close(sockfd);
    return;
  }

  // 接收响应
  char buffer[BUFFER_SIZE];
  struct sockaddr_in from_addr;
  socklen_t from_len = sizeof(from_addr);

  ssize_t recv_bytes = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&from_addr, &from_len);
  if (recv_bytes < 0) {
    perror("recvfrom failed or timeout");
    close(sockfd);
    return;
  }

  // 打印接收到的信息
  buffer[recv_bytes] = '\0';
  std::cout << addr << " device info received " << buffer << std::endl;

  close(sockfd);
}

ImuType checkImuType() {
  get_device_info("255.255.255.255");
  ImuType imu_type = ImuType::UNKNOWN;
  // if (results.empty()) {
  //   std::cerr << "[ERROR] No IMU devices found" << std::endl;
  // } else if (results.begin()->second.find("model") != results.begin()->second.end() &&
  //            results.begin()->second["model"] == "PND_IMU_01") {
  //   std::cout << "[INFO] Detected VN100 IMU" << std::endl;
  //   imu_type = ImuType::VN100_IMU;
  // } else if (results.begin()->second.find("model") != results.begin()->second.end() &&
  //            results.begin()->second["model"] == "PND_IMU_02") {
  //   std::cout << "[INFO] Detected CH IMU" << std::endl;
  //   imu_type = ImuType::CH_IMU;
  // } else {
  //   std::cerr << "[ERROR] IMU type detection failed" << std::endl;
  //   std::cerr << "[ERROR] Please check the device connection and ensure it is a supported IMU." << std::endl;
  //   std::cerr << "[ERROR] If the device is supported, please report this issue to the developers." << std::endl;
  // }
  return imu_type;
}

// int main() {
//   get_device_info("255.255.255.255");  // 使用广播地址
//   return 0;
// }