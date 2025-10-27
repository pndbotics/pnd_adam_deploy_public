#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "pnd_hand_intf.hpp"

class PND_NewHandController : public PndHandInterface {
 private:
  enum HandSide { Left = 0, Right = 1 };

  struct HandState {
    std::array<uint16_t, 6> current{};
    std::array<uint16_t, 6> position{};
    std::array<uint8_t, 6> err{};
  };

  struct HandContext {
    const char *ip;
    int sock = -1;
    sockaddr_in addr{};
    std::array<int, 6> position;                // 改为固定大小数组减少内存分配
    std::array<int, 6> new_position;            // 固定大小数组
    std::array<uint8_t, 6> protection_flags{};  // 更合适的数据类型 (0/1标志)
    std::array<std::chrono::steady_clock::time_point, 6> prot_start_time{};
    HandState state;
    int prot_threshold = 1000;
    std::thread thread;
    int core = -1;
    std::array<uint8_t, 24> ctrl_packet;  // 预分配控制包缓冲区
  };

 public:
  explicit PND_NewHandController(const std::vector<int> &values);
  ~PND_NewHandController();

  void setNewPosition(const std::vector<int> &pos);
  void setPosition(const std::vector<int> &pos) override { setNewPosition(pos); }

 private:
  void start();
  void stop();

  void notifyOnce(HandSide side);
  void bindThreadToCore(std::thread &thread, int core_id);
  void initAddr(sockaddr_in &addr, const char *ip);

  void updateControlPacket(HandSide side, const std::array<int, 6> &target);  // 新增控制包更新方法

  void checkError(HandSide side);
  void parsePacket(HandSide side, const uint8_t *data, size_t len);
  bool receivePacket(HandSide side, uint8_t *buffer, size_t buffer_size);

  void sendBytes(HandSide side, const uint8_t *data,
                 size_t size);  // 避免vector拷贝
  void logPacket(const std::string &prefix, const uint8_t *packet, size_t len, const std::string &target);
  std::string to_hex(uint8_t value) const;  // const方法

  void ctrlHand(HandSide side);
  void handLoop(HandSide side);

 private:
  static constexpr int HAND_SIZE = 12;
  static constexpr int port_ = 2562;
  bool debug_enabled_ = false;
  bool enable_prot_ = true;

  std::atomic<bool> running_{true};

  // 新的用于控制每只手的后台线程唤醒
  std::condition_variable_any hand_cv_[2];
  std::mutex hand_mutex_[2];
  std::atomic<bool> hand_triggered_[2];

  HandContext hands_[2] = {
      {"10.10.10.18"},  // Left
      {"10.10.10.38"}   // Right
  };
};
