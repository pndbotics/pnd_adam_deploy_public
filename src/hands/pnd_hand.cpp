#include "pnd_hand.h"

#include <algorithm>
#include <iostream>
#include <sstream>

void PND_NewHandController::bindThreadToCore(std::thread &thread, int core_id) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpuset);
}

PND_NewHandController::PND_NewHandController(const std::vector<int> &initPos) {
  // 预定义的控制包模板 (不变部分)
  constexpr std::array<uint8_t, 24> packet_template = {
      0x55, 0xAA, 0x13, 0xFF, 0xF2,  // Header
      1,    0,    0,                 // Finger 1
      2,    0,    0,                 // Finger 2
      3,    0,    0,                 // Finger 3
      4,    0,    0,                 // Finger 4
      5,    0,    0,                 // Finger 5
      6,    0,    0,                 // Finger 6
      0                              // 校验位
  };

  for (int side = 0; side < 2; ++side) {
    // 初始化固定大小数组 (修复后)
    std::copy_n(initPos.begin() + (side * 6), 6, hands_[side].new_position.begin());
    hands_[side].protection_flags.fill(0);  // 使用更简洁的fill方法

    // 初始化控制包
    hands_[side].ctrl_packet = packet_template;

    // 创建socket
    hands_[side].sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (hands_[side].sock < 0) {
      std::cerr << "ERROR: Socket creation failed for side " << side << "\n";
    }

    // 设置socket超时
    struct timeval tv = {0, 10000};  // 10ms timeout
    setsockopt(hands_[side].sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 初始化地址
    initAddr(hands_[side].addr, hands_[side].ip);
  }

  hands_[Left].core = 5;
  hands_[Right].core = 6;

  // 初始化控制包为目标位置（否则 ctrl_packet 里是模板）
  updateControlPacket(Left, hands_[Left].new_position);
  updateControlPacket(Right, hands_[Right].new_position);

  // 启动线程（线程内会周期性读取并发送 ctrl_packet）
  start();
}

PND_NewHandController::~PND_NewHandController() {
  stop();  // 先停止线程

  for (int side = 0; side < 2; ++side) {
    if (hands_[side].sock >= 0) {
      close(hands_[side].sock);
    }
  }
}

void PND_NewHandController::start() {
  running_ = true;
  for (int side = 0; side < 2; ++side) {
    hand_triggered_[side] = false;
    hands_[side].thread = std::thread(&PND_NewHandController::handLoop, this, static_cast<HandSide>(side));
    bindThreadToCore(hands_[side].thread, hands_[side].core);
  }
}

void PND_NewHandController::stop() {
  running_ = false;
  for (int side = 0; side < 2; ++side) {
    if (hands_[side].thread.joinable()) {
      hands_[side].thread.join();
    }
  }
}

void PND_NewHandController::notifyOnce(HandSide side) {
  {
    std::lock_guard<std::mutex> lock(hand_mutex_[side]);
    hand_triggered_[side] = true;
  }
  hand_cv_[side].notify_one();
}

void PND_NewHandController::initAddr(sockaddr_in &addr, const char *ip) {
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  addr.sin_addr.s_addr = inet_addr(ip);
}

void PND_NewHandController::setNewPosition(const std::vector<int> &pos) {
  if (pos.size() < HAND_SIZE) return;

  // 直接拷贝数据到固定数组，避免vector分配
  std::copy_n(pos.begin(), 6, hands_[Left].new_position.begin());
  std::copy_n(pos.begin() + 6, 6, hands_[Right].new_position.begin());

  notifyOnce(Right);
  notifyOnce(Left);
}

void PND_NewHandController::updateControlPacket(HandSide side, const std::array<int, 6> &target) {
  auto &packet = hands_[side].ctrl_packet;

  // 更新位置数据 (跳过5字节头部)
  for (int i = 0; i < 6; ++i) {
    packet[6 + i * 3] = target[i] & 0xFF;         // 低字节
    packet[7 + i * 3] = (target[i] >> 8) & 0xFF;  // 高字节
  }

  // 计算校验和 (从索引2到22)
  uint8_t checksum = 0;
  for (int i = 2; i < 23; ++i) {
    checksum += packet[i];
  }
  packet[23] = checksum;
}

void PND_NewHandController::handLoop(HandSide side) {
  // 必须是500Hz，即2ms，太快则不会进入过流保护
  // 因为数据更新太快了
  const auto interval = std::chrono::milliseconds(2);
  auto next_time = std::chrono::steady_clock::now();

  while (running_) {
    std::unique_lock<std::mutex> lock(hand_mutex_[side]);

    hand_cv_[side].wait_until(lock, next_time, [&] { return hand_triggered_[side] || !running_; });

    if (!running_) break;

    ctrlHand(side);

    if (hand_triggered_[side]) {
      hand_triggered_[side] = false;
    }

    next_time += interval;
  }
}

void PND_NewHandController::ctrlHand(HandSide side) {
  auto &ctx = hands_[side];
  auto target = ctx.new_position;  // 拷贝防止直接修改

  auto now = std::chrono::steady_clock::now();

  for (int i = 0; i < 6; ++i) {
    if (ctx.state.current[i] > ctx.prot_threshold) {
      if (ctx.protection_flags[i] == 0) {
        ctx.protection_flags[i] = 1;
        ctx.prot_start_time[i] = now;
      }
      target[i] = ctx.state.position[i];
    }

    if (ctx.protection_flags[i]) {
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - ctx.prot_start_time[i]).count();
      if (elapsed_ms >= 100) {
        ctx.protection_flags[i] = 0;
      } else {
        target[i] = ctx.state.position[i];
      }
    }
  }

  // 控制命令发送
  updateControlPacket(side, target);
  logPacket("SEND", ctx.ctrl_packet.data(), ctx.ctrl_packet.size(), ctx.ip);
  sendBytes(side, ctx.ctrl_packet.data(), ctx.ctrl_packet.size());

  // 接收反馈
  uint8_t buffer[64];  // 缓冲区
  if (receivePacket(side, buffer, sizeof(buffer))) {
    parsePacket(side, buffer, sizeof(buffer));
  }

  // 调试信息输出
  if (debug_enabled_) {
    std::cout << (side == Left ? "[LEFT]" : "[RIGHT]") << " Status:\n";

    std::cout << "  Currents: ";
    for (auto c : ctx.state.current) std::cout << c << " ";
    std::cout << "\n";

    std::cout << "  Positions: ";
    for (auto p : ctx.state.position) std::cout << p << " ";
    std::cout << "\n";

    std::cout << "  Targets: ";
    for (auto t : target) std::cout << t << " ";
    std::cout << "\n";

    std::cout << "  ProtFlags: ";
    for (auto f : ctx.protection_flags) std::cout << static_cast<int>(f) << " ";
    std::cout << "\n";
  }
}

bool PND_NewHandController::receivePacket(HandSide side, uint8_t *buffer, size_t buffer_size) {
  auto &ctx = hands_[side];
  socklen_t addr_len = sizeof(ctx.addr);
  int recv_len = recvfrom(ctx.sock, buffer, buffer_size, 0, reinterpret_cast<sockaddr *>(&ctx.addr), &addr_len);

  return (recv_len >= 33);  // 最小有效包长度
}

void PND_NewHandController::parsePacket(HandSide side, const uint8_t *data, size_t len) {
  auto &ctx = hands_[side];

  // 获取当前时间
  auto now = std::chrono::steady_clock::now();

  if (len >= 33 && data[0] == 0xAA && data[1] == 0xBB) {
    // 解析电流、电机位置、错误码
    for (int i = 0; i < 6; ++i) {
      ctx.state.current[i] = data[2 + i * 2] | (data[3 + i * 2] << 8);
      ctx.state.position[i] = data[14 + i * 2] | (data[15 + i * 2] << 8);
      ctx.state.err[i] = data[26 + i];
    }

    checkError(side);
  }
}

void PND_NewHandController::checkError(HandSide side) {
  auto &ctx = hands_[side];
  for (int i = 0; i < 6; ++i) {
    uint8_t code = ctx.state.err[i];
    if (code != 0) {
      // 使用字符串缓存减少多次输出
      std::ostringstream oss;
      oss << "Motor " << (i + 1) << " error: ";
      if (code & 0x01) oss << "堵转保护 ";
      if (code & 0x02) oss << "过温保护 ";
      if (code & 0x04) oss << "过流保护 ";
      if (code & 0x08) oss << "电机异常 ";
      std::cerr << oss.str() << "\n";
    }
  }
}

void PND_NewHandController::sendBytes(HandSide side, const uint8_t *data, size_t size) {
  auto &ctx = hands_[side];
  sendto(ctx.sock, data, size, 0, (sockaddr *)&ctx.addr, sizeof(ctx.addr));
}

std::string PND_NewHandController::to_hex(uint8_t value) const {
  // 使用静态变量避免重复构建
  static const char *hex_digits = "0123456789ABCDEF";
  std::string result;
  result.reserve(2);
  result.push_back(hex_digits[value >> 4]);
  result.push_back(hex_digits[value & 0xF]);
  return result;
}

void PND_NewHandController::logPacket(const std::string &prefix, const uint8_t *packet, size_t len,
                                      const std::string &target) {
  if (!debug_enabled_) return;

  // 预分配足够空间减少多次分配
  std::ostringstream ss;
  ss << "NETWORK " << prefix << " to " << target << " [";

  for (size_t i = 0; i < len; ++i) {
    if (i != 0) ss << " ";
    ss << to_hex(packet[i]);
  }
  ss << "]";

  std::cout << ss.str() << std::endl;
}
