#include "pnd_hand_old.h"

PND_HandController::PND_HandController(const std::vector<int>& values) {
  // Create a socket
  sok_l_ = socket(AF_INET, SOCK_DGRAM, 0);
  sok_r_ = socket(AF_INET, SOCK_DGRAM, 0);
  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(sok_l_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(sok_r_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // init hand pos
  auto sbytes = handCtrlPack(values);

  struct sockaddr_in addr_l_in, addr_r_in;
  memset(&addr_l_in, 0, sizeof(addr_l_in));
  memset(&addr_r_in, 0, sizeof(addr_r_in));

  addr_l_in.sin_family = AF_INET;
  addr_l_in.sin_port = htons(port_);
  addr_l_in.sin_addr.s_addr = inet_addr(addr_l_);

  addr_r_in.sin_family = AF_INET;
  addr_r_in.sin_port = htons(port_);
  addr_r_in.sin_addr.s_addr = inet_addr(addr_r_);

  sendto(sok_l_, sbytes.data(), sbytes.size(), 0, (struct sockaddr*)&addr_l_in, sizeof(addr_l_in));
  sendto(sok_r_, sbytes.data(), sbytes.size(), 0, (struct sockaddr*)&addr_r_in, sizeof(addr_r_in));
}

PND_HandController::~PND_HandController() {
  close(sok_l_);
  close(sok_r_);
  std::cout << "PND_HandController destructed, thread terminated." << std::endl;
}

std::vector<uint8_t> PND_HandController::addChecksum(std::vector<uint8_t> sbytes) {
  uint8_t checksum = 0x00;
  for (size_t i = 2; i < sbytes.size(); i++) {
    checksum += sbytes[i];
  }
  checksum &= 0xFF;
  sbytes.push_back(checksum);
  return sbytes;
}

std::vector<uint8_t> PND_HandController::handCreatPack(uint8_t id, uint8_t cmd, uint8_t index,
                                                       const std::vector<uint8_t>& data) {
  std::vector<uint8_t> sbytes = {0x55, 0xAA};
  sbytes.push_back(data.size() + 2);
  sbytes.push_back(id);
  sbytes.push_back(cmd);
  sbytes.push_back(index);

  sbytes.insert(sbytes.end(), data.begin(), data.end());
  return addChecksum(sbytes);
}

std::vector<uint8_t> PND_HandController::handCtrlPack(const std::vector<int>& position,
                                                      const std::vector<uint8_t>& id) {
  if (id.size() != position.size() || id.size() > 6 || id.empty()) {
    return std::vector<uint8_t>();
  }

  std::vector<uint8_t> sbytes = {0x55, 0xAA};
  sbytes.push_back(1 + 3 * id.size());
  sbytes.push_back(0xff);  // id
  sbytes.push_back(0xF2);  // cmd

  for (size_t i = 0; i < id.size(); i++) {
    sbytes.push_back(id[i]);
    sbytes.push_back(position[i] & 0xFF);
    sbytes.push_back((position[i] >> 8) & 0xFF);
  }

  return addChecksum(sbytes);
}

void PND_HandController::ctrl(const std::vector<int>& pos) {
  if (pos.size() != 12) {
    std::cerr << "Error: Position vector must contain exactly 12 values" << std::endl;
    return;
  }

  struct sockaddr_in addr_l_in, addr_r_in;
  memset(&addr_l_in, 0, sizeof(addr_l_in));
  memset(&addr_r_in, 0, sizeof(addr_r_in));

  addr_l_in.sin_family = AF_INET;
  addr_l_in.sin_port = htons(port_);
  addr_l_in.sin_addr.s_addr = inet_addr(addr_l_);

  addr_r_in.sin_family = AF_INET;
  addr_r_in.sin_port = htons(port_);
  addr_r_in.sin_addr.s_addr = inet_addr(addr_r_);

  std::vector<int> left_pos(pos.begin(), pos.begin() + 6);
  std::vector<int> right_pos(pos.begin() + 6, pos.end());

  auto sbytesl = handCtrlPack(left_pos);
  auto sbytesr = handCtrlPack(right_pos);

  sendto(sok_l_, sbytesl.data(), sbytesl.size(), 0, (struct sockaddr*)&addr_l_in, sizeof(addr_l_in));
  sendto(sok_r_, sbytesr.data(), sbytesr.size(), 0, (struct sockaddr*)&addr_r_in, sizeof(addr_r_in));
  // }
}