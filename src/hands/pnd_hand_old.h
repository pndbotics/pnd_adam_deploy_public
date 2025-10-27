#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include "pnd_hand_intf.hpp"

class PND_HandController : public PndHandInterface {
 private:
  std::vector<uint8_t> addChecksum(std::vector<uint8_t> sbytes);
  std::vector<uint8_t> handCreatPack(uint8_t id, uint8_t cmd, uint8_t index, const std::vector<uint8_t>& data);
  std::vector<uint8_t> handCtrlPack(const std::vector<int>& position,
                                    const std::vector<uint8_t>& id = {1, 2, 3, 4, 5, 6});

  const int port_ = 2562;
  const char* addr_l_ = "10.10.10.18";
  const char* addr_r_ = "10.10.10.38";
  int sok_l_, sok_r_;

 public:
  void ctrl(const std::vector<int>& pos);
  void setPosition(const std::vector<int>& pos) override { ctrl(pos); }
  PND_HandController(const std::vector<int>& values);
  ~PND_HandController();
};