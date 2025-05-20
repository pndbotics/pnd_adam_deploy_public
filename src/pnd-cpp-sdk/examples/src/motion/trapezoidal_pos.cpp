
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"

using namespace Pnd;

// for test
#ifdef WIN32
#include <WS2tcpip.h>
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#define DllExport __declspec(dllexport)
#else
#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include <time.h>

#define StartTimeChrono(funName)                                                                                  \
  std::chrono::microseconds ms##funName =                                                                         \
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()); \
  long start##funName = ms##funName.count();

#define EndTimeChrono(funName)                                                                                    \
  ms##funName =                                                                                                   \
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()); \
  long end##funName = ms##funName.count();                                                                        \
  double funName##ms_time = (end##funName - start##funName) / 1000.0;                                             \
  std::cout << "The function " << #funName << " runs for " << funName##ms_time << "ms" << std::endl;

int main() {
  std::string str("10.10.10.255");
  // After construction,start the background thread lookup actuator
  Lookup lookup(&str);
  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);

  auto group = lookup.getGroupFromFamily("Default");

  if (!group) {
    std::cout << "Group not found! Check that the family and name of a module "
                 "on the network"
              << std::endl
              << "matches what is given in the source file." << std::endl;
    return -1;
  }

  std::cout << group->size() << std::endl;
  GroupCommand group_command(group->size());

  GroupFeedback group_feedback(group->size());

  group_command.resetLinearCount(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);

  group_command.enable(std::vector<float>(group->size(), 1));
  group->sendCommand(group_command);

  std::vector<float> pos_infos;
  int timeout_num = 0;

  for (int i = 0; i < 10; ++i) {
    StartTimeChrono(all);
    // StartTimeChrono(feedback);
    auto ret = group->getNextFeedback(group_feedback, 10);
    // EndTimeChrono(feedback);
    if (ret) {
      for (int j = 0; j < group_feedback.size(); ++j) {
        if (group_feedback[j]->position == std::numeric_limits<float>::quiet_NaN()) {
          timeout_num++;
          std::cout << "***********************************" << std::endl;
          if (timeout_num > 3) return 0;
        }
        std::cout << "p:" << group_feedback[j]->position << " ";
        // pos_infos.push_back(i / 10.0);
        pos_infos.push_back(i);
      }
      std::cout << std::endl;
      StartTimeChrono(command);
        group_command.setTrapezoidalMove(pos_infos);
      // group_command.setPosition(pos_infos);
      group->sendCommand(group_command);
      EndTimeChrono(command);
      pos_infos.clear();
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    EndTimeChrono(all);
  }

  group_command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);
  std::cout << "end!!!" << std::endl;

  return 0;
}
