
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
#include <corecrt_math_defines.h>
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
  pndSetLogLevel("ERROR", "INFO");
  // After construction,start the background thread lookup actuator
  Lookup lookup(&str);
  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);

  auto group1 = lookup.getGroupFromIps({"10.10.10.30"});
  auto group = lookup.getGroupFromIps({"10.10.10.34"});

  if (!group) {
    std::cout << "Group not found! Check that the family and name of a module "
                 "on the network"
              << std::endl
              << "matches what is given in the source file." << std::endl;
    return -1;
  }

  std::cout << "group size: " << group->size() << std::endl;
  GroupCommand group_command(group->size());
  GroupFeedback group_feedback(group->size());

  GroupCommand group_command1(group1->size());
  GroupFeedback group_feedback1(group1->size());

  group_command1.enable(std::vector<float>(group1->size(), 1));
  group1->sendCommand(group_command1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  group_command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto ret = group->getNextFeedback(group_feedback, 10);
  if (ret) {
    for (int i = 0; i < group_feedback.size(); ++i) {
      if (group_feedback[i]->enabled) {
        std::cout << "disable failed!" << std::endl;
        exit(-1);
      }
    }
  }

  group_command.resetLinearCount(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);

  group_command.enable(std::vector<float>(group->size(), 1));
  group->sendCommand(group_command);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ret = group->getNextFeedback(group_feedback, 10);
  if (ret) {
    for (int i = 0; i < group_feedback.size(); ++i) {
      if (!group_feedback[i]->enabled) {
        std::cout << "enable failed!" << std::endl;
        exit(-1);
      }
    }
  }

  std::vector<PosPtInfo> pos_pt_infos;
  std::vector<PosPtInfo> pre_pos_infos;
  PosPtInfo pos;
  int timeout_num = 0;
  int next_feedback_timeout = 0;

  for (int j = 0; j < 1000; ++j) {
    StartTimeChrono(all);
    // StartTimeChrono(feedback);
    ret = group->getNextFeedback(group_feedback, 10);
    // EndTimeChrono(feedback);
    if (!ret) {
      next_feedback_timeout++;
      continue;
    }

    group_command1.setInputVelocityPt(std::vector<float>(group1->size(), 1));
    group1->sendCommand(group_command1);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));

    for (int i = 0; i < group_feedback.size(); ++i) {
      if (std::isnan(group_feedback[i]->position)) {
        timeout_num++;
        pos.pos = pre_pos_infos.at(i).pos;
        // if (timeout_num > 3) return 0;
      } else {
        // std::cout << group_feedback[i]->position << " ";
        // pos.pos = group_feedback[i]->position + 0.1;
        // pos.pos += 0.03;
        pos.pos = sin(j * 0.002 * M_PI) * 3;
      }
      pos.vel_ff = 0.0;
      pos.torque_ff = 0.0;
      pos_pt_infos.push_back(pos);
    }
    pre_pos_infos = pos_pt_infos;
    // std::cout << std::endl;
    StartTimeChrono(command);
    group_command.setInputPositionPt(pos_pt_infos);
    group->sendCommand(group_command);
    EndTimeChrono(command);
    pos_pt_infos.clear();
    std::cout << "sdk command duration:" << group_feedback.Duration() << " us" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    EndTimeChrono(all);
  }

  group_command1.setInputVelocityPt(std::vector<float>(group1->size(), 0));
  group1->sendCommand(group_command1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  group_command1.enable(std::vector<float>(group1->size(), 0));
  group1->sendCommand(group_command1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  group_command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "feedback timeout:" << next_feedback_timeout << " recv timeout:" << timeout_num << std::endl;

  return 0;
}
