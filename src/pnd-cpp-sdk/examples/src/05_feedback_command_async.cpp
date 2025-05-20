
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"

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
// for test

int main(int argc, char *argv[]) {
  // Try and get the requested group.
  std::shared_ptr<Pnd::Group> group;

  std::string str("10.10.10.255");
  Pnd::Lookup lookup(&str);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  lookup.setLookupFrequencyHz(0);
  group = lookup.getGroupFromFamily("Default");
  if (!group) {
    std::cout << "No group found!" << std::endl;
    return -1;
  }

  pndSetLogLevel("ERROR", NULL);

  Pnd::GroupFeedback feedback(group->size());

  Pnd::GroupCommand group_command(group->size());
  group_command.resetLinearCount(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);
  // std::this_thread::sleep_for(std::chrono::milliseconds(5));

  group_command.enable(std::vector<float>(group->size(), 1));
  group->sendCommand(group_command);
  // std::this_thread::sleep_for(std::chrono::milliseconds(5));

  auto start = std::chrono::system_clock::now();
  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
  double duration = 1;
  std::vector<float> v_pos;
  std::vector<PosPtInfo> pos_pt_infos;
  while (t.count() < duration) {
    t = std::chrono::system_clock::now() - start;
    // StartTimeChrono(all);

    // StartTimeChrono(getNextFeedback);
    group->getNextFeedback(feedback, 2);
    // EndTimeChrono(getNextFeedback);
    for (size_t mod_idx = 0; mod_idx < feedback.size(); ++mod_idx) {
      if (feedback[mod_idx]->position != std::numeric_limits<float>::quiet_NaN()) {
        PosPtInfo info = {0};
        info.pos = feedback[mod_idx]->position + .1;
        pos_pt_infos.push_back(info);
        std::cout << "pos:" << feedback[mod_idx]->position << "  "
                  << "vel:" << feedback[mod_idx]->velocity << "  "
                  << "cur:" << feedback[mod_idx]->current << "  ";
      } else {
        std::cout << "******* nan *****" << std::endl;
      }
      std::cout << std::endl;
    }

    StartTimeChrono(sendCommand);
    group_command.setInputPositionPt(pos_pt_infos);
    group->sendCommand(group_command);
    EndTimeChrono(sendCommand);

    //  std::this_thread::sleep_for(std::chrono::milliseconds(10));
    pos_pt_infos.clear();
    // EndTimeChrono(all);
  }

  group_command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);

  return 0;
}
