
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"

using namespace Pnd;

int main() {
  std::string str("10.10.10.255");
  pndSetLogLevel("ERROR", "INFO");
  // After construction,start the background thread lookup actuator
  Lookup lookup(&str);
  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);

  auto group = lookup.getGroupFromFamily("Default", 150);

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
  ret = group->getNextFeedback(group_feedback, 100);
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

  for (int j = 0; j < 10000; ++j) {
    ret = group->getNextFeedback(group_feedback, 10);
    if (!ret) { // get next feedback failed
      next_feedback_timeout++;
      continue;
    }
    for (int i = 0; i < group_feedback.size(); ++i) {
      if (std::isnan(group_feedback[i]->position)) { // timeout
        timeout_num++;
        pos.pos = pre_pos_infos.at(i).pos;
      } else {
        pos.pos = sin(j * 0.002 * M_PI) * 3;
      }
      pos.vel_ff = 0.0;
      pos.torque_ff = 0.0;
      pos_pt_infos.push_back(pos);
    }
    pre_pos_infos = pos_pt_infos;
    group_command.setInputPositionPt(pos_pt_infos);
    group->sendCommand(group_command);
    pos_pt_infos.clear();
    // std::cout << "sdk command duration:" << group_feedback.Duration() << " us" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  group_command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);
  std::cout << "feedback timeout:" << next_feedback_timeout << " recv timeout:" << timeout_num << std::endl;

  return 0;
}
