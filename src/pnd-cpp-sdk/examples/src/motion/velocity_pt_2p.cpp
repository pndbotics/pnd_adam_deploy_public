#include <atomic>
#include <cmath>
#include <iostream>
#include <thread>

#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"

using namespace Pnd;

std::shared_ptr<Group> group;

std::atomic<float> v2p_speed(0.0);
std::atomic_bool v2p_stop(false);
void v2p() {
  GroupCommand group_command(group->size());
  GroupFeedback group_feedback(group->size());
  std::vector<PosPtInfo> pos_pt_infos;
  float cur_pos = 0.0;

  std::vector<MotionControllerConfig*> vc;
  for (int i = 0; i < group->size(); ++i) {
    MotionControllerConfig* c = new MotionControllerConfig();
    c->pos_gain = 0.6;
    c->vel_gain = 0.01;
    c->vel_integrator_gain = 0.04;
    c->vel_limit = 40;
    c->vel_limit_tolerance = 1.2;
    vc.emplace_back(c);
  }
  group_command.setMotionCtrlConfig(vc);
  group->sendCommand(group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  for (auto c : vc) {
    delete c;
  }

  group_command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  group_command.resetLinearCount(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);

  group_command.enable(std::vector<float>(group->size(), 1));
  group->sendCommand(group_command);

  for (int i = 0; i < group_command.size(); ++i) {
    pos_pt_infos.push_back(PosPtInfo());
  }

  while (!v2p_stop) {
    if (v2p_speed.load() != 0.0) {
      group->getNextFeedback(group_feedback, 10);
      cur_pos += v2p_speed.load() / 200.0;
      for (int j = 0; j < group_command.size(); ++j) {
        pos_pt_infos[j].pos = cur_pos;
        pos_pt_infos[j].vel_ff = v2p_speed.load();
        pos_pt_infos[j].torque_ff = 0.0;
      }
      group_command.setInputPositionPt(pos_pt_infos);
      group->sendCommand(group_command);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  group_command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

int main() {
  std::string str("10.10.10.255");

  pndSetLogLevel("ERROR", "ERROR");
  // After construction,start the background thread lookup actuator
  Lookup lookup(&str);

  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);  // set lookup stop

  group = lookup.getGroupFromFamily("Default");
  std::cout << std::endl << "group size: " << group->size() << std::endl;
  if (group->size() == 0) {
    std::cout << "No actuator found, exit" << std::endl;
    return 0;
  }

  std::thread t(v2p);
  for (int i = 0; i < 800; ++i) {
    v2p_speed.store(i / 50.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  for (int i = 800; i > -800; --i) {
    v2p_speed.store(i / 50.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  for (int i = -800; i <= 0; ++i) {
    v2p_speed.store(i / 50.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  for (int i = 0; i <= 800; ++i) {
    v2p_speed.store(5.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  v2p_stop.store(true);
  t.join();

  return 0;
}