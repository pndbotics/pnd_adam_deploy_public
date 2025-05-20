#include <iostream>
#include <thread>

#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"

using namespace Pnd;

int main() {
  std::string str("10.10.10.255");
  Lookup lookup(&str);
  // After construction,start the background thread lookup actuator
  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);  // set lookup stop

  std::shared_ptr<Group> group = lookup.getGroupFromFamily("RCU");
  if (!group) {
    std::cout << "Group not found! Check that the family and name of a module "
                 "on the network"
              << std::endl
              << "matches what is given in the source file." << std::endl;
    return -1;
  }
  std::cout << std::endl << "rcu group size: " << group->size() << std::endl;

  auto command = std::make_shared<GroupCommand>(group->size());
  auto feedback = std::make_shared<GroupFeedback>(group->size());

  bool flag = false;

  command->RcuPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuV5VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuAV5VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuV5VAPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuV5VBPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->Rcu12VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuFan12VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->Rcu19VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  RcuBrake rcu_brake;
  rcu_brake.brake_enable = false;
  rcu_brake.brake_overvoltage = 49.1;
  rcu_brake.brake_factor = 2049;
  std::vector<RcuBrake> rcu_brake_vec;
  for (int i = 0; i < group->size(); ++i) rcu_brake_vec.push_back(rcu_brake);
  command->RcuBrakeCmd(rcu_brake_vec);
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuAdcOffset(std::vector<float>(group->size(), 42));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  std::this_thread::sleep_for(std::chrono::seconds(2));

  flag = !flag;
  command->RcuPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuV5VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuAV5VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuV5VAPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuV5VBPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->Rcu12VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuFan12VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->Rcu19VPower(std::vector<bool>(group->size(), flag));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  rcu_brake.brake_enable = true;
  rcu_brake.brake_overvoltage = 49.0;
  rcu_brake.brake_factor = 2048;
  rcu_brake_vec.clear();
  for (int i = 0; i < group->size(); ++i) rcu_brake_vec.push_back(rcu_brake);
  command->RcuBrakeCmd(rcu_brake_vec);
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  command->RcuAdcOffset(std::vector<float>(group->size(), 0));
  group->sendCommand(*command);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  return 0;
}