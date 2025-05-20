#include <iostream>
#include <thread>

#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"

using namespace Pnd;

const int num = 300;

int main() {
  std::string str("10.10.10.255");

  // After construction,start the background thread lookup actuator
  Lookup lookup(&str);
  pndSetLogLevel("ERROR", "ERROR");

  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);  // set lookup stop

  std::shared_ptr<Group> group = lookup.getGroupFromFamily("Default");
  std::cout << std::endl << "group size: " << group->size() << std::endl;
  if (group->size() == 0) {
    std::cout << "No actuator found, exit" << std::endl;
    return 0;
  }
  GroupCommand group_command(group->size());
  GroupFeedback group_feedback(group->size());

  group_command.enable(std::vector<float>(group->size(), 1));
  group->sendCommand(group_command);

  group->getNextFeedback(group_feedback);
  for (int i = 0; i < group->size(); ++i) {
    if (!group_feedback[i]->enabled) {
      std::cout << "Actuator " << i << " is not enabled, exit" << std::endl;
      return 0;
    }
  }

  for (int i = 1; i < num; ++i) {
    group_command.setInputVelocityPt(std::vector<float>(group->size(), i / 50.0));
    group->sendCommand(group_command);
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  for (int i = num; i > 0; --i) {
    group_command.setInputVelocityPt(std::vector<float>(group->size(), i / 50.0));
    group->sendCommand(group_command);
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  group_command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);

  return 0;
}