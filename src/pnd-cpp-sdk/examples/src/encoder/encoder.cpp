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
  pndSetLogLevel(NULL, NULL);

  std::shared_ptr<Group> group = lookup.getGroupFromFamily("PNDencoder");
  if (!group) {
    std::cout << "Group not found! Check that the family and name of a module "
                 "on the network"
              << std::endl
              << "matches what is given in the source file." << std::endl;
    return -1;
  }
  std::cout << std::endl << "group size: " << group->size() << std::endl;

  GroupCommand group_command(group->size());
  GroupFeedback group_feedback(group->size());
  group_command.getEncoderAngle(std::vector<float>(group->size(), 1));

  auto startTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::seconds(100);
  while (std::chrono::high_resolution_clock::now() - startTime < duration) {
    group->sendCommand(group_command);
    auto ret = group->getNextFeedback(group_feedback, 10);
    if (ret) {
      for (int i = 0; i < group_feedback.size(); ++i) {
        std::cout << group_feedback[i]->encoder_angle << std::endl;
      }
    } else {
      std::cout << "getNextFeedback failed!" << std::endl;
      exit(-1);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  return 0;
}