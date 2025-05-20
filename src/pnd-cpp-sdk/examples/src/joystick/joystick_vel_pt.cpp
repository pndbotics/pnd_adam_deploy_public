#include <iostream>
#include <thread>

#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "joystick.hpp"
#include "lookup.hpp"

using namespace Pnd;

int main() {
  std::string str("10.10.10.255");

  // After construction,start the background thread lookup actuator
  Lookup lookup(&str);

  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);  // set lookup stop
  pndSetLogLevel();
  Joystick joystick;
  joystick.run();

  std::shared_ptr<Group> group = lookup.getGroupFromFamily("Default");
  std::cout << std::endl << "group size: " << group->size() << std::endl;
  GroupCommand group_command(group->size());

  group_command.enable(std::vector<float>(group->size(), 1));
  group->sendCommand(group_command);

  float cur_vel = 0;

  while (true) {
    if (joystick.key_.load() == 'q') {
      break;
    }
    int axis_value = joystick.axis_front_rear_.load();
    axis_value = axis_value / 32768.0 * 40.0;
    if (axis_value > cur_vel) {
      cur_vel += 0.15;
    } else if (axis_value < cur_vel) {
      cur_vel -= 0.2;
    }
    std::cout << cur_vel << std::endl;

    group_command.setInputVelocityPt(std::vector<float>(group->size(), cur_vel));
    group->sendCommand(group_command);

    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }

  group_command.setInputVelocityPt(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);

  group_command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(group_command);

  return 0;
}