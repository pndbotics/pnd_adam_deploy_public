#include <iostream>
#include <thread>

#include "lookup.hpp"

using namespace Pnd;

int main() {
  std::string str("10.10.10.255");
  pndSetLogLevel("INFO", "INFO");
  Lookup lookup(&str);
  // After construction,start the background thread lookup actuator
  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);  // set lookup stop

  std::shared_ptr<Group> group = lookup.getGroupFromFamily("Default");
  if (!group) {
    std::cout << "Group not found! Check that the family and name of a module "
                 "on the network"
              << std::endl
              << "matches what is given in the source file." << std::endl;
    return -1;
  }
  std::cout << std::endl << "group size: " << group->size() << std::endl;

  group = lookup.getGroupFromFamily("PNDriver");
  std::cout << std::endl << "PNDriver group size: " << group->size() << std::endl;

  auto entry_list = lookup.getEntryList();
  for (const auto &entry : *entry_list) {
    std::cout << "Name: " << entry.name_ << std::endl;
    std::cout << "Family: " << entry.family_ << std::endl;
  }
  return 0;
}