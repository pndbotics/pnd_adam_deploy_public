#include <iostream>
#include <thread>

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
  auto device_list = lookup.getDeviceList();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  std::cout << "device list size: " << device_list->size << std::endl;
  for (int i = 0; i < device_list->size; ++i) {
    std::cout << "ip: " << device_list->deviceInfo[i].ip << std::endl;
    std::cout << "serial_number: " << device_list->deviceInfo[i].serial_number << std::endl;
    std::cout << "mac_address: " << device_list->deviceInfo[i].mac_address << std::endl;
    std::cout << "model: " << device_list->deviceInfo[i].model << std::endl;
  }
  pndLookupDeviceListFree(device_list);
  return 0;
}