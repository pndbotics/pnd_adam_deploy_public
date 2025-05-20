
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"

int main() {
  // broadcast address
  std::string str("10.10.10.255");
  std::shared_ptr<Pnd::Lookup> lookup = std::make_shared<Pnd::Lookup>(&str);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup->setLookupFrequencyHz(0);
  auto group = lookup->getGroupFromFamily("Default");
  pndSetLogLevel(NULL, NULL);
  Pnd::GroupCommand command(group->size());
  Pnd::GroupFeedback feedback(group->size());

  std::cout << std::endl;

  auto entry_list = lookup->getEntryList();
  for (const auto &entry : *entry_list) {
    std::cout << "Name: " << entry.name_ << std::endl;
    std::cout << "Family: " << entry.family_ << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  command.enable(std::vector<float>(group->size(), 1));
  group->sendCommand(command);

  std::vector<bool> v(group->size(), true);
  std::vector<PosPtInfo> pos_pt_infos;
  for (int i = 0; i < group->size(); ++i) {
    pos_pt_infos.push_back({0, 0, 0});
  }

  auto startTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::seconds(10 * 60);
  std::ofstream outfile("communication_time.txt");
  if (!outfile.is_open()) {
    std::cout << "Failed to open file" << std::endl;
    return 1;
  }
  while (std::chrono::high_resolution_clock::now() - startTime < duration) {
    // command.setLatencyTest(v);
    // group->sendCommand(command);
    // group->getNextFeedback(feedback, 5);
    // std::cout << "communication time:" << feedback.Duration() << std::endl;

    command.setInputPositionPt(pos_pt_infos);
    group->sendCommand(command);
    group->getNextFeedback(feedback, 5);
    std::cout << "pt communication time:" << feedback.Duration() << std::endl;

    outfile << feedback.Duration() << std::endl;
  }

  command.enable(std::vector<float>(group->size(), 0));
  group->sendCommand(command);
  outfile.close();

  return 0;
}
