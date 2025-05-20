#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"

using namespace Pnd;

void getConfigParam(std::shared_ptr<Group> group, Pnd::GroupFeedback& feedback) {
  group->sendFeedbackRequest();
  if (group->getNextFeedback(feedback, 15)) {
    for (size_t mod_idx = 0; mod_idx < feedback.size(); ++mod_idx) {
      std::cout << "\t pos:" << feedback[mod_idx]->position << "\t vel:" << feedback[mod_idx]->velocity
                << "\t cur:" << feedback[mod_idx]->current << "\t error_code:" << feedback[mod_idx]->error_code
                << "\t voltage:" << feedback[mod_idx]->voltage
                << "\t motor_temp_m1:" << feedback[mod_idx]->motor_temp_m1
                << "\t inverter_temp_m1:" << feedback[mod_idx]->inverter_temp_m1
                << "\t drive_status:" << feedback[mod_idx]->drive_status << "\t ip:" << feedback[mod_idx]->ip
                << "\t serial_number:" << feedback[mod_idx]->serial_number
                << "\t connect_mode:" << feedback[mod_idx]->connect_mode << "\t model:" << feedback[mod_idx]->model
                << "\t mac_address:" << feedback[mod_idx]->mac_address
                << "\t hw_version:" << feedback[mod_idx]->hw_version
                << "\t fw_version:" << feedback[mod_idx]->fw_version
                << std::endl

                //  motion control config
                << "\t pos_gain:" << feedback[mod_idx]->motion_ctrl_config.pos_gain
                << "\t vel_gain:" << feedback[mod_idx]->motion_ctrl_config.vel_gain
                << "\t vel_integrator_gain:" << feedback[mod_idx]->motion_ctrl_config.vel_integrator_gain
                << "\t vel_limit:" << feedback[mod_idx]->motion_ctrl_config.vel_limit
                << "\t vel_limit_tolerance:" << feedback[mod_idx]->motion_ctrl_config.vel_limit_tolerance
                << std::endl

                // motor config
                << "\t current_lim:" << feedback[mod_idx]->motor_config.current_lim
                << "\t current_lim_margin:" << feedback[mod_idx]->motor_config.current_lim_margin
                << "\t inverter_temp_limit_lower:" << feedback[mod_idx]->motor_config.inverter_temp_limit_lower
                << "\t inverter_temp_limit_upper:" << feedback[mod_idx]->motor_config.inverter_temp_limit_upper
                << "\t requested_current_range:" << feedback[mod_idx]->motor_config.requested_current_range
                << "\t current_control_bandwidth:" << feedback[mod_idx]->motor_config.current_control_bandwidth
                << std::endl

                // trap traj
                << "\t accel_limit:" << feedback[mod_idx]->trap_traj.accel_limit
                << "\t decel_limit:" << feedback[mod_idx]->trap_traj.decel_limit
                << "\t vel_limit:" << feedback[mod_idx]->trap_traj.vel_limit
                << std::endl

                // network
                << "\t dhcp_enable:" << feedback[mod_idx]->network_setting.dhcp_enable
                << "\t SSID:" << feedback[mod_idx]->network_setting.SSID
                << "\t password:" << feedback[mod_idx]->network_setting.password
                << "\t name:" << feedback[mod_idx]->network_setting.name
                << "\t staticIP:" << feedback[mod_idx]->network_setting.staticIP
                << "\t gateway:" << feedback[mod_idx]->network_setting.gateway
                << "\t subnet:" << feedback[mod_idx]->network_setting.subnet
                << "\t dns_1:" << feedback[mod_idx]->network_setting.dns_1
                << "\t dns_2:" << feedback[mod_idx]->network_setting.dns_2 << std::endl;
    }
  } else {
    std::cout << "Received no feedback from group!" << std::endl;
  }
}

void verifyMotionCtrlConfig(const GroupFeedback& feedback, const std::vector<MotionControllerConfig*>& vc) {
  if (feedback.size() != vc.size()) {
    std::cout << "======= verify motion controller config size diff failed." << std::endl;
    return;
  }
  for (int i = 0; i < feedback.size(); ++i) {
    bool flag = feedback[i]->motion_ctrl_config.pos_gain == vc.at(i)->pos_gain &&
                feedback[i]->motion_ctrl_config.vel_gain == vc.at(i)->vel_gain &&
                feedback[i]->motion_ctrl_config.vel_integrator_gain == vc.at(i)->vel_integrator_gain &&
                feedback[i]->motion_ctrl_config.vel_limit == vc.at(i)->vel_limit &&
                feedback[i]->motion_ctrl_config.vel_limit_tolerance == vc.at(i)->vel_limit_tolerance;
    if (!flag) {
      std::cout << "======= verify motion controller config failed." << std::endl;
    }
  }
}
void verifyMotorConfig(const GroupFeedback& feedback, const std::vector<MotorConfig*>& vm) {
  if (feedback.size() != vm.size()) {
    std::cout << "======= verify motor config size diff failed." << std::endl;
    return;
  }
  for (int i = 0; i < feedback.size(); ++i) {
    bool flag = feedback[i]->motor_config.current_lim == vm.at(i)->current_lim &&
                feedback[i]->motor_config.current_lim_margin == vm.at(i)->current_lim_margin &&
                feedback[i]->motor_config.inverter_temp_limit_lower == vm.at(i)->inverter_temp_limit_lower &&
                feedback[i]->motor_config.inverter_temp_limit_upper == vm.at(i)->inverter_temp_limit_upper &&
                feedback[i]->motor_config.requested_current_range == vm.at(i)->requested_current_range &&
                feedback[i]->motor_config.current_control_bandwidth == vm.at(i)->current_control_bandwidth;
    if (!flag) {
      std::cout << "======= verify motor config failed." << std::endl;
    }
  }
}

void verifyTrapTraj(const GroupFeedback& feedback, const std::vector<TrapTraj*>& traj) {
  if (feedback.size() != traj.size()) {
    std::cout << "======= verify trap traj size diff failed." << std::endl;
    return;
  }
  for (int i = 0; i < feedback.size(); ++i) {
    bool flag = feedback[i]->trap_traj.accel_limit == traj.at(i)->accel_limit &&
                feedback[i]->trap_traj.decel_limit == traj.at(i)->decel_limit &&
                feedback[i]->trap_traj.vel_limit == traj.at(i)->vel_limit;
    if (!flag) {
      std::cout << "======= verify trap traj failed." << std::endl;
    }
  }
}

void verifyNetworkConfig(const GroupFeedback& feedback, const std::vector<NetworkSetting*>& str) {
  if (feedback.size() != str.size()) {
    std::cout << "======= verify network size diff failed." << std::endl;
    return;
  }
  for (int i = 0; i < feedback.size(); ++i) {
    bool flag = feedback[i]->network_setting.dhcp_enable == str.at(i)->dhcp_enable &&
                feedback[i]->network_setting.SSID == str.at(i)->SSID &&
                feedback[i]->network_setting.password == str.at(i)->password &&
                feedback[i]->network_setting.name == str.at(i)->name &&
                feedback[i]->network_setting.staticIP == str.at(i)->staticIP &&
                feedback[i]->network_setting.gateway == str.at(i)->gateway &&
                feedback[i]->network_setting.subnet == str.at(i)->subnet &&
                feedback[i]->network_setting.dns_1 == str.at(i)->dns_1 &&
                feedback[i]->network_setting.dns_2 == str.at(i)->dns_2;
    if (!flag) {
      std::cout << "======= verify network failed." << std::endl;
    }
  }
}

int main() {
  std::string str("10.10.10.255");
  Lookup lookup(&str);  // After construction,start the background thread lookup actuator
  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);  // set lookup stop

  std::shared_ptr<Group> group = lookup.getGroupFromFamily("Default");
  Pnd::GroupFeedback feedback(group->size());
  GroupCommand group_command(group->size());

  std::cout << std::endl << "group size: " << group->size() << std::endl;
  if (group->size() == 0) {
    std::cout << "No modules found, exiting" << std::endl;
    return 1;
  }
  // pndSetLogLevel("ERROR");
  auto entry_list = lookup.getEntryList();
  for (const auto& entry : *entry_list) {
    std::cout << "Name: " << entry.name_ << std::endl;
    std::cout << "Family: " << entry.family_ << std::endl;
  }
  getConfigParam(group, feedback);

  // motion control config
  std::vector<MotionControllerConfig*> vc;
  for (int i = 0; i < group->size(); ++i) {
    MotionControllerConfig* c = new MotionControllerConfig();
    c->pos_gain = 61;
    c->vel_gain = 2;
    c->vel_integrator_gain = 0.05;
    c->vel_limit = 41;
    c->vel_limit_tolerance = 1.3;
    vc.emplace_back(c);
  }
  group_command.setMotionCtrlConfig(vc);
  group->sendCommand(group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  getConfigParam(group, feedback);
  verifyMotionCtrlConfig(feedback, vc);

  // motor config
  std::vector<MotorConfig*> vm;
  for (int i = 0; i < group->size(); ++i) {
    MotorConfig* c = new MotorConfig();
    c->current_lim = 16;
    c->current_lim_margin = 5;
    c->inverter_temp_limit_lower = 81;
    c->inverter_temp_limit_upper = 91;
    c->requested_current_range = 31;
    c->current_control_bandwidth = 501;
    vm.emplace_back(c);
  }
  group_command.setMotorConfig(vm);
  group->sendCommand(group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  getConfigParam(group, feedback);
  verifyMotorConfig(feedback, vm);

  // trap traj
  std::vector<TrapTraj*> trap;
  for (int i = 0; i < group->size(); ++i) {
    TrapTraj* c = new TrapTraj();
    c->accel_limit = 121;
    c->decel_limit = 121;
    c->vel_limit = 31;
    trap.emplace_back(c);
  }
  group_command.setTrapTraj(trap);
  group->sendCommand(group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  getConfigParam(group, feedback);
  verifyTrapTraj(feedback, trap);

  // network config
  std::vector<NetworkSetting*> net;
  for (int i = 0; i < group->size(); ++i) {
    NetworkSetting* c = new NetworkSetting();
    c->dhcp_enable = false;
    strcpy(c->SSID, "PNDsemi");
    strcpy(c->password, "pnd3.1415926");
    strcpy(c->name, "test");
    strcpy(c->staticIP, ("[10,10,10,1" + std::to_string(i + 4) + "]").c_str());
    strcpy(c->gateway, "[10,10,10,1]");
    strcpy(c->subnet, "[255,255,255,0]");
    strcpy(c->dns_1, "[114,114,114,114]");
    strcpy(c->dns_2, "[8,8,8,8]");
    net.emplace_back(c);
  }
  group_command.setNetworkSetting(net);
  group->sendCommand(group_command);

  std::this_thread::sleep_for(std::chrono::milliseconds(1001));

  for (auto elem : vc) {
    delete elem;
  }
  for (auto elem : vm) {
    delete elem;
  }
  for (auto elem : trap) {
    delete elem;
  }
  for (auto elem : net) {
    delete elem;
  }

  return 0;
}
