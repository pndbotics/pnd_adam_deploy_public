#include "joint_interface.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "pconfig.hpp"

JointInterface::JointInterface() { joint_error_ = false; }

JointInterface::~JointInterface() = default;

void JointInterface::init(Eigen::VectorXd absolute_pos, Eigen::VectorXd Kp, Eigen::VectorXd Kd) {
  // size init
  absolute_pos_.resize(kRobotDof);
  linear_count_.resize(kRobotDof);
  control_config_.resize(kRobotDof);
  enable_status_.resize(kRobotDof);
  control_command_.resize(kRobotDof);
  lose_error_count_.resize(kRobotDof);

  // data
  q_a_.resize(kRobotDof);
  qd_a_.resize(kRobotDof);
  current_a_.resize(kRobotDof);
  torque_a_.resize(kRobotDof);

  // init value
  absolute_pos_ = absolute_pos;

  for (int i = 0; i < kRobotDof; i++) {
    linear_count_[i] = 0.0;
    control_config_[i] = new MotionControllerConfig();
    // enable motor
    enable_status_[i] = 1;
    // control command init
    control_command_[i].pos = 0.0;
    control_command_[i].vel_ff = 0.0;
    control_command_[i].torque_ff = 0.0;
  }

  for (int i = 0; i < kRobotDof; i++) {
    joint_ip_index_.insert(std::pair<std::string, int>(PConfig::getInst().ipList()[i], i));
  }

  for (int i = 0; i < kRobotDof; i++) {
    control_config_[i]->pos_gain = Kp[i];
    control_config_[i]->vel_gain = Kd[i];
    control_config_[i]->vel_integrator_gain = 0.0;
    control_config_[i]->vel_limit = 100.0;
    control_config_[i]->vel_limit_tolerance = 100.0;
    if (i == 3 || i == 9) {
      control_config_[i]->vel_limit = 25.0;
    }
  }

  // communication error
  for (int i = 0; i < kRobotDof; i++) {
    lose_error_count_[i] = 0;
  }

  // Try and get the requested group.
  std::string str("10.10.10.255");
  Pnd::Lookup lookup(&str);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  lookup.setLookupFrequencyHz(0);
  group_ = lookup.getGroupFromIps(PConfig::getInst().ipList());

  if (!group_) {
    std::cout << "No group found!" << std::endl;
    return;
  }
  assert(group_->size() == kRobotDof);
  std::cout << "group size: " << group_->size() << std::endl;

  // set error level
  pndSetLogLevel("ERROR", "ERROR");
  feedback_ = std::make_unique<Pnd::GroupFeedback>(group_->size());
  group_command_ = std::make_unique<Pnd::GroupCommand>(group_->size());

  // set linear count
  std::vector<float> linear_count(kRobotDof);
  for (int i = 0; i < kRobotDof; i++) {
    linear_count_[i] =
        absolute_pos_(i) * PConfig::getInst().jointDir()[i] * PConfig::getInst().jointGearRatio()[i] / (2.0 * M_PI);
  }

  std::map<std::string, int>::iterator it;
  int count = 0;
  for (it = joint_ip_index_.begin(); it != joint_ip_index_.end(); it++) {
    linear_count[count] = linear_count_[it->second];
    count++;
  }
  group_command_->resetLinearCount(linear_count);
  group_->sendCommand(*group_command_);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  std::cout << "set pndrive linear_count succeed" << std::endl;

  // set motor config
  std::vector<MotionControllerConfig *> control_config(kRobotDof);
  count = 0;
  for (it = joint_ip_index_.begin(); it != joint_ip_index_.end(); it++) {
    control_config[count] = control_config_[it->second];
    count++;
  }
  group_command_->setMotionCtrlConfig(control_config);
  group_->sendCommand(*group_command_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "Set pndrive motionCtrlConfig succeed" << std::endl;

  // get motor config
  group_->sendFeedbackRequest();
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  group_->getNextFeedback(*feedback_, 20);

  // start enabling Devices
  for (int i = 0; i < group_->size(); ++i) {
    enable_status_[i] = 1;
  }
  group_command_->enable(enable_status_);
  group_->sendCommand(*group_command_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "Enable pndrive succeed" << std::endl;
}

void JointInterface::setCommand(Eigen::VectorXd pos_cmd, Eigen::VectorXd vel_cmd, Eigen::VectorXd tor_cmd) {
  //
  pos_cmd = pos_cmd - absolute_pos_;
  std::map<std::string, int>::iterator it;
  int count = 0;
  for (it = joint_ip_index_.begin(); it != joint_ip_index_.end(); it++) {
    control_command_[count].pos = pos_cmd[it->second] * PConfig::getInst().jointDir()[it->second] *
                                  PConfig::getInst().jointGearRatio()[it->second] / (2.0 * M_PI);
    control_command_[count].vel_ff = vel_cmd[it->second] * PConfig::getInst().jointDir()[it->second] *
                                     PConfig::getInst().jointGearRatio()[it->second] / (2.0 * M_PI);
    control_command_[count].torque_ff =
        tor_cmd[it->second] * PConfig::getInst().jointDir()[it->second] /
        (PConfig::getInst().jointGearRatio()[it->second] * PConfig::getInst().curTorScale()[it->second]);
    count++;
  }
  group_command_->setInputPositionPt(control_command_);
  group_->sendCommand(*group_command_);
}

void JointInterface::getState(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &tor) {
  group_->getNextFeedback(*feedback_, 3);

  std::map<std::string, int>::iterator it;
  int count = 0;
  for (it = joint_ip_index_.begin(); it != joint_ip_index_.end(); it++) {
    // error code
    if ((*feedback_)[count]->error_code != 0) {
      std::cout << "Joint [" << it->second << "] error code: " << (*feedback_)[count]->error_code << std::endl;
      joint_error_ = true;
    }
    // lose communication
    if ((*feedback_)[count]->position == std::numeric_limits<float>::quiet_NaN()) {
      lose_error_count_[it->second]++;
    } else {
      lose_error_count_[it->second] = 0;
    }
    if (lose_error_count_[it->second] == tolerance_count_) {
      std::vector<bool> status(kRobotDof, 0);
      group_command_->RcuPower(status);
      group_->sendCommand(*group_command_);
      std::cout << "Joint [" << it->second << "] lose communication, ctrlBox is disabled !" << std::endl;
      joint_error_ = true;
    }
    q_a_(it->second) = (*feedback_)[count]->position * PConfig::getInst().jointDir()[it->second] * 2.0 * M_PI /
                       PConfig::getInst().jointGearRatio()[it->second];
    qd_a_(it->second) = (*feedback_)[count]->velocity * PConfig::getInst().jointDir()[it->second] * 2.0 * M_PI /
                        PConfig::getInst().jointGearRatio()[it->second];
    current_a_(it->second) = (*feedback_)[count]->current * PConfig::getInst().jointDir()[it->second];
    torque_a_(it->second) = current_a_(it->second) * PConfig::getInst().curTorScale()[it->second] *
                            PConfig::getInst().jointGearRatio()[it->second];
    count++;
  }
  q_a_ = q_a_ + absolute_pos_;
  pos = q_a_;
  vel = qd_a_;
  tor = torque_a_;
}

void JointInterface::disable() {
  for (int i = 0; i < group_->size(); ++i) {
    enable_status_[i] = 0;
  }
  group_command_->enable(enable_status_);
  group_->sendCommand(*group_command_);
  for (int i = 0; i < kRobotDof; i++) {
    std::cout << "Joint [" << i << "] disabled!" << std::endl;
  }
}
