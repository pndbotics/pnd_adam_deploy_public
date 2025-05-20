#include "groupCommand.hpp"

namespace Pnd {

GroupCommand::GroupCommand(size_t number_of_modules)
    : internal_(pndGroupCommandCreate(number_of_modules)), number_of_modules_(number_of_modules) {
  for (size_t i = 0; i < number_of_modules_; i++) commands_.emplace_back(pndGroupCommandGetModuleCommand(internal_, i));
}

GroupCommand::~GroupCommand() noexcept {
  if (internal_ != nullptr) pndGroupCommandRelease(internal_);
}

size_t GroupCommand::size() const { return number_of_modules_; }

PndCommandPtr GroupCommand::operator[](size_t index) { return commands_[index]; }

const PndCommandPtr GroupCommand::operator[](size_t index) const { return commands_[index]; }

void GroupCommand::clear() { pndGroupCommandClear(internal_); }

void GroupCommand::setPosition(const std::vector<float> &position) {
  if (position.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandPosition);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = position[i];
  }
}

void GroupCommand::setTrapezoidalMove(const std::vector<float> &val) {
  if (val.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandTrapezoidal);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = val[i];
  }
}
void GroupCommand::setVelocity(const std::vector<float> &velocity) {
  if (velocity.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandVelocity);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = velocity[i];
  }
}
void GroupCommand::setVelocityRamp(const std::vector<float> &velocity) {
  if (velocity.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandVelocityRamp);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = velocity[i];
  }
}
void GroupCommand::setCurrent(const std::vector<float> &current) {
  if (current.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandCurrent);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = current[i];
  }
}

void GroupCommand::enable(const std::vector<float> &status) {
  if (status.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandEnable);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = status[i];
  }
}

void GroupCommand::reboot(const std::vector<bool> &flag) {
  if (flag.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandReboot);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = flag[i];
  }
}

void GroupCommand::getError(const std::vector<bool> &flag) {
  if (flag.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandGetError);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = flag[i];
  }
}

void GroupCommand::clearError(const std::vector<bool> &flag) {
  if (flag.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandClearError);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = flag[i];
  }
}

void GroupCommand::resetLinearCount(const std::vector<float> &linearCount) {
  if (linearCount.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandResetLinearCount);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = linearCount[i];
  }
}

void GroupCommand::setMotionCtrlConfig(const std::vector<MotionControllerConfig *> &config) {
  if (config.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandMotionControllerConfig);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->motion_ctrl_config = config[i];
  }
}

void GroupCommand::setMotorConfig(const std::vector<MotorConfig *> &config) {
  if (config.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandMotorConfig);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->motor_config = config[i];
  }
}

void GroupCommand::setTrapTraj(const std::vector<TrapTraj *> &config) {
  if (config.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandTrapTraj);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->trap_traj = config[i];
  }
}

void GroupCommand::saveConfig(const std::vector<bool> &flag) {
  if (flag.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandSaveConfig);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = flag[i];
  }
}

void GroupCommand::setNetworkSetting(const std::vector<NetworkSetting *> &config) {
  if (config.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandNetworkSetting);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->network_setting = config[i];
  }
}

void GroupCommand::getMotorRotorAbsPos(const std::vector<bool> &flag) {
  if (flag.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandGetMotorRotorAbsPos);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = flag[i];
  }
}

void GroupCommand::setLatencyTest(const std::vector<bool> &flag) {
  if (flag.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandLatencyTest);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = flag[i];
  }
}

void GroupCommand::setInputPositionPt(const std::vector<PosPtInfo> &param) {
  if (param.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandPositionPt);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->pos_pt_info_.pos = param[i].pos;
    commands_[i]->pos_pt_info_.vel_ff = param[i].vel_ff;
    commands_[i]->pos_pt_info_.torque_ff = param[i].torque_ff;
  }
}

void GroupCommand::setInputVelocityPt(const std::vector<float> &param) {
  if (param.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandVelocityPt);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = param[i];
  }
}

void GroupCommand::setInputTorquePt(const std::vector<float> &param) {
  if (param.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandTorquePt);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = param[i];
  }
}

// RCU
void GroupCommand::RcuPower(const std::vector<bool> &status) {
  if (status.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcuPower);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = status[i];
  }
}

void GroupCommand::RcuAV5VPower(const std::vector<bool> &status) {
  if (status.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcuAV5V);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = status[i];
  }
}

void GroupCommand::RcuV5VPower(const std::vector<bool> &status) {
  if (status.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcuV5V);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = status[i];
  }
}

void GroupCommand::RcuV5VAPower(const std::vector<bool> &status) {
  if (status.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcuV5VA);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = status[i];
  }
}

void GroupCommand::RcuV5VBPower(const std::vector<bool> &status) {
  if (status.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcuV5VB);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = status[i];
  }
}

void GroupCommand::Rcu12VPower(const std::vector<bool> &status) {
  if (status.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcu12V);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = status[i];
  }
}

void GroupCommand::RcuFan12VPower(const std::vector<bool> &status) {
  if (status.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcuFan12V);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = status[i];
  }
}

void GroupCommand::Rcu19VPower(const std::vector<bool> &status) {
  if (status.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcu19V);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->bool_fields_ = status[i];
  }
}

void GroupCommand::RcuBrakeCmd(const std::vector<RcuBrake> &param) {
  if (param.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcuBrake);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->rcu_brake_.brake_enable = param[i].brake_enable;
    commands_[i]->rcu_brake_.brake_overvoltage = param[i].brake_overvoltage;
    commands_[i]->rcu_brake_.brake_factor = param[i].brake_factor;
  }
}

void GroupCommand::RcuAdcOffset(const std::vector<float> &param) {
  if (param.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndCommandRcuAdcOffset);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = param[i];
  }
}

void GroupCommand::RcuFwUpgrade() { pndGroupCommandSetType(internal_, PndCommandRcuFirmwareUpgrade); }

// Encoder
void GroupCommand::getEncoderAngle(const std::vector<float> &param) {
  if (param.size() != number_of_modules_) return;
  pndGroupCommandSetType(internal_, PndEncoderCommandGetAngle);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    commands_[i]->float_fields_ = param[i];
  }
}
}  // namespace Pnd
