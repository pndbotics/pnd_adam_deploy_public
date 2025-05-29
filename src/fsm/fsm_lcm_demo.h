#pragma once

#include "basic_function.h"
#include "joystick.h"
#include "robot_common.hpp"

class StateLcmDemo : public FSMState {
 public:
  explicit StateLcmDemo(RobotData *robot_data);
  ~StateLcmDemo() override;

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;
};