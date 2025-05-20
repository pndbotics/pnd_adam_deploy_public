#pragma once

#include "basic_function.h"
#include "joystick.h"
#include "robot_common.hpp"

class StatePosDemo : public FSMState {
 public:
  explicit StatePosDemo(RobotData *robot_data);
  ~StatePosDemo() override;

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;
};