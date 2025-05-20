#pragma once

#include "basic_function.h"
#include "joystick.h"
#include "robot_common.hpp"

class StateAnkleTestDemo : public FSMState {
 public:
  explicit StateAnkleTestDemo(RobotData *robot_data);
  ~StateAnkleTestDemo() override;

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;
};