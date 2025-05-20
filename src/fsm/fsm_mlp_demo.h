#pragma once

#include "basic_function.h"
#include "joystick.h"
#include "robot_common.hpp"

class StateMlpDemo : public FSMState {
 public:
  explicit StateMlpDemo(RobotData *robot_data);
  ~StateMlpDemo() override;

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;

 private:
  Eigen::VectorXd input_data_mlp;
  Eigen::VectorXd output_data_mlp = Eigen::VectorXd::Zero(kObsDof);
  float *input_array;
};