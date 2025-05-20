#pragma once

#include "basic_function.h"
#include "joystick.h"
#include "robot_common.hpp"
class StateRosDemo : public FSMState {
 public:
  explicit StateRosDemo(RobotData *robot_data);
  ~StateRosDemo() override;

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;

  private:
  Eigen::VectorXd mlp_out_scaled = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd def_dof_pos_;
  Eigen::VectorXd zero_pos_;
  std::vector<int> obs_joint_ids_;
  std::vector<int> wrist_yaw_ids_;
};