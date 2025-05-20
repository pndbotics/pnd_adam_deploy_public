#pragma once

#include "basic_function.h"
#include "joystick.h"
#include "robot_common.hpp"

class StateMLP : public FSMState {
 public:
  explicit StateMLP(RobotData *robot_data);
  ~StateMLP() override;

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;

 private:
  Eigen::VectorXd input_data_mlp;
  Eigen::VectorXd output_data_mlp = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd last_action_d = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd last_action_dot_d = Eigen::VectorXd::Zero(kObsDof);

  Eigen::Vector3d command = Eigen::Vector3d::Zero();
  Eigen::Vector3d joystick_command = Eigen::Vector3d::Zero();
  Eigen::Vector3d command_scales = Eigen::Vector3d::Zero();
  Eigen::VectorXd cur_joint_pos_ = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd cur_joint_vel_ = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd action_last = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd joint_pos_last_gait = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd joint_vel_last_gait = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd def_dof_pos_;
  Eigen::VectorXd zero_pos_;

  double obs_scales_lin_vel = 2.0;
  double obs_scales_ang_vel = 0.25;
  double obs_scales_dof_pos = 1.0;
  double obs_scales_dof_vel = 0.05;
  double height_scales = 5.0;
  double action_scales = 0.5;

  double gait_cycle = 1.0;
  double left_phase_ratio = 0.;
  double right_phase_ratio = 0.;
  double left_theta_offset = 0.35;
  double right_theta_offset = 0.35;

  LowPassFilter *omega_filter;
  double predictive_time = 0.02;
  Eigen::VectorXd para_0 = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd para_1 = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd para_2 = Eigen::VectorXd::Zero(kObsDof);
  Eigen::VectorXd para_3 = Eigen::VectorXd::Zero(kObsDof);
  double timer_plan = 0.0;
  double timer_gait = 0.0;
  GaitEnum gait_d = GaitEnum::Stand;
  GaitEnum gait_a = GaitEnum::Stand;
  double trans_time = 0.3;
  double left_phase;
  double right_phase;
  double x_vel_command_offset = 0.0;
  double y_vel_command_offset = 0.0;
  float *input_array;
  std::vector<int> obs_joint_ids_;
  std::vector<int> wrist_yaw_ids_;
};