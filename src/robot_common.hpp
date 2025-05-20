#ifndef ROBOT_COMMON_HPP_
#define ROBOT_COMMON_HPP_

#include <torch/script.h>

#include <Eigen/Dense>
#include <vector>

#include "pnd_algorithm.h"

constexpr double kDt = 0.0025;

enum class AdamStatusCode { UNKNOW = -1, AdamStatusSuccess = 0, AdamStatusFailure = 1 };

enum FSMStateName {
  STOP = 0,
  ZERO,
  MLP,
};

class RobotCommon {
 public:
  virtual ~RobotCommon() = default;
  virtual AdamStatusCode init() = 0;
  virtual AdamStatusCode getState(double t, RobotData &robot_data) = 0;
  virtual AdamStatusCode setCommand(RobotData &robot_data) = 0;
  virtual AdamStatusCode disableAllJoints() = 0;

 public:
  Eigen::VectorXd joint_Kp_ = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_Kd_ = Eigen::VectorXd::Zero(kRobotDof);

 protected:
  Eigen::VectorXd joint_pos_ = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_vel_ = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_tau_ = Eigen::VectorXd::Zero(kRobotDof);
};

class FSMState {
 public:
  explicit FSMState(RobotData *robot_data) { robot_data_ = robot_data; };
  virtual ~FSMState() = default;

  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;
  // Run the normal behavior for the state
  virtual void run() = 0;
  // Manages state specific transitions
  virtual FSMStateName checkTransition() = 0;
  // Behavior to be carried out when exiting a state
  virtual void onExit() = 0;

  FSMStateName current_state_name_;
  torch::jit::script::Module *mlp_model_;

 protected:
  const int freq_ = 4;
  double timer_ = 0.;

  RobotData *robot_data_;
};

struct StateList {
  FSMState *zero;
  FSMState *stop;
  FSMState *mlp;
};
#endif  // ROBOT_COMMON_HPP_