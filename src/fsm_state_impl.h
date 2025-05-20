
#ifndef FSM_STATE_IMPL_H_
#define FSM_STATE_IMPL_H_

#include <time.h>

#include <map>

#include "basic_function.h"
#include "joystick.h"
#include "robot_common.hpp"

FSMState *FSMInit(RobotData &robot_data);

class StateZero : public FSMState {
 public:
  explicit StateZero(RobotData *robot_data);

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;

 private:
  const double total_time = 2.0;

  bool zero_finish_flag = false;
  Eigen::VectorXd init_joint_pos = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd zero_ = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_pos_ = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_vel_ = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_acc_ = Eigen::VectorXd::Zero(kRobotDof);
};

class StateStop : public FSMState {
 public:
  explicit StateStop(RobotData *robot_data);

  void onEnter() override;
  void run() override;
  FSMStateName checkTransition() override;
  void onExit() override;

 private:
  Eigen::VectorXd init_joint_pos = Eigen::VectorXd::Zero(kRobotDof);
};

class FSMFactory {
 public:
  static FSMState *createState(const std::string &state_name, RobotData *robot_data) {
    if (state_map_.find(state_name) != state_map_.end()) {
      return state_map_[state_name](robot_data);
    }
    return nullptr;
  }
  static void registerState(const std::string &state_name, std::function<FSMState *(RobotData *)> state_creator) {
    state_map_[state_name] = state_creator;
  }

 private:
  static std::map<std::string, std::function<FSMState *(RobotData *)>> state_map_;
};

#endif  // FSM_STATE_IMPL_H_
