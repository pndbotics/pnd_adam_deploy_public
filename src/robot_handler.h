/**
 * @file RobotInterfaceImpl.h
 * @brief
 * @version 1.0
 * @date 2023-06-30
 *
 * @copyright Copyright (c) 2023 PND robotics
 *
 */

#ifndef ROBOT_HANDLER_H_
#define ROBOT_HANDLER_H_

#include <memory>

#ifdef WEBOTS
#include "webots_robot.h"
#elif MUJOCO
#include "mujoco_robot.h"
#else
#include "real_robot.h"
#endif
#include "fsm_state_impl.h"
#include "joystick.h"

class Framework {
 public:
  explicit Framework(RobotData &robot_data);
  ~Framework();
  bool init();
  void getState(double t, RobotData &robot_data);
  void setCommand(RobotData &robot_data);
  void disableAllJoints();
  void entryStop();

  void runFSM();
  FSMStateName getCurrentState();
  bool disableJoints;

 private:
  FSMState *getNextState(FSMStateName stateName) const;
  void readJointConfig();

 private:
  std::unique_ptr<RobotCommon> robot_common_;
  std::vector<int> leg_joint_ids_;
  Eigen::VectorXd leg_pos_;
  Eigen::VectorXd leg_vel_;
  Eigen::VectorXd leg_tau_;

  StateList state_list_;
  FSMState *current_state_;
  FSMState *next_state_;
  FSMStateName next_state_name_;

  bool first_run_;
  bool error_to_stop_ = false;
};

#endif  // ROBOT_HANDLER_H_
