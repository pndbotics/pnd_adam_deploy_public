
/**
 * @file joint_interface.h
 * @brief
 * @version 0.1
 * @date 2023-06-20
 *
 * @copyright Copyright (c) 2023 PND robotics
 *
 */
#ifndef JOINT_INTERFACE_H_
#define JOINT_INTERFACE_H_
#include <fstream>
#include <map>

#include "aios.h"
#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"
#include "robot_common.hpp"

class JointInterface {
 public:
  //
  explicit JointInterface();
  ~JointInterface();

  void init(Eigen::VectorXd absolute_pos, Eigen::VectorXd Kp, Eigen::VectorXd Kd);
  void setCommand(Eigen::VectorXd pos_cmd, Eigen::VectorXd vel_cmd, Eigen::VectorXd tor_cmd);
  void getState(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &tor);
  // Disable
  void disable();

  bool joint_error_;

 private:
  // group
  std::shared_ptr<Pnd::Group> group_;
  //
  std::unique_ptr<Pnd::GroupFeedback> feedback_;
  //
  std::unique_ptr<Pnd::GroupCommand> group_command_;

  // iplist
  std::map<std::string, int> joint_ip_index_;
  //-----------absolute encoder--------------//
  Eigen::VectorXd absolute_pos_;

  // set linear count, same sort with group
  std::vector<float> linear_count_;
  // set motor config
  std::vector<MotionControllerConfig *> control_config_;
  // start enabling Devices
  std::vector<float> enable_status_;
  // control command
  std::vector<PosPtInfo> control_command_;

  // data
  Eigen::VectorXd q_a_;
  Eigen::VectorXd qd_a_;
  Eigen::VectorXd current_a_;
  Eigen::VectorXd torque_a_;

  // communication error
  std::vector<int> lose_error_count_;
  int tolerance_count_ = 3;  // default 1;
};
#endif  // JOINTINTERFACE_H_
