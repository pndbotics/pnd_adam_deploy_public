#ifndef REAL_ROBOT_HPP_
#define REAL_ROBOT_HPP_

#include "joint_interface.h"
#include "robot_common.hpp"
#include "vnIMU.h"

class RealRobot : public RobotCommon {
 public:
  RealRobot();
  ~RealRobot() override;
  AdamStatusCode init() override;
  AdamStatusCode getState(double t, RobotData &robot_data) override;
  AdamStatusCode setCommand(RobotData &robot_data) override;
  AdamStatusCode disableAllJoints() override;

  AdamStatusCode readAbsEncoder(Eigen::VectorXd &init_pos, Eigen::VectorXd &motor_enc_init_pos);

 private:
  std::unique_ptr<JointInterface> joint_interface_;
  vnIMU imu_;

  Eigen::VectorXd joint_Kp_s = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_Kd_s = Eigen::VectorXd::Zero(kRobotDof);
  std::vector<int> ankle_ids_;
  std::vector<int> wrist_ids_;

  Eigen::VectorXd max_tau_ = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::ArrayXd kp_mul_kd_ = Eigen::ArrayXd::Zero(kRobotDof);
  Eigen::VectorXd max_qd_ = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd min_qd_ = Eigen::VectorXd::Zero(kRobotDof);
};

#endif  // REAL_ROBOT_HPP_