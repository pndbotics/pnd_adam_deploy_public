#include "mujoco_robot.h"
MujocoRobotImpl::MujocoRobotImpl() : humanoid_(m, d) {}

MujocoRobotImpl::~MujocoRobotImpl() {}

AdamStatusCode MujocoRobotImpl::init() {
  return AdamStatusCode::AdamStatusSuccess;
}
AdamStatusCode MujocoRobotImpl::getState(double t, RobotData& robot_data) {
  humanoid_.readData(t, robot_state_sim_);
  joint_pos_ = robot_state_sim_.jointPosAct;
  joint_vel_ = robot_state_sim_.jointVelAct;
  joint_tau_ = robot_data.tau_d_.tail(kRobotDof);
  robot_data.q_a_.tail(kRobotDof) = joint_pos_;
  robot_data.q_dot_a_.tail(kRobotDof) = joint_vel_;
  robot_data.tau_a_.tail(kRobotDof) = joint_tau_;
  robot_data.imu_data_ = robot_state_sim_.imu9DAct;
  return AdamStatusCode::AdamStatusSuccess;
}

AdamStatusCode MujocoRobotImpl::setCommand(RobotData& robot_data) {
  for (int i = 0; i < kRobotDof; i++) {
    robot_data.tau_d_(i + 6) =
        joint_Kp_(i) * (robot_data.q_d_(i + 6) - robot_data.q_a_(i + 6)) + joint_Kd_(i) * (-robot_data.q_dot_a_(i + 6));
  }
  // robot_data.pos_mode_ = false;

  if (robot_data.pos_mode_) {
    humanoid_.setMotorPos(robot_data.q_d_.tail(kRobotDof), robot_data, joint_Kp_, joint_Kd_);
  } else {
    humanoid_.setMotorTau(robot_data.tau_d_.tail(kRobotDof));
  }

  return AdamStatusCode::AdamStatusSuccess;
}

AdamStatusCode MujocoRobotImpl::disableAllJoints() { return AdamStatusCode::AdamStatusSuccess; }
