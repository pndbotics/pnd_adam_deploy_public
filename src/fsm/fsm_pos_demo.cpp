#include "fsm/fsm_pos_demo.h"

#include "pconfig.hpp"
#include "pdata_handler.hpp"

StatePosDemo::StatePosDemo(RobotData *robot_data) : FSMState(robot_data) { current_state_name_ = FSMStateName::MLP; }

StatePosDemo::~StatePosDemo() {}

void StatePosDemo::onEnter() {
  timer_ = 0.;
  robot_data_->clip_qd_ = true;
}

void StatePosDemo::run() {
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_pos = PConfig::getInst().zeroPos();
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(kRobotDof);

  // parameters for demo motion
  double time_interval = 2.0;
  auto joint_pos_motion = PConfig::getInst().jointPosMotion();
  int motion_num = joint_pos_motion.size();
  // std::cout << "motion_num:" << motion_num << std::endl;

  if (timer_ < (motion_num - 1) * time_interval) {
    int i = (int)(timer_ / time_interval);
    fifthPoly(joint_pos_motion[i], zero, zero, joint_pos_motion[i + 1], zero, zero, time_interval,
              timer_ - i * time_interval, joint_pos, joint_vel, joint_acc);
  } else {
    joint_pos = joint_pos_motion[motion_num - 1];
    joint_vel.setZero();
  }

  // set des here
  robot_data_->q_d_.tail(kRobotDof) = joint_pos;
  robot_data_->q_dot_d_.tail(kRobotDof) = joint_vel;
  robot_data_->tau_d_.setZero();

  timer_ += kDt;

#ifdef DATALOG
  DataHandler::getInstance().cacheData(robot_data_, timer_);
#endif
}

FSMStateName StatePosDemo::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoStop") {
    std::cout << "MLP2Stop" << std::endl;
    return FSMStateName::STOP;
  } else {
    return FSMStateName::MLP;
  }
}

void StatePosDemo::onExit() {}
