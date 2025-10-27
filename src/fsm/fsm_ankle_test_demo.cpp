#include "fsm/fsm_ankle_test_demo.h"

#include "pconfig.hpp"
#include "pdata_handler.hpp"
#include "putil.h"

StateAnkleTestDemo::StateAnkleTestDemo(RobotData *robot_data) : FSMState(robot_data) {
  current_state_name_ = FSMStateName::MLP;
}

StateAnkleTestDemo::~StateAnkleTestDemo() {}

void StateAnkleTestDemo::onEnter() {
  timer_ = 0.;
  robot_data_->clip_qd_ = true;
}

void StateAnkleTestDemo::run() {
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_pos = PConfig::getInst().zeroPos();
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(kRobotDof);

  std::vector<std::string> ankle_joint_names = {"anklePitch_Left", "anklePitch_Right"};
  auto ankle_ids_ = PConfig::getInst().jointIdFromNames(ankle_joint_names);

  // set des here
  robot_data_->q_d_.tail(kRobotDof) = robot_data_->q_a_.tail(kRobotDof);
  robot_data_->q_dot_d_.tail(kRobotDof) = robot_data_->q_dot_a_.tail(kRobotDof);
  robot_data_->tau_d_.setZero();
  Eigen::VectorXd ankle_val = Eigen::VectorXd::Zero(ankle_ids_.size());
  for (size_t i = 0; i < ankle_ids_.size(); i++) {
    ankle_val[i] = 2 * sin(2 * M_PI * timer_);
  }
  setInfoFromJointIds(ankle_val, ankle_ids_, robot_data_->tau_d_);

  timer_ += kDt;

#ifdef DATALOG
  DataHandler::getInstance().cacheData(robot_data_, timer_);
#endif
}

FSMStateName StateAnkleTestDemo::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoStop") {
    std::cout << "MLP2Stop" << std::endl;
    return FSMStateName::STOP;
  } else {
    return FSMStateName::MLP;
  }
}

void StateAnkleTestDemo::onExit() {}
